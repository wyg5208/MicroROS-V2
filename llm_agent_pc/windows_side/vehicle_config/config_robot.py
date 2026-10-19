import serial
import struct
import sys
import time

ORDER = {
    "WIFI_SSID":            [0x01],
    "WIFI_PASSWD":          [0x02],
    "AGENT_IP":             [0x03, 0, 0, 0, 0],
    "AGENT_PORT":           [0x04, 0, 0],
    "DOMAIN_ID":            [0x06, 0, 0],
    "SERVO_OFFSET":         [0x08, 0, 0],
    "MOTOR_PID":            [0x09, 0, 0, 0, 0, 0, 0],
    "IMU_YAW_PID":          [0x0C, 0, 0, 0, 0, 0, 0],
    "VELOCITY_SCALE":       [0x0A, 0, 0, 0, 0],
    "ROS_NAMESPACE":        [0x0B],

    "CAMERA_WIFI_SSID":     [0x30],
    "CAMERA_WIFI_PASSWD":   [0x31],
    "CAMERA_AUTO_CONFIG":   [0x32, 0],
    "CAMERA_CONFIG_NOW":    [0x33, 0],
    "CAMERA_IP":            [0x34],
    "CAMERA_QUERY_IP_NOW":  [0x35, 0],

    "REBOOT_DEVICE":        [0x20, 0x00, 0x00],
    "RESET_CONFIG":         [0x21, 0x00, 0x00],
    "CONFIG_DONE":          [0x22, 0x00, 0x00],

    "REQUEST_DATA":         [0x50, 0, 0],
    "FIRMWARE_VERSION":     [0x51],
}

# Velocity calibration scales are packed as unsigned Q10000 fixed-point
# (value * 10000 into a uint16, little-endian), usable range 0 ~ 6.5535.
VELOCITY_SCALE_FACTOR = 10000
VELOCITY_SCALE_MIN = 0.1
VELOCITY_SCALE_MAX = 5.0

CONFIG_KEY_NAMES = {
    "WIFI_SSID": "CFG_SSID",
    "WIFI_PASSWD": "CFG_PASSWD",
    "CAMERA_WIFI_SSID": "CFG_CAM_SSID",
    "CAMERA_WIFI_PASSWD": "CFG_CAM_PASSWD",
}

CAMERA_APPLY_STATUS_NAMES = {
    0: "success",
    1: "SSID command failed",
    2: "password command failed",
    3: "invalid argument",
    4: "UART2 busy or internal error",
}


class MicroROS_Robot:
    """MicroROS-V2 robot configuration tool."""

    def __init__(self, port="/dev/ttyUSB0", debug=False):
        self.__ser = serial.Serial(port, 115200, timeout=0.05)
        # Disable DTR/RTS to avoid resetting ESP32 on some USB-UART adapters.
        try:
            self.__ser.setDTR(False)
            self.__ser.setRTS(False)
        except Exception:
            pass
        self.__rx_FLAG = 0
        self.__rx_COUNT = 0
        self.__rx_ADDR = 0
        self.__rx_LEN = 0
        self.__RX_BUF_LEN_MAX = 256
        self.__rx_DATA = bytearray(self.__RX_BUF_LEN_MAX)
        self.__send_delay = 0.03
        self.__read_delay = 0.15
        self.__debug = debug

        self.__HEAD = 0xFF
        self.__DEVICE_ID = 0xF8
        self.__RETURN_ID = 0xF7
        self.__READ_DATA = 0x50

    def __reset_rx_state(self):
        """Reset receive state machine."""
        self.__rx_FLAG = 0
        self.__rx_COUNT = 0
        self.__rx_ADDR = 0
        self.__rx_LEN = 0

    def __clear_rx_buffer(self):
        """Clear receive data buffer."""
        for i in range(self.__RX_BUF_LEN_MAX):
            self.__rx_DATA[i] = 0

    def __send_payload(self, order, payload=None):
        """Send a configuration frame with specified payload."""
        if payload is None:
            payload = []

        value = [int(item) & 0xFF for item in payload]
        length = len(value)
        value_sum = sum(value)
        sum_data = (self.__HEAD + self.__DEVICE_ID + (length + 0x05)
                    + order + value_sum) % 256
        tx = [self.__HEAD, self.__DEVICE_ID, (length + 0x05), order]
        tx.extend(value)
        tx.append(sum_data)
        self.__ser.write(tx)
        self.__ser.flush()
        if self.__send_delay > 0:
            time.sleep(self.__send_delay)
        if self.__debug:
            print("Send: [0x" + ', 0x'.join('{:02X}'.format(x) for x in tx) + "]")

    def __send(self, key, length=1):
        """Send a configuration frame according to ORDER table."""
        order = ORDER[key][0]
        value = []
        for i in range(0, length):
            value.append(ORDER[key][1 + i])
        self.__send_payload(order, value)

    def __send_string(self, key, text, allow_empty=False):
        """Send a string configuration item."""
        text = "" if text is None else str(text)
        text_bytes = list(text.encode("utf-8"))
        if not text_bytes:
            if allow_empty:
                text_bytes = [0]
            else:
                raise ValueError("%s cannot be empty" % key)
        self.__send_payload(ORDER[key][0], text_bytes)

    @staticmethod
    def __format_secret(text):
        """Return a plaintext password string with length for temporary debug readback."""
        if text is None:
            return "(none), len=0"
        text = str(text)
        if not text:
            return "(empty), len=0"
        return "%s, len=%d" % (text, len(text))

    def __read_string_value(self, key):
        """Read a string config item and normalize empty/missing to an empty string."""
        value = self.__read_string(key)
        return "" if value is None else value

    def __write_string_verified(self, key, text, allow_empty=False, attempts=3):
        """Write a string item and verify it by reading the same protocol address."""
        expected = "" if text is None else str(text)
        key_name = CONFIG_KEY_NAMES.get(key, key)
        actual = None

        for attempt in range(1, int(attempts) + 1):
            self.__send_string(key, expected, allow_empty=allow_empty)
            time.sleep(0.30)
            actual = self.__read_string_value(key)
            if actual == expected:
                if self.__debug:
                    print("%s verified on attempt %d" % (key_name, attempt))
                return
            if self.__debug:
                print("%s verify failed on attempt %d: expected=%r actual=%r" %
                      (key_name, attempt, expected, actual))

        raise RuntimeError("%s write verification failed: expected=%r actual=%r" %
                           (key_name, expected, actual))

    def diagnose_wifi_passwd_write(self, expected_passwd):
        """Probe CFG_PASSWD write behavior and restore the expected password if possible."""
        probe_value = "__CFG_PASSWD_PROBE__"
        original_value = self.__read_string_value("WIFI_PASSWD")

        print("CFG_PASSWD diagnostic: current=%r, expected=%r" %
              (original_value, "" if expected_passwd is None else str(expected_passwd)))

        self.__send_string("WIFI_PASSWD", probe_value, allow_empty=True)
        time.sleep(0.50)
        probe_readback = self.__read_string_value("WIFI_PASSWD")
        print("CFG_PASSWD diagnostic: probe_readback=%r" % probe_readback)

        if probe_readback == probe_value:
            print("CFG_PASSWD diagnostic: probe write worked, restoring expected password")
            self.__write_string_verified("WIFI_PASSWD", expected_passwd, allow_empty=True, attempts=5)
            return

        print("CFG_PASSWD diagnostic: probe write failed; firmware/NVS did not update CFG_PASSWD")
        if original_value != probe_readback:
            print("CFG_PASSWD diagnostic: readback changed unexpectedly from %r to %r" %
                  (original_value, probe_readback))

    def __request(self, addr, param=0):
        """Request one configuration item."""
        order = self.__READ_DATA
        buf_len = 7
        sum_data = (self.__HEAD + self.__DEVICE_ID + buf_len + order
                    + addr + param) % 256
        tx = [self.__HEAD, self.__DEVICE_ID, buf_len, order, addr, param, sum_data]
        self.__reset_rx_state()
        self.__clear_rx_buffer()
        self.__ser.flushInput()
        self.__ser.flushOutput()
        self.__ser.write(tx)

        # Retry up to five times. Only accept the response frame for the requested address
        # and ignore boot logs or stale frames from previous requests.
        for _ in range(5):
            time.sleep(self.__read_delay)
            if self.__unpack(addr):
                return True
        if self.__debug:
            print("Read: [0x" + ', 0x'.join('{:02X}'.format(x) for x in tx) + "]")
        return False

    def __unpack(self, expected_addr=None):
        """Parse firmware response frame.

        When expected_addr is None, the first valid frame is returned. Otherwise,
        only the frame with the requested address is accepted, and other valid
        frames are discarded to avoid stale delayed responses corrupting reads.
        """
        n = self.__ser.inWaiting()
        if n:
            data_array = self.__ser.read_all()
            if self.__debug:
                print("rx_data: [0x" + ', 0x'.join('{:02X}'.format(x) for x in data_array) + "]")
            for data in data_array:
                if self.__rx_FLAG == 0:
                    if data == self.__HEAD:
                        self.__rx_FLAG = 1
                    else:
                        self.__rx_FLAG = 0
                elif self.__rx_FLAG == 1:
                    if data == self.__RETURN_ID:
                        self.__rx_FLAG = 2
                    elif data == self.__HEAD:
                        self.__rx_FLAG = 1
                    else:
                        self.__rx_FLAG = 0
                elif self.__rx_FLAG == 2:
                    self.__rx_LEN = data
                    if self.__rx_LEN < 5 or self.__rx_LEN - 5 > self.__RX_BUF_LEN_MAX:
                        self.__reset_rx_state()
                    else:
                        self.__rx_FLAG = 3
                elif self.__rx_FLAG == 3:
                    self.__rx_ADDR = data
                    self.__rx_FLAG = 4 if self.__rx_LEN > 5 else 5
                    self.__rx_COUNT = 0
                    self.__clear_rx_buffer()
                elif self.__rx_FLAG == 4:
                    if self.__rx_COUNT < self.__rx_LEN - 5:
                        self.__rx_DATA[self.__rx_COUNT] = data
                        self.__rx_COUNT = self.__rx_COUNT + 1
                    if self.__rx_COUNT >= (self.__rx_LEN - 5):
                        self.__rx_FLAG = 5
                elif self.__rx_FLAG == 5:
                    data_len = self.__rx_LEN - 5
                    current_addr = self.__rx_ADDR
                    current_payload = bytes(self.__rx_DATA[:data_len])
                    rx_CHECK = sum(current_payload)
                    rx_CHECK = (self.__HEAD + self.__RETURN_ID
                                + self.__rx_LEN + current_addr
                                + rx_CHECK) % 256
                    self.__reset_rx_state()
                    if data == rx_CHECK:
                        if expected_addr is None or current_addr == expected_addr:
                            self.__clear_rx_buffer()
                            self.__rx_ADDR = current_addr
                            self.__rx_DATA[:data_len] = current_payload
                            return True
                        if self.__debug:
                            print("Ignore stale frame addr=0x%02X, expected=0x%02X" %
                                  (current_addr, expected_addr))
                    else:
                        self.__clear_rx_buffer()
        return False

    def __read_version_once(self):
        """Read firmware version once."""
        if not self.__request(ORDER["FIRMWARE_VERSION"][0]):
            return None
        return "%d.%d.%d" % (self.__rx_DATA[0], self.__rx_DATA[1],
                              self.__rx_DATA[2])

    def wait_config_ready(self, timeout=10.0, interval=0.2):
        """Wait until firmware config window is ready."""
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            version = self.__read_version_once()
            if version is not None and version != "0.0.0":
                if self.__debug:
                    print("config window ready, firmware version:", version)
                return True
            time.sleep(float(interval))
        return False

    # Configuration write

    def set_wifi_config(self, ssid, passwd):
        """Set main controller Wi-Fi."""
        self.__write_string_verified("WIFI_SSID", ssid, allow_empty=False)
        try:
            self.__write_string_verified("WIFI_PASSWD", passwd, allow_empty=True)
        except RuntimeError:
            self.diagnose_wifi_passwd_write(passwd)
            raise

    def set_udp_config(self, ip, port):
        """Set micro-ROS Agent UDP address."""
        ORDER["AGENT_IP"][1] = int(ip[0]) & 0xFF
        ORDER["AGENT_IP"][2] = int(ip[1]) & 0xFF
        ORDER["AGENT_IP"][3] = int(ip[2]) & 0xFF
        ORDER["AGENT_IP"][4] = int(ip[3]) & 0xFF
        self.__send("AGENT_IP", length=4)
        ORDER["AGENT_PORT"][1] = int(port) & 0xFF
        ORDER["AGENT_PORT"][2] = int(port >> 8) & 0xFF
        self.__send("AGENT_PORT", length=2)

    def set_ros_domain_id(self, domain_id):
        """Set ROS_DOMAIN_ID."""
        if domain_id > 100:
            domain_id = 100
        if domain_id < 0:
            domain_id = 0
        value_s = bytearray(struct.pack('h', int(domain_id)))
        ORDER["DOMAIN_ID"][1] = value_s[0]
        ORDER["DOMAIN_ID"][2] = value_s[1]
        self.__send("DOMAIN_ID", length=2)

    def set_pwm_servo_offset(self, servo_id, offset):
        """Set PWM servo offset."""
        ORDER["SERVO_OFFSET"][1] = int(servo_id) & 0xFF
        ORDER["SERVO_OFFSET"][2] = int(offset) & 0xFF
        self.__send("SERVO_OFFSET", length=2)

    def set_motor_pid_parm(self, pid_p, pid_i, pid_d):
        """Set motor PID parameters."""
        pid_p_s = bytearray(struct.pack('h', int(pid_p * 100)))
        pid_i_s = bytearray(struct.pack('h', int(pid_i * 100)))
        pid_d_s = bytearray(struct.pack('h', int(pid_d * 100)))
        ORDER["MOTOR_PID"][1] = pid_p_s[0]
        ORDER["MOTOR_PID"][2] = pid_p_s[1]
        ORDER["MOTOR_PID"][3] = pid_i_s[0]
        ORDER["MOTOR_PID"][4] = pid_i_s[1]
        ORDER["MOTOR_PID"][5] = pid_d_s[0]
        ORDER["MOTOR_PID"][6] = pid_d_s[1]
        self.__send("MOTOR_PID", length=6)

    def set_imu_yaw_pid_parm(self, pid_p, pid_i, pid_d):
        """Set IMU yaw PID parameters."""
        pid_p_s = bytearray(struct.pack('h', int(pid_p * 100)))
        pid_i_s = bytearray(struct.pack('h', int(pid_i * 100)))
        pid_d_s = bytearray(struct.pack('h', int(pid_d * 100)))
        ORDER["IMU_YAW_PID"][1] = pid_p_s[0]
        ORDER["IMU_YAW_PID"][2] = pid_p_s[1]
        ORDER["IMU_YAW_PID"][3] = pid_i_s[0]
        ORDER["IMU_YAW_PID"][4] = pid_i_s[1]
        ORDER["IMU_YAW_PID"][5] = pid_d_s[0]
        ORDER["IMU_YAW_PID"][6] = pid_d_s[1]
        self.__send("IMU_YAW_PID", length=6)

    def set_ros_namespace(self, ros_namespace):
        """Set ROS namespace."""
        self.__write_string_verified("ROS_NAMESPACE", ros_namespace, allow_empty=True)

    def set_camera_wifi_config(self, ssid, passwd):
        """Set ESP32 camera Wi-Fi."""
        self.__write_string_verified("CAMERA_WIFI_SSID", ssid, allow_empty=False)
        self.__write_string_verified("CAMERA_WIFI_PASSWD", passwd, allow_empty=True)

    def set_camera_auto_config(self, enable):
        """Set ESP32 camera startup auto-config switch."""
        ORDER["CAMERA_AUTO_CONFIG"][1] = 1 if bool(enable) else 0
        self.__send("CAMERA_AUTO_CONFIG", length=1)

    def apply_camera_wifi_config_now(self):
        """Apply ESP32 camera Wi-Fi only, without querying IP."""
        self.__reset_rx_state()
        self.__clear_rx_buffer()
        self.__ser.flushInput()
        self.__ser.flushOutput()
        self.__send("CAMERA_CONFIG_NOW", length=1)

        end_time = time.time() + 22.0
        while time.time() < end_time:
            time.sleep(self.__read_delay)
            if self.__unpack(ORDER["CAMERA_CONFIG_NOW"][0]):
                status = int(self.__rx_DATA[0])
                if status == 0:
                    return
                status_name = CAMERA_APPLY_STATUS_NAMES.get(status, "unknown error")
                raise RuntimeError("camera Wi-Fi apply failed: %s (status=%d)" %
                                   (status_name, status))

        raise RuntimeError("camera Wi-Fi apply timed out waiting for firmware response")

    def apply_camera_config_now(self):
        """Backward-compatible API: apply camera Wi-Fi only, without querying IP."""
        self.apply_camera_wifi_config_now()

    def query_camera_ip_now(self, wait_timeout=12.0):
        """Query ESP32 camera IP only, without setting Wi-Fi."""
        self.__reset_rx_state()
        self.__clear_rx_buffer()
        self.__ser.flushInput()
        self.__ser.flushOutput()
        self.__send("CAMERA_QUERY_IP_NOW", length=1)

        end_time = time.time() + float(wait_timeout)
        while time.time() < end_time:
            time.sleep(self.__read_delay)
            if self.__unpack(ORDER["CAMERA_IP"][0]):
                if self.__rx_DATA[0] != 0:
                    return bytes(self.__rx_DATA).decode('utf-8', errors='ignore').rstrip('\x00')
                return None
        return None

    def set_velocity_scale(self, linear_scale, angular_scale, attempts=3):
        """Set velocity calibration scales and verify them by readback."""
        linear_scale = float(linear_scale)
        angular_scale = float(angular_scale)
        if not VELOCITY_SCALE_MIN <= linear_scale <= VELOCITY_SCALE_MAX:
            raise ValueError("linear scale must be in [%.1f, %.1f]" %
                             (VELOCITY_SCALE_MIN, VELOCITY_SCALE_MAX))
        if not VELOCITY_SCALE_MIN <= angular_scale <= VELOCITY_SCALE_MAX:
            raise ValueError("angular scale must be in [%.1f, %.1f]" %
                             (VELOCITY_SCALE_MIN, VELOCITY_SCALE_MAX))

        linear_q10000 = int(round(linear_scale * VELOCITY_SCALE_FACTOR))
        angular_q10000 = int(round(angular_scale * VELOCITY_SCALE_FACTOR))
        payload = struct.pack('<HH', linear_q10000, angular_q10000)
        expected = (linear_q10000 / float(VELOCITY_SCALE_FACTOR),
                    angular_q10000 / float(VELOCITY_SCALE_FACTOR))
        actual = None

        for attempt in range(1, int(attempts) + 1):
            self.__send_payload(ORDER["VELOCITY_SCALE"][0], payload)
            time.sleep(0.30)
            actual = self.read_velocity_scale()
            if actual == expected:
                if self.__debug:
                    print("velocity scales verified on attempt %d" % attempt)
                return actual

        raise RuntimeError(
            "velocity scale write verification failed: expected=%r actual=%r" %
            (expected, actual))




    def reboot_device(self):
        """Hardware-reset device via USB-UART control lines."""
        try:
            # Use DTR/RTS to trigger ESP32-S3 reset. Keep the pulse sequence deterministic to avoid entering download mode.
            self.__ser.setDTR(False)
            self.__ser.setRTS(True)
            time.sleep(0.10)
            self.__ser.setDTR(True)
            self.__ser.setRTS(True)
            time.sleep(1.80)
            self.__ser.flushInput()
            self.__ser.flushOutput()
        except Exception as exc:
            raise RuntimeError("hardware reboot failed: %s" % exc) from exc

    def auto_enter_config_window(self, ready_timeout=12.0, interval=0.2):
        """Reset controller and wait for config window."""
        self.reboot_device()
        time.sleep(0.30)
        self.__reset_rx_state()
        self.__clear_rx_buffer()
        self.__ser.flushInput()
        self.__ser.flushOutput()

        # After reset, device has a 2s probe window. Send 0x23 repeatedly until firmware responds.
        deadline = time.time() + float(ready_timeout)
        while time.time() < deadline:
            self.__send_payload(0x23)
            time.sleep(0.15)
            version = self.__read_version_once()
            if version is not None and version != "0.0.0":
                return True
            time.sleep(float(interval))
        return False

    def reset_config(self):
        """Reset configuration."""
        self.__send("RESET_CONFIG", length=2)

    # Configuration readback

    def __read_string(self, addr):
        """Read a string configuration item."""
        if not self.__request(ORDER[addr][0]):
            return None
        if self.__rx_DATA[0] != 0:
            return bytes(self.__rx_DATA).decode('utf-8', errors='ignore').rstrip('\x00')
        return None

    def read_wifi_ssid(self):
        return self.__read_string("WIFI_SSID")

    def read_wifi_passwd(self):
        return self.__read_string("WIFI_PASSWD")

    def read_agent_ip_addr(self):
        if not self.__request(ORDER["AGENT_IP"][0]):
            return None
        return "%d.%d.%d.%d" % (self.__rx_DATA[0], self.__rx_DATA[1],
                                 self.__rx_DATA[2], self.__rx_DATA[3])

    def read_agent_ip_port(self):
        if not self.__request(ORDER["AGENT_PORT"][0]):
            return None
        port = struct.unpack('H', bytearray(self.__rx_DATA[0:2]))[0]
        return "%d" % port

    def read_ros_domain_id(self):
        if not self.__request(ORDER["DOMAIN_ID"][0]):
            return None
        return "%d" % struct.unpack('h', bytearray(self.__rx_DATA[0:2]))[0]

    def read_pwm_servo_offset(self):
        for _ in range(3):
            if not self.__request(ORDER["SERVO_OFFSET"][0]):
                time.sleep(0.05)
                continue
            s1 = struct.unpack('b', bytearray(self.__rx_DATA[0:1]))[0]
            s2 = struct.unpack('b', bytearray(self.__rx_DATA[1:2]))[0]
            return "%d, %d" % (s1, s2)
        return None

    def read_motor_pid_parm(self):
        if not self.__request(ORDER["MOTOR_PID"][0]):
            return None
        p = struct.unpack('h', bytearray(self.__rx_DATA[0:2]))[0] / 100.0
        i = struct.unpack('h', bytearray(self.__rx_DATA[2:4]))[0] / 100.0
        d = struct.unpack('h', bytearray(self.__rx_DATA[4:6]))[0] / 100.0
        return "%.2f, %.2f, %.2f" % (p, i, d)

    def read_imu_yaw_pid_parm(self):
        if not self.__request(ORDER["IMU_YAW_PID"][0]):
            return None
        p = struct.unpack('h', bytearray(self.__rx_DATA[0:2]))[0] / 100.0
        i = struct.unpack('h', bytearray(self.__rx_DATA[2:4]))[0] / 100.0
        d = struct.unpack('h', bytearray(self.__rx_DATA[4:6]))[0] / 100.0
        return "%.2f, %.2f, %.2f" % (p, i, d)

    def read_ros_namespace(self):
        return self.__read_string("ROS_NAMESPACE")

    def read_velocity_scale(self):
        """Read velocity calibration scales as (linear, angular) floats."""
        if not self.__request(ORDER["VELOCITY_SCALE"][0]):
            return None
        linear_q10000, angular_q10000 = struct.unpack(
            '<HH', bytearray(self.__rx_DATA[0:4]))
        return (linear_q10000 / float(VELOCITY_SCALE_FACTOR),
                angular_q10000 / float(VELOCITY_SCALE_FACTOR))

    def read_camera_wifi_ssid(self):
        """Read ESP32 camera Wi-Fi SSID."""
        return self.__read_string("CAMERA_WIFI_SSID")

    def read_camera_wifi_passwd(self):
        """Read ESP32 camera Wi-Fi password."""
        return self.__read_string("CAMERA_WIFI_PASSWD")

    def read_camera_auto_config(self):
        """Read ESP32 camera auto-config switch."""
        if not self.__request(ORDER["CAMERA_AUTO_CONFIG"][0]):
            return None
        return int(self.__rx_DATA[0])

    def read_camera_ip(self):
        """Read ESP32 camera IP."""
        return self.__read_string("CAMERA_IP")

    def read_version(self):
        version = self.__read_version_once()
        if version is None:
            version = self.__read_version_once()
        return version

    def print_all_firmware_parm(self):
        """Print all firmware parameters matching the firmware's
        app_config_print_runtime_info() output layout."""
        SEP = "=============================================="
        print("\n" + SEP)
        print("current firmware info")
        print(SEP)
        print("Yahboom")
        print("Name: MicroROS-V2")

        version = self.read_version()
        print("firmware_version:", version if version else "1.0.0")

        wifi_name = self.read_wifi_ssid()
        print("wifi_name:", wifi_name if wifi_name else "(default)")

        camera_ip = self.read_camera_ip()
        print("camera_ip:", camera_ip if camera_ip else "(none)")

        ip_addr = self.read_agent_ip_addr()
        print("agent_ip_addr:", ip_addr if ip_addr else "0.0.0.0")

        ip_port = self.read_agent_ip_port()
        print("agent_ip_port:", ip_port if ip_port else "0")

        domain_id = self.read_ros_domain_id()
        print("ros_domain_id:", domain_id if domain_id else "0")

        ros_namespace = self.read_ros_namespace()
        print("ros_namespace:", ros_namespace if ros_namespace else "(none)")

        servo_offset = self.read_pwm_servo_offset()
        if servo_offset:
            parts = servo_offset.split(", ")
            print("servo_1_offset:", parts[0] if len(parts) > 0 else "0")
            print("servo_2_offset:", parts[1] if len(parts) > 1 else "0")
        else:
            print("servo_1_offset: 0")
            print("servo_2_offset: 0")

        motor_pid = self.read_motor_pid_parm()
        print("motor_pid:", motor_pid if motor_pid else "0.00, 0.00, 0.00")

        imu_pid = self.read_imu_yaw_pid_parm()
        print("imu_yaw_pid:", imu_pid if imu_pid else "0.00, 0.00, 0.00")

        print(SEP + "\n")


if __name__ == '__main__':
    # ── Configuration mode ──
    # Accept "default" or "custom" as a command-line argument.
    # If not specified, defaults to "custom".
    if len(sys.argv) >= 2 and sys.argv[1] in ("default", "custom"):
        config_mode = sys.argv[1]
    elif len(sys.argv) >= 2:
        print("Usage: python3 config_robot.py [default|custom]")
        raise SystemExit(1)
    else:
        config_mode = "custom"

    robot = MicroROS_Robot(port='/dev/ttyUSB0', debug=False)

    # Reset the controller via DTR/RTS and wait until the config window becomes responsive.
    time.sleep(0.5)
    robot._MicroROS_Robot__ser.flushInput()
    robot._MicroROS_Robot__ser.flushOutput()

    if not robot.auto_enter_config_window(ready_timeout=10.0, interval=0.2):
        print("ERROR: firmware config window is not responding.")
        print("Automatic reboot/config-window entry failed. Please check the USB serial connection and try again.")
        raise SystemExit(1)

    if config_mode == "default":
        # Restore firmware factory defaults by erasing NVS and writing
        # hardcoded default values.  The device will auto-reboot, so we
        # re-enter the config window to read back and verify.
        robot.reset_config()
        print("Factory reset issued. Waiting for device to reboot...")
        time.sleep(2.5)
        robot._MicroROS_Robot__ser.flushInput()
        robot._MicroROS_Robot__ser.flushOutput()
        if not robot.auto_enter_config_window(ready_timeout=10.0, interval=0.2):
            print("ERROR: firmware config window is not responding after reset.")
            print("Please check the USB serial connection and try again.")
            raise SystemExit(1)

        robot.print_all_firmware_parm()
        print("Factory defaults restored. Parameters take effect immediately.")
        robot._MicroROS_Robot__send_payload(0x22)

    elif config_mode == "custom":
        # Modify the following main controller parameters.
        # Main controller Wi-Fi SSID.
        main_wifi_ssid = ""
        
        # Main controller Wi-Fi password.
        main_wifi_passwd = ""

        # micro-ROS Agent IP address.
        agent_ip_addr = [192, 168, 12, 28]

        # micro-ROS Agent UDP port.
        agent_ip_port = 8090

        # ROS_DOMAIN_ID parameter.
        ros_domain_id = 40

        # ROS namespace parameter.
        ros_namespace = ""
	
        # PWM servo 1 offset.
        servo_1_offset = -5

        # PWM servo 2 offset.
        servo_2_offset = 0

        # Motor PID parameters.
        motor_pid = (1.8, 0.12, 0.0)

        # IMU yaw PID parameters.
        imu_yaw_pid = (2.4, 0.0, 0.16)

        # True: apply camera Wi-Fi now and again after every controller reset.
        camera_auto_config = True

        print("Configuring...")

        robot.set_wifi_config(main_wifi_ssid, main_wifi_passwd)
        robot.set_camera_wifi_config(main_wifi_ssid, main_wifi_passwd)
        robot.set_udp_config(agent_ip_addr, agent_ip_port)
        robot.set_ros_domain_id(ros_domain_id)
        robot.set_ros_namespace(ros_namespace)
        robot.set_pwm_servo_offset(1, servo_1_offset)
        robot.set_pwm_servo_offset(2, servo_2_offset)
        robot.set_motor_pid_parm(motor_pid[0], motor_pid[1], motor_pid[2])
        robot.set_imu_yaw_pid_parm(imu_yaw_pid[0], imu_yaw_pid[1], imu_yaw_pid[2])

        print("Enabling camera Wi-Fi auto-config...")
        robot.set_camera_auto_config(camera_auto_config)
        time.sleep(0.30)
        expected_camera_auto_config = 1 if camera_auto_config else 0
        actual_camera_auto_config = robot.read_camera_auto_config()
        if actual_camera_auto_config != expected_camera_auto_config:
            raise RuntimeError(
                "camera auto-config verification failed: expected=%d actual=%r" %
                (expected_camera_auto_config, actual_camera_auto_config))

        print("Applying Wi-Fi credentials to camera...")
        robot.apply_camera_wifi_config_now()
        print("Camera Wi-Fi credentials accepted.")

        # Wait at least 1s after applying ESP32 camera Wi-Fi, then query camera IP.
        time.sleep(2)
        camera_ip = None
        for i in range(5):
            camera_ip = robot.query_camera_ip_now(wait_timeout=4.0)
            if camera_ip:
                break
            if i < 4:
                time.sleep(2)
        if not camera_ip:
            print("Warning: camera IP query failed after 5 retries")

        # Readback verification before closing config window
        robot.print_all_firmware_parm()
        print("Configuration complete! Parameters take effect immediately.")

        robot._MicroROS_Robot__send_payload(0x22)
