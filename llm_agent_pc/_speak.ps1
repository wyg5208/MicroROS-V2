param(
  [Parameter(Mandatory=$true)][string]$Path,
  [string]$Voice = '',
  [string]$Out = ''
)
# Speaks the UTF-8 text in $Path through SAPI, or writes a WAV to $Out instead of making sound.
# The text comes from a file, never the command line: an upstream shell mangles non-ASCII
# arguments before they land here (bit us in part 6 of the dev log).
Add-Type -AssemblyName System.Speech
$s = New-Object System.Speech.Synthesis.SpeechSynthesizer
if ($Voice -ne '') {
  try { $s.SelectVoice($Voice) } catch { "select voice failed: $($_.Exception.Message)" | Out-Host }
}
$txt = [System.IO.File]::ReadAllText($Path, [System.Text.Encoding]::UTF8)
$s.Rate = 0
if ($Out -ne '') {
  $s.SetOutputToWaveFile($Out)
  $s.Speak($txt)
  $s.SetOutputToNull()
} else {
  $s.SetOutputToDefaultAudioDevice()
  $s.Speak($txt)
}
