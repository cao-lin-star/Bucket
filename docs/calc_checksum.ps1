# Calculate checksum for the 30-byte foot-bath robot communication frame.
# Checksum rule: sum data[2] through data[28], keep the low 8 bits as data[29].

param(
  [Parameter(ValueFromRemainingArguments = $true)]
  [string[]]$InputBytes,

  [switch]$UpdateFrame
)

function Show-Usage {
  Write-Host 'Usage:'
  Write-Host '  .\docs\calc_checksum.ps1 55 AA 02 B1 00 ... 00'
  Write-Host '  .\docs\calc_checksum.ps1 "55 AA 02 B1 00 ... 00"'
  Write-Host '  "55 AA 02 B1 ..." | .\docs\calc_checksum.ps1'
  Write-Host ''
  Write-Host 'Notes:'
  Write-Host '  - Input 29 bytes to generate data[29].'
  Write-Host '  - Input 30 bytes to verify existing data[29].'
  Write-Host '  - Use -UpdateFrame to print the full frame with corrected data[29].'
}

$text = ($InputBytes -join ' ')
if ([string]::IsNullOrWhiteSpace($text) -and -not [Console]::IsInputRedirected) {
  Show-Usage
  exit 1
}

if ([string]::IsNullOrWhiteSpace($text) -and [Console]::IsInputRedirected) {
  $text = [Console]::In.ReadToEnd()
}

$tokens = [regex]::Matches($text, '(?i)(?:0x)?[0-9a-f]{1,2}') | ForEach-Object { $_.Value }

if (($tokens.Count -ne 29) -and ($tokens.Count -ne 30)) {
  Write-Error "Expected 29 or 30 bytes, got $($tokens.Count)."
  Show-Usage
  exit 1
}

$frame = New-Object byte[] 30
for ($i = 0; $i -lt $tokens.Count; $i++) {
  $hex = $tokens[$i] -replace '(?i)^0x', ''
  $value = [Convert]::ToInt32($hex, 16)
  if (($value -lt 0) -or ($value -gt 255)) {
    Write-Error "Byte[$i] is out of range: $($tokens[$i])"
    exit 1
  }
  $frame[$i] = [byte]$value
}

$sum = 0
for ($i = 2; $i -le 28; $i++) {
  $sum += $frame[$i]
}

$checksum = $sum -band 0xFF
$frame[29] = [byte]$checksum

Write-Host ('Checksum data[29] = 0x{0:X2} ({1})' -f $checksum, $checksum)
Write-Host ('Sum data[2..28] = 0x{0:X}' -f $sum)

if ($tokens.Count -eq 30) {
  $actual = [Convert]::ToInt32(($tokens[29] -replace '(?i)^0x', ''), 16)
  if ($actual -eq $checksum) {
    Write-Host 'Verify: OK'
  } else {
    Write-Host ('Verify: FAIL, input data[29] = 0x{0:X2}, expected 0x{1:X2}' -f $actual, $checksum)
  }
}

if ($UpdateFrame -or ($tokens.Count -eq 29)) {
  $hexFrame = ($frame | ForEach-Object { '{0:X2}' -f $_ }) -join ' '
  Write-Host 'Frame with checksum:'
  Write-Host $hexFrame
}
