param(
    [string]$COM = ""
)

if ($COM -eq "") {
    Write-Host "Available COM ports:"
    [System.IO.Ports.SerialPort]::GetPortNames()
    $COM = Read-Host -Prompt "Enter the COM port for your Arduino (e.g., COM8 or just 8)"
}

if ($COM -match '^\d+$') {
    $COM = "COM$COM"
}

Write-Host "Connecting to $COM at 115200 baud..."

try {
    $port = new-Object System.IO.Ports.SerialPort $COM,115200,"None",8,"one"
    $port.Open()
    Write-Host "Connected! Press '1' to open valve, '0' to close valve."
    Write-Host "Press 'q' or Ctrl+C to quit."
    
    while ($true) {
        $key = [Console]::ReadKey($true).KeyChar
        if ($key -eq 'q') {
            break
        }
        if ($key -eq '1' -or $key -eq '0') {
            $port.Write($key.ToString())
            if ($key -eq '1') {
                Write-Host "`n[SENT] 1 - Opened Valve"
            } else {
                Write-Host "`n[SENT] 0 - Closed Valve"
            }
        }
    }
} catch {
    Write-Error "Failed to connect to $COM. Make sure the port is correct and not used by another program (like the Arduino IDE Serial Monitor)."
} finally {
    if ($port -and $port.IsOpen) {
        $port.Close()
        Write-Host "`nPort closed."
    }
}
