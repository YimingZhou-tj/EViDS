param(
    [Parameter(Mandatory=$true)][string]$CarlaExe,
    [string]$CarlaHost = 'localhost',
    [int]$CarlaPort = 2000
)
$ErrorActionPreference = 'Stop'
function Test-CarlaReadyPort {
    $socket = [System.Net.Sockets.TcpClient]::new()
    try { $socket.Connect($CarlaHost, $CarlaPort); return $true }
    catch { return $false }
    finally { $socket.Dispose() }
}
if (Test-CarlaReadyPort) {
    Write-Output "Using CARLA already listening on ${CarlaHost}:${CarlaPort}."
    exit 0
}
if ($CarlaHost -notin @('localhost','127.0.0.1')) {
    throw 'A remote CARLA server must be started on its own host.'
}
if (-not (Test-Path -LiteralPath $CarlaExe -PathType Leaf)) {
    throw "CARLA executable not found: $CarlaExe"
}
$serverArguments = @('-quality-level=Low','-windowed','-ResX=800','-ResY=600','-nosound',"-carla-rpc-port=$CarlaPort")
$serverProcess = Start-Process -FilePath $CarlaExe -ArgumentList $serverArguments -WindowStyle Hidden -PassThru
$deadline = (Get-Date).AddSeconds(90)
while (-not (Test-CarlaReadyPort)) {
    if ((Get-Date) -ge $deadline) { throw 'CARLA did not open its RPC port within 90 seconds.' }
    Start-Sleep -Seconds 1
}
Write-Output "Started CARLA test server (launcher PID $($serverProcess.Id)). The server is left running for repeated tests."
