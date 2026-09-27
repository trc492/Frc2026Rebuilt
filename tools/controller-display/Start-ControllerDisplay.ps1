$ErrorActionPreference = 'Stop'
$port = 8765
$listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback, $port)
try { $listener.Start(); $listener.Stop() } catch { throw "Port $port is already in use." }
Start-Process "http://localhost:$port"
py -m http.server $port --bind 127.0.0.1 --directory $PSScriptRoot
