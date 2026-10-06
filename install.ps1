# Registers (or with -Uninstall, unregisters) the HandEmu driver with SteamVR.
# SteamVR picks the change up the next time it starts.
param( [switch]$Uninstall )
$ErrorActionPreference = 'Stop'

$driver = Join-Path $PSScriptRoot 'driver\handemu'
$vrpath = Join-Path $env:LOCALAPPDATA 'openvr\openvrpaths.vrpath'
if ( -not ( Test-Path $vrpath ) ) { throw "SteamVR not found ($vrpath is missing). Start SteamVR once, then try again." }
$runtime = ( Get-Content -Raw $vrpath | ConvertFrom-Json ).runtime[ 0 ]
$vrpathreg = Join-Path $runtime 'bin\win64\vrpathreg.exe'
if ( -not ( Test-Path $vrpathreg ) ) { throw "vrpathreg.exe not found under $runtime" }

if ( Get-Process -Name vrserver -ErrorAction SilentlyContinue ) {
	Write-Warning 'SteamVR is running. Restart it for this change to take effect.'
}

if ( $Uninstall ) {
	& $vrpathreg removedriver $driver
	Write-Host 'HandEmu driver unregistered.'
}
else {
	if ( -not ( Test-Path ( Join-Path $driver 'bin\win64\driver_handemu.dll' ) ) ) { throw 'driver_handemu.dll is missing; run build.bat first.' }
	& $vrpathreg adddriver $driver
	Write-Host 'HandEmu driver registered. Start SteamVR through KRVR as usual.'
}
& $vrpathreg show
