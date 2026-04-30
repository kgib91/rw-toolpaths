param(
    [string] $BoostVersion    = "1.87.0",
    [string] $BoostIncludeDir = "",
    [switch] $Clean
)

$ErrorActionPreference = "Stop"
$ScriptDir = $PSScriptRoot

function Find-BoostHeaders {
    $candidates = @(
        $env:BOOST_ROOT,
        $env:BOOST_INCLUDEDIR,
        "C:\local\boost",
        "C:\boost",
        "C:\tools\boost",
        (Join-Path $ScriptDir "boost_headers")
    )
    if (Test-Path "C:\local") {
        $candidates += Get-ChildItem "C:\local" -Directory -Filter "boost_*" |
            Select-Object -ExpandProperty FullName
    }
    foreach ($c in $candidates) {
        if ($c -and (Test-Path (Join-Path $c "boost\polygon\voronoi.hpp"))) {
            return $c
        }
    }
    return $null
}

if ($BoostIncludeDir -and (Test-Path (Join-Path $BoostIncludeDir "boost\polygon\voronoi.hpp"))) {
    Write-Host "[boost] Using provided path: $BoostIncludeDir"
} else {
    $BoostIncludeDir = Find-BoostHeaders
    if ($BoostIncludeDir) {
        Write-Host "[boost] Found at: $BoostIncludeDir"
    } else {
        Write-Host "[boost] Headers not found -- downloading Boost $BoostVersion ..."
        $VersionUnder = $BoostVersion -replace "\.", "_"
        $ArchiveName  = "boost_${VersionUnder}.zip"
        $DownloadUrl  = "https://archives.boost.io/release/${BoostVersion}/source/${ArchiveName}"
        $DownloadDest = Join-Path $env:TEMP $ArchiveName
        $ExtractDir   = Join-Path $ScriptDir "boost_headers"

        if (-not (Test-Path $DownloadDest)) {
            Write-Host "[boost] Downloading $DownloadUrl ..."
            $ProgressPreference = "SilentlyContinue"
            Invoke-WebRequest -Uri $DownloadUrl -OutFile $DownloadDest -UseBasicParsing
            Write-Host "[boost] Download complete."
        } else {
            Write-Host "[boost] Using cached $DownloadDest"
        }

        Write-Host "[boost] Extracting headers..."
        Add-Type -AssemblyName System.IO.Compression.FileSystem
        $zip    = [System.IO.Compression.ZipFile]::OpenRead($DownloadDest)
        $prefix = "boost_${VersionUnder}/boost/"
        $count  = 0
        foreach ($entry in $zip.Entries) {
            if ($entry.FullName.StartsWith($prefix)) {
                $relative   = $entry.FullName.Substring("boost_${VersionUnder}/".Length)
                $targetPath = Join-Path $ExtractDir $relative
                $targetDir  = Split-Path $targetPath -Parent
                if (-not (Test-Path $targetDir)) {
                    New-Item -ItemType Directory -Path $targetDir -Force | Out-Null
                }
                if ($entry.Name -ne "") {
                    [System.IO.Compression.ZipFileExtensions]::ExtractToFile($entry, $targetPath, $true)
                    $count++
                }
            }
        }
        $zip.Dispose()
        Write-Host "[boost] Extracted $count header files into $ExtractDir"
        $BoostIncludeDir = $ExtractDir
    }
}

$BuildDir      = Join-Path $ScriptDir "build"
$InstallPrefix = (Get-Item (Join-Path $ScriptDir "..")).FullName

if ($Clean -and (Test-Path $BuildDir)) {
    Write-Host "[cmake] Cleaning $BuildDir ..."
    Remove-Item $BuildDir -Recurse -Force
}
if (-not (Test-Path $BuildDir)) { New-Item -ItemType Directory -Path $BuildDir | Out-Null }

Write-Host "[cmake] Configuring..."
cmake $ScriptDir -B $BuildDir "-DCMAKE_BUILD_TYPE=Release" "-DBOOST_INCLUDEDIR=$BoostIncludeDir" "-DCMAKE_INSTALL_PREFIX=$InstallPrefix"
if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

Write-Host "[cmake] Building..."
cmake --build $BuildDir --config Release
if ($LASTEXITCODE -ne 0) { throw "cmake build failed" }

Write-Host "[cmake] Installing..."
cmake --install $BuildDir --config Release
if ($LASTEXITCODE -ne 0) { throw "cmake install failed" }

Write-Host "[done] Native library installed:"
Get-ChildItem (Join-Path $InstallPrefix "runtimes") -Recurse -Filter "boostvoronoi.dll" | Select-Object FullName