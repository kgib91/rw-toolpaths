param([string]$Compiler = 'clang++')
$ErrorActionPreference = 'Stop'
$repoRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
$kernelRoot = Join-Path $repoRoot 'vendor/Clipper2Medial'
$nativeOutput = Join-Path $repoRoot 'RW.Toolpaths/runtimes/win-x64/native/clipper2medial.dll'
New-Item -ItemType Directory -Force -Path (Split-Path $nativeOutput) | Out-Null
& $Compiler '-std=c++17' '-O2' '-shared' ('-I' + (Join-Path $kernelRoot 'include')) `
    (Join-Path $PSScriptRoot 'medial_polygons.cpp') (Join-Path $kernelRoot 'src/clipper.engine.cpp') `
    (Join-Path $kernelRoot 'src/clipper.offset.cpp') (Join-Path $kernelRoot 'src/clipper.rectclip.cpp') '-o' $nativeOutput
if ($LASTEXITCODE -ne 0) { throw "Native medial polygon build failed ($LASTEXITCODE)." }
Get-FileHash -LiteralPath $nativeOutput -Algorithm SHA256
