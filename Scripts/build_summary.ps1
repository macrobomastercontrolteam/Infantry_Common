#!/usr/bin/env pwsh
# ------------------------------------------------------------------------------
# build_summary.ps1
#
# Runs the firmware build (`make`) while capturing all output to a log file, then
# prints a consolidated summary of every compiler/linker error and warning at the
# end. This solves the problem of long build output scrolling past the terminal
# scrollback buffer.
#
# Usage (from repo root or anywhere):
#   pwsh Scripts/build_summary.ps1                 # equivalent to `make`
#   pwsh Scripts/build_summary.ps1 clean all       # runs `make clean all`
#   pwsh Scripts/build_summary.ps1 DEBUG=0         # pass any make args/goals
#
# The full build log is always written to: build/build.log
# ------------------------------------------------------------------------------
[CmdletBinding()]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$MakeArgs
)

$repoRoot = Split-Path -Parent $PSScriptRoot
Push-Location $repoRoot
try {
    $buildDir = Join-Path $repoRoot 'build'
    if (-not (Test-Path $buildDir)) {
        New-Item -ItemType Directory -Path $buildDir | Out-Null
    }
    $logFile = Join-Path $buildDir 'build.log'

    Write-Host "Running: make $($MakeArgs -join ' ')" -ForegroundColor Cyan

    # Stream make's stdout+stderr to the console AND capture it to the log file.
    # $LASTEXITCODE keeps make's exit code (Tee-Object is a cmdlet, not native).
    & make @MakeArgs 2>&1 | Tee-Object -FilePath $logFile
    $makeExit = $LASTEXITCODE

    # ----------------------------------------------------------------------
    # Parse the captured log for diagnostics.
    # GCC format:  path/file.c:LINE:COL: warning|error: message [-Wflag]
    # Linker:      ... undefined reference to `sym'   /   ld: ... error ...
    # ----------------------------------------------------------------------
    $lines = @()
    if (Test-Path $logFile) {
        $lines = Get-Content -LiteralPath $logFile
    }

    $errorMatches   = $lines | Select-String -Pattern ':\s+(fatal\s+)?error:'  -AllMatches
    $warningMatches = $lines | Select-String -Pattern ':\s+warning:'           -AllMatches
    $linkerErrors   = $lines | Select-String -Pattern '(undefined reference|multiple definition|region .* overflowed|: error:|ld(\.exe)?:.*(error|cannot))'

    $errorCount   = ($errorMatches   | Measure-Object).Count
    $warningCount = ($warningMatches | Measure-Object).Count
    $linkerCount  = ($linkerErrors   | Measure-Object).Count

    Write-Host ""
    Write-Host "==================== BUILD SUMMARY ====================" -ForegroundColor Cyan

    if ($warningCount -gt 0) {
        Write-Host ""
        Write-Host "--- Warnings ($warningCount) ---" -ForegroundColor Yellow
        foreach ($m in $warningMatches) {
            Write-Host ("  {0}" -f $m.Line.Trim()) -ForegroundColor Yellow
        }
    }

    if ($errorCount -gt 0) {
        Write-Host ""
        Write-Host "--- Errors ($errorCount) ---" -ForegroundColor Red
        foreach ($m in $errorMatches) {
            Write-Host ("  {0}" -f $m.Line.Trim()) -ForegroundColor Red
        }
    }

    if ($linkerCount -gt 0) {
        Write-Host ""
        Write-Host "--- Linker issues ($linkerCount) ---" -ForegroundColor Red
        foreach ($m in $linkerErrors) {
            Write-Host ("  {0}" -f $m.Line.Trim()) -ForegroundColor Red
        }
    }

    Write-Host ""
    Write-Host "-------------------------------------------------------" -ForegroundColor Cyan
    $totalErrors = $errorCount + $linkerCount
    $statusColor = if ($makeExit -eq 0 -and $totalErrors -eq 0) { 'Green' } else { 'Red' }
    Write-Host ("Errors: {0}   Warnings: {1}   make exit code: {2}" -f $totalErrors, $warningCount, $makeExit) -ForegroundColor $statusColor
    Write-Host ("Full log: {0}" -f $logFile) -ForegroundColor DarkGray
    if ($makeExit -eq 0 -and $totalErrors -eq 0) {
        Write-Host "Build SUCCEEDED" -ForegroundColor Green
    } else {
        Write-Host "Build FAILED" -ForegroundColor Red
    }
    Write-Host "=======================================================" -ForegroundColor Cyan

    exit $makeExit
}
finally {
    Pop-Location
}
