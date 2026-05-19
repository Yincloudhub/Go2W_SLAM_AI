param(
    [string]$BackupArchive = "D:\go2_backup\go2_backup_2026-03-02_173310.tar.gz",
    [string]$StateArchive = "D:\go2_backup\go2_state_2026-03-02_173310.tar.gz",
    [string]$OutputDir = (Join-Path $PSScriptRoot "..\artifacts\recovered")
)

$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

if (-not (Test-Path -LiteralPath $BackupArchive)) {
    throw "Backup archive not found: $BackupArchive"
}

$resolvedOutputDir = [System.IO.Path]::GetFullPath($OutputDir)
New-Item -ItemType Directory -Force -Path $resolvedOutputDir | Out-Null

$patterns = @(
    "home/unitree/.bashrc",
    "home/unitree/.bash_history",
    "etc/systemd/system/unitree-upgrade.service",
    "etc/systemd/system/go2_test.service",
    "home/unitree/.ros/log/*launch.log",
    "home/unitree/.ros/log/*UnitreeSlam*.log",
    "home/unitree/.ros/log/*lio_sam_ros2_*.log",
    "home/unitree/.ros/log/*livox_ros_driver2*.log",
    "home/unitree/.ros/log/*nav2_costmap*.log",
    "home/unitree/.ros/log/*go2_pid_tracing*.log",
    "home/unitree/.ros/log/*main_process*.log",
    "home/unitree/.ros/log/*path_management*.log",
    "home/unitree/.ros/log/*graph_visual*.log",
    "home/unitree/unitree/Odometer_service/src/*"
)

$skipPatterns = @(
    "home/unitree/unitree/Odometer_service/src/opengv/include/opengv/test",
    "home/unitree/unitree/Odometer_service/src/catkin_simple/test/scenarios/hello_world/catkin_simple"
)

$archiveEntries = tar -tf $BackupArchive
$archiveVerboseEntries = tar -tvf $BackupArchive
$symlinkEntries = foreach ($line in $archiveVerboseEntries) {
    if ($line.Length -gt 0 -and $line[0] -eq "l" -and $line -match "((home|etc)/.*?)(\s+->\s+.*)?$") {
        $Matches[1]
    }
}
$symlinkEntries = $symlinkEntries | Sort-Object -Unique

$matches = foreach ($pattern in $patterns) {
    $archiveEntries | Where-Object { $_ -like $pattern }
}
$matches = $matches | Sort-Object -Unique
$matches = $matches | Where-Object { $_ -notlike "*/" }
$matches = $matches | Where-Object { $_ -notin $symlinkEntries }
$matches = $matches | Where-Object { $_ -notin $skipPatterns }

if (-not $matches) {
    throw "No matching files were found in the backup archive."
}

function Invoke-BatchedTarExtract {
    param(
        [string]$ArchivePath,
        [string]$Destination,
        [string[]]$Entries
    )

    $batch = New-Object System.Collections.Generic.List[string]
    $batchLength = 0

    foreach ($entry in $Entries) {
        $entryLength = $entry.Length + 1
        if ($batch.Count -ge 100 -or ($batchLength + $entryLength) -gt 6000) {
            tar -xf $ArchivePath -C $Destination @($batch.ToArray())
            $batch.Clear()
            $batchLength = 0
        }

        $batch.Add($entry)
        $batchLength += $entryLength
    }

    if ($batch.Count -gt 0) {
        tar -xf $ArchivePath -C $Destination @($batch.ToArray())
    }
}

Invoke-BatchedTarExtract -ArchivePath $BackupArchive -Destination $resolvedOutputDir -Entries $matches

if (Test-Path -LiteralPath $StateArchive) {
    $stateOutput = Join-Path $resolvedOutputDir "state"
    New-Item -ItemType Directory -Force -Path $stateOutput | Out-Null
    tar -xf $StateArchive -C $stateOutput
}

$summary = [PSCustomObject]@{
    OutputDir = $resolvedOutputDir
    ExtractedCount = $matches.Count
    StateArchiveExtracted = (Test-Path -LiteralPath $StateArchive)
}

$summary | Format-List
