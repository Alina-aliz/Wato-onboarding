param(
    [ValidateSet('Build','Up','Down','Logs','Goal','Check','Diagnose','Trace')]
    [string]$Action = 'Check',
    [double]$X = -6,
    [double]$Y = 2
)
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$env:MONO_DIR = $projectRoot.Replace('\','/')
$env:TAG = 'navigation'
$env:ROBOT_IMAGE = 'wato-local/robot'
$env:GAZEBO_SERVER_IMAGE = 'wato-local/gazebo'
$env:INFRASTRUCTURE_FOXGLOVE_IMAGE = 'wato-local/foxglove'
$env:BASE_IMAGE_OVERRIDE = 'ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04'
$env:FOXGLOVE_BRIDGE_PORT = '8765'
$env:GAZEBO_PORT = '9002'
$composeArgs = @('compose','--project-name','wato-asd','--profile','deploy')
foreach ($module in @('robot','gazebo','vis_tools')) {
    $composeArgs += @('-f', (Join-Path $projectRoot "modules/docker-compose.$module.yaml"))
}
switch ($Action) {
    'Build' { & docker @composeArgs build }
    'Up' { & docker @composeArgs up -d }
    'Down' { & docker @composeArgs down }
    'Logs' { & docker @composeArgs logs --tail 100 -f }
    'Check' { & docker @composeArgs config --quiet }
    'Diagnose' {
        $culture = [System.Globalization.CultureInfo]::InvariantCulture
        $goalX = $X.ToString('R', $culture)
        $goalY = $Y.ToString('R', $culture)
        $command = "source /opt/watonomous/setup.bash && python3 - --inspect $goalX $goalY"
        Get-Content -Raw (Join-Path $projectRoot 'tools/diagnose.py') |
            & docker @composeArgs exec -T robot bash -lc $command
        if ($LASTEXITCODE -ne 0) { throw 'ROS diagnostic failed.' }
        & docker @composeArgs logs --tail 35 robot gazeboserver
    }
    'Trace' {
        if ([double]::IsNaN($X) -or [double]::IsInfinity($X) -or
            [double]::IsNaN($Y) -or [double]::IsInfinity($Y)) {
            throw 'Goal coordinates must be finite numbers.'
        }
        $culture = [System.Globalization.CultureInfo]::InvariantCulture
        $goalX = $X.ToString('R', $culture)
        $goalY = $Y.ToString('R', $culture)
        $command = "source /opt/watonomous/setup.bash && python3 - --goal $goalX $goalY"
        Get-Content -Raw (Join-Path $projectRoot 'tools/diagnose.py') |
            & docker @composeArgs exec -T robot bash -lc $command
        if ($LASTEXITCODE -ne 0) { throw 'ROS trace failed.' }
        & docker @composeArgs logs --tail 12 robot
    }
    'Goal' {
        if ([double]::IsNaN($X) -or [double]::IsInfinity($X) -or
            [double]::IsNaN($Y) -or [double]::IsInfinity($Y)) {
            throw 'Goal coordinates must be finite numbers.'
        }
        $culture = [System.Globalization.CultureInfo]::InvariantCulture
        $goalX = $X.ToString('R', $culture)
        $goalY = $Y.ToString('R', $culture)
        $command = "source /opt/watonomous/setup.bash && ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped '{header: {frame_id: sim_world}, point: {x: $goalX, y: $goalY, z: 0.0}}'"
        & docker @composeArgs exec -T robot bash -lc $command
    }
}
if ($LASTEXITCODE -ne 0) { throw "Docker command failed (exit $LASTEXITCODE)." }
if ($Action -eq 'Up') {
    Write-Host 'Open https://app.foxglove.dev and connect to ws://localhost:8765'
    Write-Host 'Import config/wato_asd_training_foxglove_config .json and select sim_world as the fixed frame.'
}
