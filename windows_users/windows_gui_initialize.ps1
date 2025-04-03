# Check if the container is already running
$running = docker ps --filter "name=novnc" --format "{{.Names}}"

if ($running -eq "novnc") {
    Write-Host "The container 'novnc' is already running."
    exit 1
}

# Run the container
docker run -d --rm --net=ros `
   --env="DISPLAY_WIDTH=3000" --env="DISPLAY_HEIGHT=1800" --env="RUN_XTERM=no" `
   --name=novnc -p=8080:8080 `
   theasp/novnc:latest

Write-Host "The container 'novnc' has been started."
exit 0
