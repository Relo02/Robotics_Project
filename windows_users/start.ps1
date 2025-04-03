# Specify the container name and image
$CONTAINER_NAME = "robotics_container"
$IMAGE_NAME = "smentasti/robotics"

# Pull the latest image
Write-Host "Pulling the latest image: $IMAGE_NAME..."
docker pull $IMAGE_NAME

# Check if the container exists
$containerExists = docker ps -a --format "{{.Names}}" | Select-String -Pattern "^$CONTAINER_NAME$"

if ($containerExists) {
    Write-Host "Container $CONTAINER_NAME exists."

    # Check if the container is running
    $containerRunning = docker inspect -f "{{.State.Running}}" $CONTAINER_NAME 2>$null

    if ($containerRunning -eq "true") {
        Write-Host "Container $CONTAINER_NAME is running. Stopping it now..."
        docker stop $CONTAINER_NAME
        docker rm $CONTAINER_NAME
    } else {
        Write-Host "Container $CONTAINER_NAME is not running."
        docker rm $CONTAINER_NAME
    }
} else {
    Write-Host "Container $CONTAINER_NAME does not exist."
}

# Ensure the local 'data' and 'catkin_ws' folders exist
$pwd = Get-Location
$DataFolder = "$pwd\..\data"
$CatkinWsFolder = "$pwd\..\catkin_ws"

if (!(Test-Path -Path $DataFolder)) {
    New-Item -ItemType Directory -Path $DataFolder | Out-Null
}
if (!(Test-Path -Path $CatkinWsFolder)) {
    New-Item -ItemType Directory -Path $CatkinWsFolder | Out-Null
}

# Convert paths to Docker-compatible format
$DataFolderDocker = $DataFolder -replace '\\', '/'
$CatkinWsFolderDocker = $CatkinWsFolder -replace '\\', '/'

# Run the container
docker run -it `
    --user robotics `
    --env="DISPLAY=novnc:0.0" `
    --env="QT_X11_NO_MITSHM=1" `
    --net=ros `
    --rm `
    --volume="${DataFolderDocker}:/home/robotics/data" `
    --volume="${CatkinWsFolderDocker}:/home/robotics/catkin_ws" `
    --name $CONTAINER_NAME `
    -w /home/robotics `
    $IMAGE_NAME
