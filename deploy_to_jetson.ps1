# Configuration - Replace these with your Jetson's details
$JETSON_USER = "jetson"
$JETSON_IP = "192.168.0.31"
$HOST_TEMP_DIR = "~/"

Write-Host "Step 1: Transferring jetracer_ros2 to Jetson host via SCP..." -ForegroundColor Cyan
scp -r ./jetracer_ros2 ${JETSON_USER}@${JETSON_IP}:${HOST_TEMP_DIR}

Write-Host "Step 2: Copying from Jetson host into the 'my_jetracer' Docker container..." -ForegroundColor Cyan
ssh ${JETSON_USER}@${JETSON_IP} "docker cp ${HOST_TEMP_DIR}jetracer_ros2 my_jetracer:/root/ros2_ws/src/ && echo 'Successfully copied to Docker container.'"
