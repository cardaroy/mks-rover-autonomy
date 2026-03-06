import pyrealsense2 as rs

# Create a pipeline
pipeline = rs.pipeline()

# Start the pipeline
config = rs.config()
pipeline.start(config)

# Get the device
device = pipeline.get_active_profile().get_device()

# Check for the presence of IMU (i.e., accelerometer/gyro)
sensors = device.sensors
for sensor in sensors:
    if sensor.get_info(rs.camera_info.name) == "Motion Module":
        print("IMU (Motion Module) detected!")
        break
else:
    print("No IMU detected.")
