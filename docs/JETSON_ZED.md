# Jetson Orin NX / ZED integration target

This repository is validated with synthetic sensors on x86_64. No Jetson or ZED hardware has been used to validate it.

## Replacement boundary

1. Keep the mission manager and Nav2 interfaces.
2. Replace `sim_world` with the actual base driver, wheel odometry and sensors.
3. Use the official [Stereolabs ROS 2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper) for ZED data. Verify actual topic names, frames, units and QoS from the running driver.
4. Either feed an appropriate registered depth image plus CameraInfo into an adapted projector, or feed the wrapper's point cloud directly into the Nav2 obstacle layer.
5. Replace the fixed level-camera projection assumption with calibrated TF extrinsics. Never assign `base_link` to optical-frame points without transforming them.
6. Confirm the base's command interface and independent stopping behavior before any motion.

## Container boundary

The repository Dockerfile is a CPU simulation image, not a ZED SDK / JetPack image. Hardware deployment needs an image whose JetPack/L4T, CUDA and ZED SDK versions match the device. Refer to the [official Jetson Docker instructions](https://docs.stereolabs.com/docs/integrations/docker/install-on-nvidia-jetson).

The ZED model is intentionally unspecified. USB ZED cameras and GMSL2 ZED X hardware have different connection/driver requirements. The camera model must be checked on the actual device.

## Evidence to collect when hardware becomes available

Record the exact Jetson module, JetPack/L4T, camera model, SDK and wrapper versions, camera calibration, frames, depth rate, point-cloud rate, processing latency, resource use, dropped frames and failure cases. Until then, do not label simulated measurements as edge-device performance.
