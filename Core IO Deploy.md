# Instructions for connecting to the Spot Core I/O Payload

1. Copy the `spot_ros2` to the Core I/O Payload using `scp`:

   ```bash
   scp -P 20022 -r /path/to/spot_ros2 spot@128.148.140.23:~/spot_ros2
   ```

2. Connect to the Core I/O Payload via SSH:

   ```bash
   ssh -Y -p 20022 spot@128.148.140.23
   ```

3. Navigate to the `spot_ros2` directory: `cd ~/spot_ros2`
4. Build the Docker image:

   ```bash
   docker build -t spot_ros2 .
   ```

5. Run the Docker container:

   ```bash
   docker run --privileged -v '/home/spot/data:/ros_ws/data' -v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket -v /dev:/dev --device /dev/video0 --device /dev/video1 --device /dev/video2 --network=host -it spot_ros2 bash
   ```

6. Ensure the robot ip is set correctly in the `rooter.yaml` file located at `/ros_ws/install/spot_driver/share/spot_driver/config/rooter.yaml`. Default is `192.168.50.3`.

7. Start the Velodyne, Spot, and RealSense drivers

```bash
source install/setup.sh
ros2 run velodyne_driver velodyne_driver_node device_ip=192.168.1.201 model=VLP16 read_once=False &
ros2 run velodyne_pointcloud velodyne_transform_node --ros-args -p model:=VLP16 -p calibration:="/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml" &
ros2 launch spot_driver spot_driver.launch.py config_file:='/ros_ws/install/spot_driver/share/spot_driver/config/rooter.yaml' stitch_front_images:='True' &
ros2 launch realsense2_camera rs_launch.py &
```

(Note that this starts them as background processes. Kill them with `kill $(jobs -p)` if needed.)

Alternatively, run `bash data/run_drivers.sh` to start all drivers.

8. In another terminal, again ssh into the Core I/O Payload and run `docker exec -it <containerId> bash` to enter the Docker container (where `containerId` is found by running `docker ps`). Then run `source install/setup.sh && ros2 topic list` to ensure the drivers started successfully and the topics are being published.

---

## (Alternatively)

1. ssh into the Core I/O Payload: `ssh -p 20022 -Y spot@128.148.140.23`
2. `bash data/launch.sh`
3. In the launched docker, `source install/setup.sh`
4. `bash data/run_drivers.sh`

---

9. To record a ROS bag, run:

```bash
cd /ros_ws/src/data
ros2 bag record -o <bag_name> <topic1> <topic2> ...
```

10. To export the ROS bag to a remote machine, first exit the Docker container, then run:

```bash
docker cp <containerId>:/ros_ws/src/data/<bag_name> ~/data/<bag_name>
```

to copy the bag file from the Docker container to the host machine's `~/data` directory.

Then from the remote machine, transfer the bag from the host machine via:

```bash
scp -P 20022 -r ~/data/<bag_name> spot@REMOTE_IP_ADDRESS:~/path/to/remote/directory/
    ```

11. To view the recorded bag on the remote machine, run `source /opt/ros/humble/setup.sh && ros2 bag play --loop <bag_name>` and then in a new terminal `source /opt/ros/humble/setup.sh && ros2 run rviz2 rviz2 -f velodyne` to visualize the topics.

## Troubleshooting

### To connect the Core I/O Payload to Wifi

1. `ssh -p 20022 -Y spot@128.148.140.23`
2. `nmcli d`
3. If the wifi device is not connected, run: `nmcli r wifi on`
4. `sudo nmcli d wifi connect <WIFI_ESSID> password "<PASSWORD>"`
5. Confirm connection with `ping 8.8.8.8` and `ping google.com`. If pinging the IP address works but not the domain name, check DNS settings in `/etc/resolv.conf`.

See <https://support.bostondynamics.coms/s/article/Add-a-WiFi-Dongle-to-Spot-Core-and-Spot-Core-IO-72027> for more details.

If the docker claims to be connected to RLAB but cannot `ping 8.8.8.8`:

1. On the host, `sudo nmcli con modify "eth-robot" ipv4.method manual ipv4.never-default yes`
2. In the docker, `nmcli con modify "RLAB" ipv4.ignore-auto-dns yes ipv4.dns "8.8.8.8 8.8.4.4"`

If networking commands on the docker are very slow or ping returns "ping: sendmsg: No buffer space available":

1. `sudo ip neigh flush all`
2. `sudo ip route flush cache`
3. `sudo systemctl restart NetworkManager`

### To transfer a file to the Core I/O Payload

From the host,
```scp -P 20022 /path/to/local/file spot@128.148.140.23:~/path/to/remote/directory```

Note that the path must be a subdir of the home directory of the `spot` user on the Core I/O Payload.

### To transfer a file from the Core I/O Payload

From the docker host,
```docker cp <containerId>:/file/path/within/container /host/path/target```

Then to send the file to a remote: from the remote, cd into the target directory and run
```scp -r -P 20022 spot@128.148.140.23:/host/path/target .```

### Develop Dockerfile for Spot Core I/O

On the host machine, to target the arm64 architecture,

1. Ensure `qemu` and `qemu-user-static` are installed.
2. `docker buildx create --use --name multiarchbuilder`
3. `mkdir -p prebuilt` (needed due to limitations in docker buildx)
4. `docker run --rm --privileged multiarch/qemu-user-static --reset -p yes`

Then to actually build the image,

1. `docker buildx build -t spot_ros2 --load --platform linux/arm64 .` (If this command is failing to resolve source metadata, try `systemctl restart docker` and then run the command again.)
2. `docker save spot_ros2 | pigz > spot_ros2.tgz`
3. Connect to the same wifi as the Core I/O Payload.
3. `scp -P 20022 spot_ros2.tgz spot@128.148.140.23:~/path/to/remote/directory/spot_ros2.tgz`
4. `ssh -p 20022 -Y spot@128.148.140.23`

On the Core I/O Payload,

1. `docker load -i spot_ros2.tgz`
2. `docker run --privileged -v '/home/spot/data:/ros_ws/data' -v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket -v /dev:/dev --device /dev/video0 --device /dev/video1 --device /dev/video2 --network=host -it spot_ros2 bash`

Where

* `-v '/home/spot/data:/ros_ws/data'` mounts the directory into the docker from the host
* `-v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket` and `--network=host` allow networking
* `-v /dev:/dev --device /dev/video0 --device /dev/video1 --device /dev/video2` mounts one usb camera into the docker (for camera `i`, `\dev/video{i}` is RGB, `\dev/video{i+1}` is infrared, and `\dev/video{i+2}` is depth).

## GUIs (RVIZ, etc.)

To run GUI applications like RVIZ in the Docker container, you need to set up X11 forwarding.

```bash
docker run --privileged -v '/home/spot/data:/ros_ws/data' -v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket -v /dev:/dev --device /dev/video0 --device /dev/video1 --device /dev/video2 --network=host -e "DISPLAY=$DISPLAY" -e "QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v="$HOME/.Xauthority:/root/.Xauthority:rw" -it spot_ros2 bash 
```
