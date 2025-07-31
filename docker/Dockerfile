FROM zed_ros2_l4t_35.4.1_sdk_5.0.0:latest

# Set up the workspace
WORKDIR /root/ros2_ws

#Add missing dependencies
RUN apt-get update -y || true && \
  apt-get clean && \
  apt-get install -y libasio-dev libpcl-dev &&\
  rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/ros2/rosbag2.git -b humble && \
    git clone https://github.com/ros2/message_filters.git -b humble && \
    git clone https://github.com/ros-perception/pcl_msgs.git -b ros2 && \
    git clone https://github.com/ros-perception/perception_pcl


# Add ublox gps package
RUN git clone https://github.com/gokulp01/ros2-ublox-zedf9p.git src/ros2-ublox-zedf9p/

#Add multi camera zed package
RUN git clone https://github.com/stereolabs/zed-ros2-examples.git

#Add Livox ros2 package
RUN git clone https://github.com/Livox-SDK/livox_ros2_driver.git

WORKDIR /root/ros2_ws

# Source the existing ROS 2 setup and build the new package
RUN /bin/bash -c "source /opt/ros/humble/install/setup.bash && source install/setup.bash && \
  colcon build --parallel-workers $(nproc) --symlink-install --packages-select pcl_msgs message_filters \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
  ' --no-warn-unused-cli' "


RUN /bin/bash -c "source /opt/ros/humble/install/setup.bash && source install/setup.bash && \
  colcon build --parallel-workers $(nproc) --symlink-install --packages-up-to ublox ublox_gps ublox_msgs ublox_serialization zed_multi_camera livox_ros2_driver \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
  ' --no-warn-unused-cli' "
  
#Add additional files

COPY ./additional/master_launch /root/ros2_ws/src/master_launch


COPY ./additional/dynamic_lidar_launch.py /root/ros2_ws/src/livox_ros2_driver/livox_ros2_driver/launch
COPY ./additional/front_lidar_config.json ./additional/rear_lidar_config.json  /root/ros2_ws/src/livox_ros2_driver/livox_ros2_driver/config/

WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/install/setup.bash && source install/setup.bash && \
  colcon build --parallel-workers $(nproc) --symlink-install --packages-select master_launch livox_ros2_driver"



COPY ./additional/zed_camera.launch.py /root/ros2_ws/src/zed_wrapper/launch
COPY ./additional/common_stereo.yaml /root/ros2_ws/src/zed_wrapper/config/common_stereo.yaml
COPY ./additional/zed_multi_camera.launch.py /root/ros2_ws/src/zed-ros2-examples/tutorials/zed_multi_camera/launch/zed_multi_camera.launch.py

RUN echo "source /opt/ros/humble/install/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc


CMD ["bash"]



