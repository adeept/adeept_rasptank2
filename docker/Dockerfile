# ROS 2 Humble para el RaspTank, en la Raspberry Pi 4 con Raspberry Pi OS (64 bits).
# Construir en la Pi:   docker compose -f docker/compose.yaml build
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-dev gcc python3-opencv python3-yaml python3-smbus i2c-tools \
      ros-humble-xacro ros-humble-robot-state-publisher ros-humble-vision-msgs \
      ros-humble-tf2-ros ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Librerías del hardware (las mismas que usa el código de Adeept en web/)
RUN pip3 install --no-cache-dir \
      adafruit-blinka adafruit-circuitpython-pca9685 adafruit-circuitpython-motor \
      RPi.GPIO gpiozero smbus2

WORKDIR /ws
COPY ros2_ws/src src
RUN . /opt/ros/humble/setup.sh \
    && colcon build \
    && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rasptank_bringup", "robot.launch.py", "camera_backend:=tcp", "camera_source:=127.0.0.1:8555"]
