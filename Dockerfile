FROM osrf/ros:humble-desktop

# Gerekli paketlerin kurulumu: Python pip ve MQTT kutuphanesi
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Python MQTT kutuphanesini kur
RUN pip3 install paho-mqtt

# Calisma dizini
WORKDIR /ros2_ws

# Entrypoint scriptini kopyala ve yetki ver
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
