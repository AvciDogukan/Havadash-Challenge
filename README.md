# Havadash Firmware Engineering Challenge

This repository contains the solution for the **Havadash Firmware
Engineering Challenge**.\
It implements a **Virtual Drone Node** that generates synthetic
telemetry data and publishes it to a cloud MQTT broker using **ROS 2
Humble**, **Python 3.10**, **Paho MQTT**, and **Docker**.

## 📂 Project Structure

The project follows a standard ROS 2 Python package architecture:

    havadash_challenge/
    ├── src/
    │   └── havadash_bridge/
    │       ├── havadash_bridge/
    │       │   ├── __init__.py
    │       │   ├── mock_publisher.py     # Telemetry simulation node
    │       │   └── mqtt_bridge.py        # MQTT bridge node
    │       ├── launch/
    │       │   └── havadash_launch.py    # ROS 2 launch configuration
    │       ├── resource/
    │       ├── test/
    │       ├── package.xml
    │       ├── setup.cfg
    │       └── setup.py
    ├── Dockerfile
    ├── docker-compose.yml
    ├── entrypoint.sh
    └── README.md

## Architecture Overview

Two asynchronous ROS 2 nodes:

### 1. Mock Publisher (`mock_publisher.py`)

-   Role: Acts as a hardware abstraction layer since sensors are not available.
-   Simulates GPS using Random Walk (sensor/gps)
-   Simulates battery using linear discharge (sensor/battery)
-   Frequency : Publishes at 1 Hz

### 2. MQTT Bridge (`mqtt_bridge.py`)

-   Role: Acts as the communication gateway between the drone and the cloud.   
-   Subscribes to ROS topics
-   Converts data to JSON (Frozen Contract)
-   Publishes to external MQTT broker via Paho MQTT v2
-   Handles network exceptions safely

##  Setup & Installation

This project is fully containerized to ensure a consistent environment (Ubuntu 22.04 + ROS 2 Humble) regardless of the host OS.

### Prerequisites

-   Docker & Docker Compose

### Quick Start

-   Build and Start the Container: Run the following command in the root directory to build the image and start the environment:

``` bash
docker-compose up -d --build
```

-   Access the Development Environment: Enter the running container shell:

``` bash
docker exec -it havadash_dev_env bash
```

Launch the System: Inside the container, execute the launch file to start both the simulation and the bridge nodes:

``` bash
ros2 launch havadash_bridge havadash_launch.py
```

### Configuration & Assumptions
-   Safety: The bridge node includes exception handling to prevent crashes in case of network interruptions.

-   JSON Schema: The output format strictly follows the "Frozen" contract specified in the challenge documentation, ensuring compatibility with the Havadash Backend .

-   Paho MQTT v2: The code implements the CallbackAPIVersion.VERSION2 flag to ensure compatibility with the latest Paho MQTT library updates.

-   MQTT Broker: The system is configured to use the public broker test.mosquitto.org by default. This is parameterized in the launch file and can be changed without modifying the source code.

## Author

Dogukan Avci --- Firmware / Embedded Software Engineer Candidate
