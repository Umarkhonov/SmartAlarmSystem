# SmartAlertSystem

## Project Overview

The **SmartAlertSystem** is an IoT-based project designed to enhance safety by providing real-time alerts to users in the event of a gas leak, flood, or fire. The system also includes a temperature sensor that continuously monitors and sends the current temperature to both a Telegram bot and an Android application. The MQTT broker is hosted on an AWS EC2 instance and managed using Docker.

## Features

- **Gas Leak Detection**: Notifies users immediately when a gas leak is detected.
- **Flood Detection**: Alerts users in the event of flooding.
- **Fire Detection**: Sends notifications upon detecting a fire.
- **Temperature Monitoring**: Continuously measures and reports the current temperature.
- **Real-time Notifications**: Sends alerts via a Telegram bot and an Android application.
- **MQTT Broker**: Custom-built broker hosted on AWS EC2 using Docker for reliable message transmission.

## System Components

1. **Sensors**
    - Gas Sensor
    - Flood Sensor
    - Fire Sensor
    - Temperature Sensor

2. **Communication**
    - MQTT Broker hosted on AWS EC2
    - Telegram Bot for instant messaging
    - Android Application for notifications and monitoring

## Getting Started

### Prerequisites

- **Hardware**:
  - Gas, flood, fire, and temperature sensors
  - Microcontroller (e.g., Arduino, ESP8266)
  - AWS EC2 instance

- **Software**:
  - Docker
  - MQTT Broker (e.g., Mosquitto)
  - Telegram Bot API token
  - Android Studio (for Android application)

### Setting Up the MQTT Broker

1. **Launch AWS EC2 Instance**:
   - Create and configure an AWS EC2 instance (Ubuntu recommended).

2. **Install Docker**:
   ```bash
   sudo apt update
   sudo apt install docker.io
   sudo systemctl start docker
   sudo systemctl enable docker
3. Run MQTT Broker using Docker:
  - Create a docker-compose.yml file with the following content:
   ```yaml
   version: '3'
    services:
    mqtt-broker:
    image: eclipse-mosquitto
    ports:
      - "1883:1883"
      - "9001:9001"
    volumes:
      - ./mosquitto/config:/mosquitto/config
      - ./mosquitto/data:/mosquitto/data
      - ./mosquitto/log:/mosquitto/log
  ```
  - Start the MQTT Broker
    Run the following command in your terminal to start the MQTT broker:
  ```bash
  sudo docker-compose up -d
  ```

