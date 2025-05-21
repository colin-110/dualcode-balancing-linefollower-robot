# Line-Following Robot with Balancing Platform

A dual-core ESP32-based robot that follows a line using infrared sensors while balancing a platform on top using MPU6500 sensor input and a servo motor. This project demonstrates multi-core task scheduling, real-time balancing, and embedded system integration.

##  Features

- Line following using IR sensors (L298N motor driver)
- MPU6500-based platform balancing
- Real-time filtering (EMA + Moving Avg + Median)
- Servo control with angle compensation
- FreeRTOS task management (Core 0 + Core 1)
