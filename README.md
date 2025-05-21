# dualcode-balancing-linefollower-robotA dual-core ESP32-powered robot that combines line following capabilities with a self-stabilizing platform using an MPU6500 sensor. It uses FreeRTOS to run line tracking and balancing tasks in parallel. The MPU sensor data is filtered and used to control a servo motor that maintains the platform angle at zero degrees, allowing it to carry objects while navigating along a path.

Key Features:

Infrared sensor-based line following

MPU6500-based angle detection and filtering

Servo-controlled balancing platform

Multi-core task execution using FreeRTOS

Serial output for debugging and tuning
