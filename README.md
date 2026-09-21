# Projeto Robô Mecanum - ROS 2 Humble

Este repositório contém a arquitetura completa de software do Robô Mecanum omnidirecional.

## Divisão do Projeto
- **`firmware/`**: Código do Arduino Mega (Encoders e Motores).
- **`src/01_control_and_hardware/`**: Ponte serial, controle manual e launch base.
- **`src/02_gap_follower/`**: Algoritmo de desvio de obstáculos em tempo real (Gap Follower).
- **`src/03_slam_and_navigation/`**: Drivers do LiDAR COIN-D6, Câmera HP60C, EKF, SLAM e Nav2.

## Dispositivos Fixos (udev)
- Arduino Mega: `/dev/arduino`
- LiDAR COIN-D6: `/dev/lidar`
