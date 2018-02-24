# MPU6050_Kalman
MPU6050 and Kalman filter with accelerometer and gyroscope.

CoCoox project using I2C on MPU6050

SCL - PB6
SDA - PB7

In function TIM2_IRQHandler() KalAngleX, KalAngleY giving pitch and roll of MPU.
Variable "timer" should shows time between measurements data from MPU. 
