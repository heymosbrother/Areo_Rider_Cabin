# Arduino Code Log

## Development Status
### Components

- angle_detect

    `Suspended`

    Need calibration to measure the real 
    
    May have a better solution using *quaternion*

- angle_quaternion

    `In progress`

    Use quaternain function in MPU6050 to get row, pitch and yaw without another new 9-axis IMU

- encoder_test

    `Finished`

    Read encoder ticks and detect the rotation direction

- foil_lift

    `Finished`

    Control the lift mechanism trigger to rotate after a designated time

- L298N_test

    `Finished`

- Mpu6050

    `Finished`

    Read acceleration of the 6 axes, and use digital low-pass filter to ease the noise

- PID_motor_OOP

    `In progress`

    Use OOP to simplify the code and development, cannot fix static problem yet

- PIDvelocity2motors

    `Finished`

    Use PID to control the motors' rpm, motor 2 needs another set of PID parameter (problem found so far)

- PIDvelocityControl

    `Finished`

    Single motor velocity control

- scanI2Cdevice

    `Finished`

    Check if there's any I2C device connected to Arduino, if so return the port number

### Combination

- ver_1_0_0

    `Finished, need futher settings`

    Fix 2 motor velocity control bug and merge it with MPU6050 code to implement straight line driving

    **4/19**: Add 3 second buffer after switch on; motor velocity control fixed; straight finished but need to PID parameters in person
    
    **4/20**: PID Drive straight: Kp = 1, Ki = Kd = 0; 

    Since the wheels are too slipery, the functionality is not ideal
    
    May need another 9-axis IMU to complete the task

- ver_1_1_0

    `In progress`

    Need to add foil-lift motor control, posture detection funcitonalities

    **4/20**: foil-lift finished, posture detection not started
