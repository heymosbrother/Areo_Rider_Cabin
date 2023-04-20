# Arduino Code Log

## Development Status
### Components

- angle_detect

    `In progress`

    Need calibration to measure the real angle

- encoder_test

    `Finished`

    Read encoder ticks and detect the rotation direction

- L298N_test

    `Finished`

- Mpu6050

    `Finished`

    Read acceleration of the 6 axes, and use digital low-pass filter to ease the noise

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

- ver_1_1_0

    `Not started`

    Need to add foil-lift motor control, posture detection funcitonalities

