# AHRS-with-clock-based-delay
IMU 9250 built in AHRS with data parsing and an encoder measurement for angle verification on a pendulum
this code uses the built in 6 axis AHRS of the IMU 9250 from sparkfun on the razer imu board
the data is them parsed then sent over serial USB to be used in MATLAB and simulink
there is an encoder libary used for checking the output angle of the IMU for accuracy
