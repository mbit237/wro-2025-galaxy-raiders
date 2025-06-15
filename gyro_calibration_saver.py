from gyro import Gyro 
gyro = Gyro() #initialise class
print(gyro.calibration())
gyro.save_calibration()