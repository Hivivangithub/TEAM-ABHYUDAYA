# sensors.py - stubs for IMU/barometer/compass
def read_imu():
    return {'accel': (0,0,9.8), 'gyro': (0,0,0)}

def read_barometer():
    return {'pressure':101325, 'altitude_m': 10.0}

def read_compass():
    return {'heading_deg': 0.0}
