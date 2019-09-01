from __future__ import print_function

import time
import wiringpi
import serial

import tsys01
import ms5837

import adafruit_fxas21002c
import adafruit_fxos8700

from picamera import PiCamera

target_ascend_angle = 10 
target_descend_angle = -10
max_depth = 6

#motor params
max_pitch_motor
max_bouyancy_motor
max_roll_motor
pitch_motor_speed = 2
bouyancy_motor_speed = 2
roll_motor_speed = 2

#serial port
ser = None

#camera stuff
camera = None
# idle time for camera to adjust white balance etc.
camera_idle_desired = 2
current_camera_idle = 0

#sensors and values
temperature_sensor = tsys01.TSYS01()
pressure_sensor = ms5837.MS5837()
latest_depth = None
latest_temperature = None

gyro_sensor = adafruit_fxas21002c.FXAS21002C
latest_gyro = None

accel_mag_sensor = adafruit_fxos8700.FXOS8700
latest_accel = None
latest_mag = None


#propulsion mode values
prop_mode = None
ASCENDING, DESCENDING, LEVEL = range(0,3)

#operation mode values
oper_mode = None
SURFACE, DETECT_BOTTOM, SURVEY = range(0,3)

def initialize_sensors():
    """Executes each sensor's init function and prints success status."""
    global gyro_sensor
    global accel_mag_sensor

    temp_success = temperature_sensor.init()
    bar_success = pressure_sensor.init()
    gyro_success = None
    try:
        gyro_sensor = adafruit_fxas21002c.FXAS21002C(gyro_range=500)
        gyro_success = True
    except:
        gyro_success = False

    accel_mag_success = None
    try:
        accel_mag_sensor = adafruit_fxos8700.FXOS8700()
        accel_mag_success = True
    except:
        accel_mag_success = False

    print ('temperature sensor init:', temp_success)
    print ('pressure sensor init:', bar_success)
    print ('gyro sensor init:', gyro_success)
    print ('accel mag sensor init:', accel_mag_success)
    if temp_success and bar_success and gyro_success and accel_mag_success:
        return True
    else:
        return False


def initialize_wiring():
    """Configure GPIO for both camera servo and laser activation."""
    wiringpi.wiringPiSetupGpio()

    # CAMERA SERVO
    # set #18 to be a PWM output
    wiringpi.pinMode(18, wiringpi.GPIO.PWM_OUTPUT)

    # set the PWM mode to milliseconds stype
    wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)

    # divide down clock
    wiringpi.pwmSetClock(192)
    wiringpi.pwmSetRange(2000)

    # LASER ACTIVATION
    wiringpi.pinMode(17, 1)

    return True

def initialize_serial():
    global ser
    success = False
    try:
        ser = serial.Serial(port='/dev/ttyS0', baudrate=115200, timeout=1)
        success = True
    except:
        success = False
    
    return success
    

def calibrate():
    x = None
    # home the motors G28
    ser.write('G28\n')
    while True:    
        x = ser.read
        print(x)
        if x == 'OK':
            break
    x = None
    # set zero when done G92
    ser.write('G92\n')
    while True:
        x = ser.read
        print(x)
        if x == 'OK':
            break
    x = None
    # set absolute positioning G90
    ser.write('G90\n')
    while True:
        x = ser.read
        print(x)
        if x == 'OK':
            break
    x = None
    # set all motors to halfway 
    # slowly increase axis to determine maximum
    counter = 1
    while True:
        ser.write('G1 X%d\n' % (counter))
        print('X at %d\n'%(counter))
        sleep(1)
        counter += 1

def get_temperature():
    """Make the sensor update values, then read newest value"""
    temperature_sensor.read()
    temperature = temperature_sensor.temperature()
    return temperature


def get_depth():
    """Make the sensor update values, then read newest value"""
    pressure_sensor.read()
    depth = pressure_sensor.depth()
    return depth

def measure_laser_distance():
    global camera
    # turn on laser
    wiringpi.digitalWrite(17,1)
    # turn on camera if necessary, wait for camera to stabilize
    if camera.closed:
        camera = PiCamera(resolution=(3280, 2464), framerate=15)
        time.sleep(2)
    # capture resized image
    camera.capture('laser.jpg', resize=(320, 240))
    # crop image for analysis

    # measure laser position

    # determine distance based on position

def get_gps_location():
    # turn on gps

    # while loop asking for status every 5 seconds until 3d location fix

    # get location info (in lat/long minutes seconds), altitude, speed, heading

    # convert lat/long minutes seconds to lat/long decimals

    # get another reading after a minute

    # turn off gps

    # return the data as 2 tuples

def capture_image():
    global camera
    # turn on camera if necessary
    if camera.closed:
        camera = PiCamera(resolution=(3280, 2464), framerate=15)
        time.sleep(2)
    

def setup():
    global camera
    camera = PiCamera(resolution=(3280, 2464), framerate=15)
    sensors_success = initialize_sensors()
    wiring_success = initialize_wiring()
    serial_success = initialize_serial()
    if sensors_success and wiring_success and serial_success:
        pressure_sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
        return True
    else:
        return False


def monitor():
    """Read all sensor inputs and store in memory"""
    global latest_temperature
    global latest_depth
    global latest_gyro
    global gyro_sensor
    global latest_accel
    global latest_mag
    global accel_mag_sensor
    latest_temperature = get_temperature()
    latest_depth = get_depth()
    latest_gyro = gyro_sensor.gyroscope
    latest_accel = accel_mag_sensor.accelerometer
    latest_mag = accel_mag_sensor.magnetometer
    
    # measure distance to floor with cam and laser



def navigate():
    # do sensor fusion to know where we are

    # plot heading based on where we want to be and where we are, based on oper mode
    if latest_depth >= max_depth or floor_detected:
        prop_mode = ASCENDING


def steer():
    if prop_mode == ASCENDING:
        # check pitch, update bouyancy motor and pitch motor to desired value


if setup():
    print('setup successful')
else:
    # determine exactly what isn't working and if we can function.
    # If not, try to surface and call for help
    print('setup failed')

time.sleep(.5)
#approximate up
# wiringpi.pwmWrite(18, 55)
#approximate middle
wiringpi.pwmWrite(18, 136)
#approximate down
# wiringpi.pwmWrite(18, 234)
time.sleep(.5)

while True:
    monitor()
    navigate()
    steer()
    survey()


while True:
    monitor()
    print('------------')
    print(latest_temperature)
    print(latest_depth)
    print(latest_gyro)
    print(latest_accel)
    print(latest_mag)
    wiringpi.digitalWrite(17,1)
    time.sleep(.5)
    wiringpi.digitalWrite(17,0)
    time.sleep(.5)


#MAIN LOOP
# Monitor
#   Update and read sensors, store in memory
#   Capture image with laser on, calculate distance
# Interval camera?
#   Calc camera orientation based on pitch
#   Capture image
# Navigate
#   If too deep (either by reaching 40m or laser sensed floor), start ascent
#   If not at optimal angle of attack, adjust pitch
#
#   Compare measured position with expected position on the route
#   If not near position, if it's still on the route, do nothing?
#   If not near position, adjust roll in relation to current angle and route direction?
#
#   If ascending and still capturing images, descend a couple meters?? after ascent depth
#   If ascending to get to the surface, level at the surface but stay buoyant
# Communicate
#   If at the surface, wait for GPS lock and steady GPS coordinates
#   Store and send GPS coordinates with timestamp via SIM, using web sms or whatever
#   Wait for web sms or whatever reply or further instruction, for about 30 seconds
# Resume capturing
#   Descend until laser senses floor (further distance so capturing can start) or max depth reached, start capturing images
#
#

#TERMS
# Ascend/Descend
#   Alter buoyancy and change pitch
#
#


#Adjust roll motor based on current and desired heading (adjust then level, sinoid?)

#Do accelerometer, gyro and magnetometer calculations

#When GPS coordinates are updated, adjust positions from since previous update
#Use Kalman filter to unify the position inputs
