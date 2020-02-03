from __future__ import print_function

import time
import wiringpi
import serial

import tsys01
import ms5837

import adafruit_fxas21002c
import adafruit_fxos8700

from picamera import PiCamera

import subprocess

target_ascend_angle = 10 
target_descend_angle = -10
max_depth = 6

#motor params
max_pitch_motor = None
max_bouyancy_motor = None
max_roll_motor = None
pitch_motor_speed = 2
bouyancy_motor_speed = 2
roll_motor_speed = 2

pitch_second_per_unit = 8
bouyancy_second_per_unit = 8
roll_second_per_unit = 8

motor_timer = 0

pitch_motor_position = None
bouyancy_motor_position = None
roll_motor_position = None

#serial port
ser = None

#camera stuff
camera = None
# format is (x, y, w, h) as a total of the normal viewport to capture
default_zoom = (0.0, 0.0, 1.0, 1.0)
laser_zoom = (0.0, 0.3, 1.0, 0.4)
# idle time for camera to adjust white balance etc.
camera_idle_desired = 2
current_camera_idle = 0
camera_servo_pin = 18

#laser
laser_pin = 17

#sensors and values
temperature_sensor = tsys01.TSYS01()
pressure_sensor = ms5837.MS5837()
latest_depth = None
latest_temperature = None

gyro_sensor = adafruit_fxas21002c.FXAS21002C
latest_gyro_x = None
latest_gyro_y = None
latest_gyro_z = None

accel_mag_sensor = adafruit_fxos8700.FXOS8700
latest_accel_x = None
latest_accel_y = None
latest_accel_z = None
latest_mag_x = None
latest_mag_y = None
latest_mag_z = None

queueX = None
queueY = None
queueZ = None


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

    # temp_success = temperature_sensor.init()
    # bar_success = pressure_sensor.init()
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

    # print ('temperature sensor init:', temp_success)
    # print ('pressure sensor init:', bar_success)
    print ('gyro sensor init:', gyro_success)
    print ('accel mag sensor init:', accel_mag_success)
    # if temp_success and bar_success and gyro_success and accel_mag_success:
    if gyro_success and accel_mag_success:
        return True
    else:
        return False


def initialize_wiring():
    """Configure GPIO for both camera servo and laser activation."""
    wiringpi.wiringPiSetupGpio()

    # CAMERA SERVO
    # set #18 to be a PWM output
    wiringpi.pinMode(camera_servo_pin, wiringpi.GPIO.PWM_OUTPUT)

    # set the PWM mode to milliseconds stype
    wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)

    # divide down clock
    wiringpi.pwmSetClock(192)
    wiringpi.pwmSetRange(2000)

    # LASER ACTIVATION
    wiringpi.pinMode(laser_pin, 1)

    return True

def initialize_serial():
    global ser
    success = False
    # ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1)
    # ser = serial.Serial(port='/dev/ttyAMA0', baudrate=115200, timeout=1)
    # ser = serial.Serial(port='/dev/ttyS0', baudrate=115200, timeout=1)
    success = True
    return success
    

def calibrate():
    global ser
    global pitch_motor_position
    global roll_motor_position
    global bouyancy_motor_position
    x = None

    ser.write('$$\n')
    while True:    
        x = ser.readline()
        print(x)
        if x == '[\'$H\'|\'$X\' to unlock]\r\n':
            ser.write('$H\n')
            # ser.write('$X\n')
        if x == 'ok\r\n':
            break
    x = None

    time.sleep(15)

    pitch_motor_position = 0
    roll_motor_position = 0
    bouyancy_motor_position = 0
    # home the motors G28
    # ser.write('G28\n')
    

    # while True:    
    #     x = ser.read
    #     print(x)
    #     if x == 'OK':
    #         break
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
        time.sleep(2)
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
    wiringpi.digitalWrite(laser_pin,1)
    # turn on camera if necessary, wait for camera to stabilize
    if camera.closed:
        camera = PiCamera(resolution=(3280, 2464), framerate=15)
        time.sleep(2)
    # capture resized image
    camera.capture('laser.jpg', resize=(320, 240))
    # crop image for analysis

    # turn off laser
    wiringpi.digitalWrite(laser_pin,0)

    # call golang program to analyze image and return distance 
    # why go? i've made an image processing program in go before, more
    # convenient right now to repurpose that
    # try:
    #     laserDistanceStrBytes = subprocess.check_output(["laser-distance", "laser.jpg"])
    # except subprocess.CalledProcessError as e:
    #     print(e.returncode, e.output)

    # laserDistanceStr = laserDistanceStrBytes.decode("utf-8")
    # #distance returned in mm (just so we don't deal with floats)
    # laserDistance = int(laserDistanceStr)

    # measure laser position

    # determine distance based on position

# def get_gps_location():
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
    # global camera
    # camera = PiCamera(resolution=(3280, 2464), framerate=15)
    sensors_success = initialize_sensors()
    # wiring_success = initialize_wiring()
    serial_success = initialize_serial()
    # if sensors_success and wiring_success and serial_success:
    if sensors_success and serial_success:
        pressure_sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
        return True
    else:
        return False


def monitor():
    """Read all sensor inputs and store in memory"""
    global latest_temperature
    global latest_depth
    global gyro_sensor
    global latest_gyro_x
    global latest_gyro_y
    global latest_gyro_z
    global accel_mag_sensor
    global latest_accel_x
    global latest_accel_y
    global latest_accel_z
    global latest_mag_x
    global latest_mag_y
    global latest_mag_z
    # latest_temperature = get_temperature()
    # latest_depth = get_depth()
    gyro_3tuple = gyro_sensor.gyroscope
    latest_gyro_x, latest_gyro_y, latest_gyro_z = [gyro_3tuple[i] for i in range(3)]
    accel_3tuple = accel_mag_sensor.accelerometer
    latest_accel_x, latest_accel_y, latest_accel_z = [accel_3tuple[i] for i in range(3)]
    mag_3tuple = accel_mag_sensor.magnetometer
    latest_mag_x, latest_mag_y, latest_mag_z = [mag_3tuple[i] for i in range(3)]
    
    # measure distance to floor with cam and laser

def driveMotor():
    global ser
    global queueX
    global queueY
    global queueZ

    global pitch_second_per_unit
    global roll_second_per_unit

    global motor_timer

    x = None
    call = None
    
    if queueX and queueY:
        queueX = round(queueX, 4)
        queueY = round(queueY, 4)
        call = 'G0X{}Y{}\n'.format(queueX, queueY)
        rolltime = queueX * roll_second_per_unit
        pitchtime = queueY * pitch_second_per_unit
        if rolltime > pitchtime:
            motor_timer = time.time() + rolltime
        else:
            motor_timer = time.time() + pitchtime
        queueX = None
        queueY = None
    elif queueX:
        queueX = round(queueX, 4)
        call = 'G0X{}\n'.format(queueX)
        rolltime = queueX * roll_second_per_unit
        motor_timer = time.time() + rolltime
        queueX = None
    elif queueY:
        queueY = round(queueY, 4)
        call = 'G0Y{}\n'.format(queueY)
        pitchtime = queueY * pitch_second_per_unit
        motor_timer = time.time() + pitchtime
        queueY = None

    if not call:
        print('not calling')

    if call:
        print(call)
        ser.write(call)
        while True:    
            x = ser.readline()
            print(x)
            if x == 'ok\r\n':
                break


def pitchAdjust(amount):
    global queueY
    global pitch_motor_position
    print(pitch_motor_position)
    print(amount)
    queueY = amount
    pitch_motor_position += amount
    print(pitch_motor_position)
    # driveMotor('Y', amount)

def rollAdjust(amount):
    global queueX
    global roll_motor_position
    print(roll_motor_position)
    print(amount)
    queueX = amount
    roll_motor_position += amount
    print(roll_motor_position)
    # driveMotor('X', amount)

def pitchZeroFrom(accel_x):
    global pitch_motor_position
    pitch_scale = 0.5
    # pitch:  facing up = accel X +   facing down = accel X -
    desired_position = accel_x * pitch_scale
    adjustment = desired_position - pitch_motor_position
    pitchAdjust(adjustment)

def rollZeroFrom(accel_y):
    global roll_motor_position
    # roll:   left = accel Y -        right = accel Y +
    roll_scale = 0.3
    desired_position = accel_y * roll_scale
    adjustment = desired_position - roll_motor_position
    rollAdjust(adjustment)

def navigate():
    global prop_mode
    global latest_depth
    global max_depth
    global floor_detected   
    # do sensor fusion to know where we are

    # plot heading based on where we want to be and where we are, based on oper mode
    if latest_depth >= max_depth or floor_detected:
        prop_mode = ASCENDING


# def steer():
#     if prop_mode == ASCENDING:
#         # check pitch, update bouyancy motor and pitch motor to desired value

def balance():
    global latest_accel_x
    global latest_accel_y
    # pitch:  facing up = accel X +   facing down = accel X -
    # roll:   left = accel Y -        right = accel Y +
    pitchZeroFrom(latest_accel_x)
    
    rollZeroFrom(latest_accel_y)
    driveMotor()

# def float():
            

if setup():
    print('setup successful')
    calibrate()
    print('done calibrating')
else:
    # determine exactly what isn't working and if we can function.
    # If not, try to surface and call for help
    print('setup failed')

# time.sleep(.5)
# #approximate up
# # wiringpi.pwmWrite(camera_servo_pin, 55)
# #approximate middle
# wiringpi.pwmWrite(camera_servo_pin, 136)
# #approximate down
# # wiringpi.pwmWrite(camera_servo_pin, 234)
# time.sleep(.5)

# while True:
#     monitor()
#     navigate()
#     steer()
#     survey()

# position = 0

# xdistance = 1.5
# xdelay = 4

# ydistance = 1.5
# ydelay = 4

# zdistance = 4
# zdelay = 4

# call = None
# second_call = None
# the_delay = None

while True:
    monitor()
    # print('------------')
    # print('temp =', latest_temperature)
    # print('depth =', latest_depth)
    # print('gyro X =', latest_gyro_x)
    # print('gyro Y =', latest_gyro_y)
    # print('gyro Z =', latest_gyro_z)
    # print('accel X =', latest_accel_x)
    # print('accel Y =', latest_accel_y)
    # print('accel Z =', latest_accel_z)
    # print('mag X =', latest_mag_x)
    # print('mag Y =', latest_mag_y)
    # print('mag Z =', latest_mag_z)
    if time.time() >= motor_timer:
        print('Motor timer, balancing')
        # balance()
    time.sleep(0.1)

    # if position == 0:
    #     call = 'G0X{}\n'.format(xdistance)
    #     second_call = 'G0X-{}\n'.format(xdistance)
    #     the_delay = xdelay
 
    # if position == 1:
    #     call = 'G0Y{}\n'.format(ydistance)
    #     second_call = 'G0Y-{}\n'.format(ydistance)
    #     the_delay = ydelay
 
    # if position == 2:
    #     call = 'G0Z{}\n'.format(zdistance)
    #     second_call = 'G0Z-{}\n'.format(zdistance)
    #     the_delay = zdelay
 
    # if position == 3:
    #     call = 'G0X{}Y{}\n'.format(xdistance, ydistance)
    #     second_call = 'G0X-{}Y-{}\n'.format(xdistance, ydistance)
    #     the_delay = max(xdelay, ydelay)
 
    # if position == 4:
    #     call = 'G0X{}Z{}\n'.format(xdistance, zdistance)
    #     second_call = 'G0X-{}Z-{}\n'.format(xdistance, zdistance)
    #     the_delay = max(xdelay, zdelay)
 
    # if position == 5:
    #     call = 'G0Y{}Z{}\n'.format(ydistance, zdistance)
    #     second_call = 'G0Y-{}Z-{}\n'.format(ydistance, zdistance)
    #     the_delay = max(ydelay, zdelay)
 
    # if position == 6:
    #     call = 'G0X{}Y{}Z{}\n'.format(xdistance, ydistance, zdistance)
    #     second_call = 'G0X-{}Y-{}Z-{}\n'.format(xdistance, ydistance, zdistance)
    #     the_delay = max(xdelay, ydelay, zdelay)
 
    # x = None
    # ser.write(call)
    # while True:    
    #     x = ser.readline()
    #     print(x)
    #     if x == 'ok\r\n':
    #         break
    
    # time.sleep(the_delay)

    # x = None
    # ser.write(second_call)
    # while True:    
    #     x = ser.readline()
    #     print(x)
    #     if x == 'ok\r\n':
    #         break

    # time.sleep(the_delay)
    # # time.sleep(15)

    # position += 1

    # if position > 6:
    #     position = 0


    print('LOOPING!')


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
