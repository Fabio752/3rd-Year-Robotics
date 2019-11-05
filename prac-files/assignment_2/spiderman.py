import brickpi3
import time
BP = brickpi3.BrickPi3()
gain_value = 1
default_speed = -180
speed_vals = [0] * 5
BP.set_motor_limits(BP.PORT_A, 70, 200) #Power limit 70%, speed limit 200 dps)

#Set sensor type
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)
time.sleep(2)
def find_speed(speed_vals, new_speed):
    if len(speed_vals) > 5:
        speed_vals.pop()
    speed_vals.insert(1,new_speed)

    #Find median
    return sorted(speed_vals) [2]

try:
    while True:
        #Measure the distance to object in front
        distance = BP.get_sensor(BP.PORT_1) # Reading in cm
        movement_distance = 30 - distance
        control_speed = find_speed(speed_vals, gain_value * movement_distance)
        print("Measured distance: %s, Difference: %s, Control speed: %s" % (distance, movement_distance, control_speed))

        #Use velocity control
        BP.set_motor_dps(BP.PORT_A, default_speed + control_speed)
        BP.set_motor_dps(BP.PORT_B, default_speed - control_speed)
	time.sleep(2)

	#BP.set_motor_dps(BP.PORT_A, default_speed - control_speed)
	#BP.set_motor_dps(BP.PORT_B, default_speed + control_speed)
	#time.sleep(2)
except KeyboardInterrupt:
    BP.reset_all()
    #BP.set_motor_dps(BP.PORT_A, 0)
    #BP.set-motor_dps(BP.PORT_B, 0)
