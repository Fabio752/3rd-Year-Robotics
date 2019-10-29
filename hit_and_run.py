import brickpi3
import time
import random
BP = brickpi3.BrickPi3()

speed_dps = 50

def get_encode_length(distance):
	return (distance/39.1)*819.5

def get_rotation_amount (degree):
	return (degree/138.7) * 360

def go_forward(distance, speed_dps):
	BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
	BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
	#Initializations
	target_degree_rotation = get_encode_length(distance)
	actual_degree_rotation = 0
	BP.set_motor_dps(BP.PORT_A, speed_dps)
	BP.set_motor_dps(BP.PORT_B, speed_dps)

	print("target degree rotation %s" % target_degree_rotation)
	while actual_degree_rotation < target_degree_rotation:
		actual_degree_rotation = (BP.get_motor_encoder(BP.PORT_A) + BP.get_motor_encoder(BP.PORT_B))/2
                #print("actual degree rotation: %s" % actual_degree_rotation)
		print_robot_stats(BP.PORT_A)
		print_robot_stats(BP.PORT_B)
                time.sleep(0.02)
        print("Motion complete")


def rotate(degree_amount, speed_dps):
	#Rotate robot left by 90 degrees
	actual_degree = 0
	BP.set_motor_dps(BP.PORT_A, -speed_dps)
	BP.set_motor_dps(BP.PORT_B, speed_dps)
	target_rotation = get_rotation_amount(degree_amount)
	print("target rotation %s" % target_rotation)
	while actual_degree < target_rotation:
		actual_degree = BP.get_motor_encoder(BP.PORT_B)
        	print_robot_stats(BP.PORT_A)
		print_robot_stats(BP.PORT_B)
		time.sleep(0.02)
	print("Rotation complete")

# BP.set_sensor_type configures the BrickPi3 for a specific sensor.
# BP.PORT_1 specifies that the sensor will be on sensor port 1.
# BP.SENSOR_TYPE.TOUCH specifies that the sensor will be a touch sensor.
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.TOUCH)
BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.TOUCH)

try:
    while (True):
        #Run forward forever
        BP.set_motor_dps(BP.PORT_A, speed_dps)
        BP.set_motor_dps(BP.PORT_B, speed_dps)

        try:
            left_hit = BP.get_sensor(BP.PORT_1)
            right_hit = BP.get_sensor(BP.PORT_2)
            print ("Right: %s, Left: %s" % (right_hit,left_hit))
            
        except brickpi3.SensorError as error:
            print(error)

        if left_hit or right_hit:
            go_forward(-5, 50)
            if left_hit and right_hit:
                if bool(random.getrandbits(1)):
                   rotate(-45, 50)
                else:
                    rotate(45, 50) 
            if left_hit:
                rotate(-45, 50)
            elif right_hit:
                rotate(45, 50)
                





except KeyboardInterrupt:
	stop_robot()
