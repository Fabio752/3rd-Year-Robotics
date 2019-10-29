import brickpi3
import time
import random
BP = brickpi3.BrickPi3()

speed_dps = -90

def get_encode_length(distance):
	return (distance/39.1)*819.5

def get_rotation_amount (degree):
	return (degree/138.7) * 360

def stop_robot():
        BP.reset_all()
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))

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
		#print_robot_stats(BP.PORT_A)
		#print_robot_stats(BP.PORT_B)
                time.sleep(0.02)
        print("Motion complete")


def rotate(degree_amount, speed_dps):
	#Rotate robot left by 90 degrees
	actual_degree = 0
        target_rotation = get_rotation_amount(abs(degree_amount))
        turn_right = True if degree_amount < 0 else False
        if turn_right:
            speed_dps = -speed_dps
	BP.set_motor_dps(BP.PORT_A, -speed_dps)
	BP.set_motor_dps(BP.PORT_B, speed_dps)
	print("target rotation %s" % target_rotation)
	while actual_degree < target_rotation:
		actual_degree = BP.get_motor_encoder(BP.PORT_A) if turn_right else BP.get_motor_encoder(BP.PORT_B)
        	#print_robot_stats(BP.PORT_A)
		#print_robot_stats(BP.PORT_B)
		time.sleep(0.02)
	print("Rotation complete")

# BP.set_sensor_type configures the BrickPi3 for a specific sensor.
# BP.PORT_1 specifies that the sensor will be on sensor port 1.
# BP.SENSOR_TYPE.TOUCH specifies that the sensor will be a touch sensor.
BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.TOUCH)
BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.TOUCH)

try:
    while (True):
        #Run forward forever
        BP.set_motor_dps(BP.PORT_A, speed_dps)
        BP.set_motor_dps(BP.PORT_B, speed_dps)
        
        left_hit = 0
        right_hit = 0
        try:
            left_hit = BP.get_sensor(BP.PORT_3)
            right_hit = BP.get_sensor(BP.PORT_4)
            print ("Right: %s, Left: %s" % (right_hit,left_hit))
            
        except brickpi3.SensorError as error:
            print(error)

        if left_hit or right_hit:
            go_forward(5, 50)
            if left_hit and right_hit:
                if bool(random.getrandbits(1)):
                   rotate(-360, 90)
                else:
                    rotate(360, 90) 
            if left_hit:
                rotate(-360, 90)
            elif right_hit:
                rotate(360, 90)
         
                





except KeyboardInterrupt:
	stop_robot()
