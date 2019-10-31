#Generating particle distribution

import brickpi3
import time
BP = brickpi3.BrickPi3()


NUMBER_OF_PARTICLES = 100
general_weight = 1/NUMBER_OF_PARTICLES
PARTICLE_SET = []

def get_encode_length(distance):
	return (distance/39.1)*819.5

def get_rotation_amount (degree):
	return (degree/138.7) * 360

def print_robot_stats(port):
	print("Port: %s Flag: %s Power: %s Position: %s Velocity: %s" % (port, BP.get_motor_status(port) [0], BP.get_motor_status(port) [1], BP.get_motor_status(port) [2], \
	BP.get_motor_status(port) [3]))

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


def update_particle_set(mean, stand_dev):



###################################################
#STARTING SCRIPT
##################################################


#Populating initial array of tuples
for _ in range(100):
    #Set all initial values to zero
    PARTICLE_SET.append((0,0,0))

go_forward(40,90)




