# Write your code here :-)
import brickpi3
import time
BP = brickpi3.BrickPi3()

def get_encode_length(distance):
	return (distance/39.5)*819.5

def get_rotation_amount (degree):
	return (degree/135.1) * 360

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
                print("actual degree rotation: %s" % actual_degree_rotation)
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
        	print("actual degree: %s" % actual_degree)
		time.sleep(0.02)
	print("Rotation complete")

try:
	for _ in range(4):
		#Move the robot forwards
		go_forward(40, 180)
		#Reset encoders
         	BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
		BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
		#Set degree rotation
		target_degree_rotation = get_encode_length(40)
		#Perform rotation
	        rotate(90, 90)
	stop_robot()

except KeyboardInterrupt:
	stop_robot()
