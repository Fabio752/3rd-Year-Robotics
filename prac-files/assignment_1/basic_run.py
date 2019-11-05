# Write your code here :-)
import brickpi3
BP = brickpi3.BrickPi3()

def get_encode_length(distance):
    return (distance/39.5)*819.5

def stop_robot():
    BP.reset_all() 
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))


try:
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    target_degree_rotation = get_encode_length(40)
    BP.set_motor_dps(BP.PORT_A, 90)
    BP.set_motor_dps(BP.PORT_B, 90)
    actual_degree = 0
    print("target rotation: %s" % target_degree_rotation)
    while actual_degree < target_degree_rotation:
    	actual_degree = (BP.get_motor_encoder(BP.PORT_A) + BP.get_motor_encoder(BP.PORT_B)) / 2
	print("A: %s, B: %s" % (BP.get_motor_encoder(BP.PORT_A), BP.get_motor_encoder(BP.PORT_B)))

    print("reached corner")
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    print("reset encoders")
    #Rotate robot left by 90 degrees
    actual_degree = 0
    BP.set_motor_dps(BP.PORT_A, -90)
    BP.set_motor_dps(BP.PORT_B, 90)
    while actual_degree < 360:
	actual_degree = BP.get_motor_encoder(BP.PORT_B)
   
    stop_robot()

except KeyboardInterrupt:
    stop_robot()
