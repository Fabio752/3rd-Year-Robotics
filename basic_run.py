# Write your code here :-)
import brickpi3
BP = brickpi3.BrickPi3()

def get_encode_length(distance):
    return (distance / 22)*450

try:
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    target_degree_rotation = get_encode_length(40)
    BP.set_motor_dps(BP.PORT_A, 90)
    BP.set_motor_dps(BP.PORT_B, 90)
    actual_degree = 0
    while actual_degree < target_degree_rotation:
    	actual_degree = (BP.get_motor_encoder(BP.PORT_A) + BP.get_motor_encoder(BP.PORT_B)) / 2
	print("A: %s, B: %s" % (BP.get_motor_encoder(BP.PORT_A), BP.get_motor_encoder(BP.PORT_B)))

except KeyboardInterrupt:
    BP.reset_all()
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
