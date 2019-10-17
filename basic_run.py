# Write your code here :-)
import brickpi3
BP = brickpi3.BrickPi3()
try:
    while True:
    	BP.set_motor_power(BP.PORT_A, 50)
    	BP.set_motor_power(BP.PORT_B, 50)

except KeyboardInterrupt:
    BP.reset_all()
