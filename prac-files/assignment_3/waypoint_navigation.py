#Generating particle distribution

import brickpi3
import time
import random
import math
BP = brickpi3.BrickPi3()


NUMBER_OF_PARTICLES = 100
general_weight = 1

class state:
    def __init__(self):
        self.line = [0,0,0,0]
        PARTICLE_SET = []

        for i in range(NUMBER_OF_PARTICLES):
            particle = [0,0,0, general_weight]
            PARTICLE_SET.append(particle)

        self.PARTICLE_SET = PARTICLE_SET

    def update_particle_set_line(self, mean_distance, stand_dev_distance, mean_angle, stand_dev_angle, distance):
        i = 0
        for particle in self.PARTICLE_SET:
            print ("x_new = ")
            print(particle[0])
            print(" + ")
            print( distance)
            print("random.gauss(mean_distance, stand_dev_distance)")
            print( " * math.cos(")
            print(particle[2])
            print( ")\n") 
            x_new = particle[0] + (distance + random.gauss(mean_distance, stand_dev_distance)) * math.cos(particle[2])
            y_new = particle[1] + (distance + random.gauss(mean_distance, stand_dev_distance)) * math.sin(particle[2])
            theta_new = particle[2] + random.gauss(mean_angle, stand_dev_angle)
            self.PARTICLE_SET[i] = (x_new, y_new, theta_new, particle[3])
            i = i + 1

    def update_particle_set_angle(self, mean, stand_dev, alpha):
        i = 0
        for particle in self.PARTICLE_SET:
            theta_new = particle[2] + alpha + random.gauss(mean, stand_dev)
            self.PARTICLE_SET[i] = (particle[0], particle[1], theta_new, particle[3])
            i = i + 1

    def print_set(self):
        print ("drawLine:" + str(self.line))
        for particle in self.PARTICLE_SET:
            print ("drawParticles:" + str(particle))

    def update_line(self, update_x, positive, distance):
        if update_x and positive:
            #moving in the x direction
            self.line = (self.line[2], self.line[1], self.line[2] + distance, self.line[3])
        if not update_x and positive:
            self.line = (self.line[2], self.line[3], self.line[2], self.line[3] + distance)
        if update_x and not positive:
            self.line = (self.line[2], self.line[3], self.line[2] - distance, self.line[3])
        if not update_x and not positive:
            self.line = (self.line[2], self.line[3], self.line[2], self.line[3] - distance)

def get_encode_length(distance):
    return (distance / 39.1) * 819.5

def get_rotation_amount (degree):
    return (degree / 138.7) * 360

def print_robot_stats(port):
    print("Port: %s Flag: %s Power: %s Position: %s Velocity: %s" % (port, BP.get_motor_status(port) [0], BP.get_motor_status(port) [1], BP.get_motor_status(port) [2], \
    BP.get_motor_status(port) [3]))

def stop_robot():
    BP.reset_all()
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))

def go_forward(distance, speed_dps):
    #Negating speed
    speed_dps = -speed_dps
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    #Initializations
    target_degree_rotation = get_encode_length(distance)
    actual_degree_rotation = 0
    BP.set_motor_dps(BP.PORT_A, speed_dps)
    BP.set_motor_dps(BP.PORT_B, speed_dps)

    #print("target degree rotation %s" % target_degree_rotation)
    while actual_degree_rotation < target_degree_rotation:
        actual_degree_rotation = -(BP.get_motor_encoder(BP.PORT_A) + BP.get_motor_encoder(BP.PORT_B))/2
        #print("actual degree rotation: %s" % actual_degree_rotation)
        #print_robot_stats(BP.PORT_A)
        #print_robot_stats(BP.PORT_B)
        time.sleep(0.02)
        #print("Motion complete")

def rotate(degree_amount, speed_dps):
    #Negate the speed
    speed_dps = -speed_dps

    #Rotate robot left by 90 degrees
    actual_degree = 0
    BP.set_motor_dps(BP.PORT_A, -speed_dps)
    BP.set_motor_dps(BP.PORT_B, speed_dps)
    target_rotation = get_rotation_amount(degree_amount)
    #print("target rotation %s" % target_rotation)
    while actual_degree < target_rotation:
        #Actual degree negate
        actual_degree = -BP.get_motor_encoder(BP.PORT_B)
        #print_robot_stats(BP.PORT_A)
        #print_robot_stats(BP.PORT_B)
        time.sleep(0.02)
        #print("Rotation complete")


def move():
    s = state()
    for i in range (4):
        #Update booleans
        positive = (i < 2)
        update_x = not(i % 2)

        for j in range(4):
            go_forward(10, 180)
            stop_robot()
            #New values according on position and particle set
            s.update_line(update_x, positive, 100)
            s.update_particle_set_line(0, 0.01, 0, 0.01, 100)
            s.print_set()

            #Waiting time
            time.sleep(2)
        rotate(90, 90)
        s.update_particle_set_angle(0,0.01, 90)
    stop_robot()

###################################################
#STARTING SCRIPT
##################################################

try:
    move()

except KeyboardInterrupt:
    stop_robot()
