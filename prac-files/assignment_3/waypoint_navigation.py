#Generating particle distribution

import brickpi3
import time
import random
import math
BP = brickpi3.BrickPi3()


NUMBER_OF_PARTICLES = 100
general_weight = 1/NUMBER_OF_PARTICLES
DISTANCE = 100

def get_encode_length(distance):
    return (distance / 39.1) * 819.5

def get_rotation_amount (degree):
    return (degree / 2.42) * 2 * math.pi

class state:
    def __init__(self):
        self.line = [400, 500, 400,500]
        PARTICLE_SET = []

        for i in range(NUMBER_OF_PARTICLES):
            particle = [400, 500, 0, general_weight]
            PARTICLE_SET.append(particle)

        self.PARTICLE_SET = PARTICLE_SET
    
    def get_line(self):
        return line
    
    def set_line(self, line):
        self.line = line
    
    def get_particle_set(self):
        return self.PARTICLE_SET
    
    def set_particle_set(self, particle_set):
        self.PARTICLE_SET = particle_set
        
    def update_particle_set_line(self, opposite_direction, mean_distance, stand_dev_distance, mean_angle, stand_dev_angle, distance):
        i = 0
        if opposite_direction:
            distance = -distance
        for particle in self.PARTICLE_SET:
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
        print ("drawParticles:" + str(self.PARTICLE_SET))

    def update_line(self, update_x, positive, distance):
        if update_x and positive:
            #moving in the x direction
            self.line = (self.line[2], self.line[1], self.line[2] + distance, self.line[3])
        if not update_x and positive:
            self.line = (self.line[2], self.line[3], self.line[2], self.line[3] - distance)
        if update_x and not positive:
            self.line = (self.line[2], self.line[3], self.line[2] - distance, self.line[3])
        if not update_x and not positive:
            self.line = (self.line[2], self.line[3], self.line[2], self.line[3] + distance)

class robot:
    def __init__(self, positive_motion, angle_motion, state):
        self.estimate_position = [0, 0]
        self.positive_motion = positive_motion
        self.estimate_angle_motion = angle_motion
        self.state = state
    
    def get_positive_motion(self):
        return self.positive_motion
    
    def set_positive_motion(self, positive_motion):
        self.positive_motion = positive_motion
        
    def get_estimate_angle_motion(self):
        return self.estimate_angle_motion
    
    def set_estimate_angle_motion(self):
        estimate_angle_motion = 0
        for particle in self.state.get_particle_set():
            estimate_angle_motion = estimate_angle_motion + particle[2]
        self.estimate_angle_motion = estimate_angle_motion / NUMBER_OF_PARTICLES
        print("estimate angle motion: ", self.estimate_angle_motion)
        
    def get_estimate_position(self):
        return self.estimate_position
    
    def set_estimate_position(self):
        estimate_position = [0, 0]
        for particle in self.state.get_particle_set():
            estimate_position = (estimate_position[0] + particle[0], estimate_position[1] + particle[1])
        self.estimate_position = (estimate_position[0] / NUMBER_OF_PARTICLES, estimate_position[1] / NUMBER_OF_PARTICLES)
        print("estimate position: ", self.estimate_position)
        
    def print_robot_stats(self, port):
        print("Port: %s Flag: %s Power: %s Position: %s Velocity: %s" % (port, BP.get_motor_status(port) [0], BP.get_motor_status(port) [1], BP.get_motor_status(port) [2], \
        BP.get_motor_status(port) [3]))
    
    def stop_robot(self):
        BP.reset_all()
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))

    def go_forward(self, distance, speed_dps):
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
        self.set_estimate_angle_motion()
        self.set_estimate_position()

    def rotate(self, rad_amount, speed_dps):
        #Negate the speed
        speed_dps = -speed_dps

        #Rotate robot left by 90 degrees
        actual_degree = 0
        BP.set_motor_dps(BP.PORT_A, -speed_dps)
        BP.set_motor_dps(BP.PORT_B, speed_dps)
        target_rotation = get_rotation_amount(rad_amount)
        #print("target rotation %s" % target_rotation)
        while actual_degree < target_rotation:
            #Actual degree negate
            actual_degree = -BP.get_motor_encoder(BP.PORT_B)
            #print_robot_stats(BP.PORT_A)
            #print_robot_stats(BP.PORT_B)
            time.sleep(0.02)
            #print("Rotation complete")
        self.state.update_particle_set_angle(0, 0.15, rad_amount)
        self.set_estimate_angle_motion()
###################################################
#STARTING SCRIPT
##################################################

try:
    s = state()
    r = robot(1, 0, s)
    for i in range (4):
        #Update booleans
        positive_motion = i < 2
        x_direction = not(i % 2)
        r.set_positive_motion(positive_motion)
        opposite_direction = i ==1 or i == 3
        for j in range(4):
            r.go_forward(DISTANCE/4, 180)
            r.stop_robot()
            #New values according on position and particle set
            s.update_line(x_direction, positive_motion, DISTANCE)
            s.update_particle_set_line(opposite_direction, 0, 1, 0, 0.002, DISTANCE)
            #s.print_set()

            #Waiting time
            time.sleep(2)
        r.rotate(math.pi/2, 90)
    r.stop_robot()
    

except KeyboardInterrupt:
    r.stop_robot()
