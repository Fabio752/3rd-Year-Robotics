
# Imports.
import brickpi3
import time
import random
import math

BP = brickpi3.BrickPi3()
NUMBER_OF_PARTICLES = 100
general_weight = 1/NUMBER_OF_PARTICLES

# General purpose functions.
def get_encode_length(distance):
    return (distance / 39.1) * 819.5

def get_rotation_amount (rad):
    degree_amount = rad * 57.296
    return (degree_amount / 144.7) * 360

# A state has a particle set and a line.
class state:
    # Initialise member variables.
    # Note that 400,500 are the coordinates' initial offsets needed for displaying on the graphic interface. 
    def __init__(self):
        self.line = [0, 0, 0, 0]
        PARTICLE_SET = []

        for i in range(NUMBER_OF_PARTICLES):
            particle = [0, 0, 0, general_weight]
            PARTICLE_SET.append(particle)

        self.PARTICLE_SET = PARTICLE_SET
    
    # Getters and setters.
    def get_line(self):
        return line
    
    def set_line(self, line):
        self.line = line
    
    def get_particle_set(self):
        return self.PARTICLE_SET
    
    def set_particle_set(self, particle_set):
        self.PARTICLE_SET = particle_set
    
    # Updaters.
    # Straight motion updater.
    def update_particle_set_line(self, mean_distance, stand_dev_distance, mean_angle, stand_dev_angle, distance):
        i = 0
        for particle in self.PARTICLE_SET:
            x_new = particle[0] + (distance + random.gauss(mean_distance, stand_dev_distance)) * math.cos(particle[2])
            y_new = particle[1] + (distance + random.gauss(mean_distance, stand_dev_distance)) * math.sin(particle[2])
            theta_new = particle[2] + random.gauss(mean_angle, stand_dev_angle)
            # We are currently not changing the weights.
            self.PARTICLE_SET[i] = (x_new, y_new, theta_new, particle[3])
            i = i + 1
    
    # Rotation updater.
    def update_particle_set_angle(self, mean, stand_dev, alpha):
        i = 0
        for particle in self.PARTICLE_SET:
            theta_new = particle[2] + alpha + random.gauss(mean, stand_dev)
            # We are currently not changing the weights.
            self.PARTICLE_SET[i] = (particle[0], particle[1], theta_new, particle[3])
            i = i + 1
    
    # Line updater.
    def update_line(self, final_x, final_y):
        self.line = (self.line[2], self.line[3], final_x, final_y)
    
      # Graphics.
    def print_set(self):
        scale = 10
        res_line = [res * scale for res in self.line]
        res_line[0] += 400
        res_line[1] += 500
        res_line[2] += 400
        res_line[3] += 500

        res_particle_list = []
        for particle in self.PARTICLE_SET:
            # print("Before" + str(particle))
            res_particle = [None, None, None]
            res_particle[0] = particle[0] * scale + 400
            res_particle[1] = particle[1] * scale + 500
            res_particle[2] = particle[2]
            # print("After" + str(tuple(res_particle)))
            res_particle_list.append(tuple(res_particle))
        print (res_line[0])
        print(res_line[1])
        print (res_line[2])
        print(res_line[3])
        print ("drawLine:" + str(tuple(res_line)))
        print ("drawParticles:" + str(res_particle_list))
        
# A robot has a state and an estimated location.
class robot:
    # Initialise member variables.
    def __init__(self):
        self.state = state()
        self.set_estimate_location()

    # Getters and setters.    
    def get_estimate_location(self):
        return self.estimate_location
    
    def set_estimate_location(self):
        estimate_location = [0, 0, 0]
        for particle in self.state.get_particle_set():
            estimate_location = (estimate_location[0] + particle[0], estimate_location[1] + particle[1], estimate_location[2] + particle[2])
        self.estimate_location = (estimate_location[0] / NUMBER_OF_PARTICLES, estimate_location[1] / NUMBER_OF_PARTICLES, estimate_location[2] / NUMBER_OF_PARTICLES)
    
    # Motion functions.
    # Straight motion.
    def go_forward(self, distance, speed_dps, final_x, final_y):
        # Negating speed.
        speed_dps = -speed_dps
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
        # Initializations.
        target_degree_rotation = get_encode_length(distance)
        actual_degree_rotation = 0
        BP.set_motor_dps(BP.PORT_A, speed_dps)
        BP.set_motor_dps(BP.PORT_B, speed_dps)

        # print("target degree rotation %s" % target_degree_rotation)
        
        while actual_degree_rotation < target_degree_rotation:
            actual_degree_rotation = -(BP.get_motor_encoder(BP.PORT_A) + BP.get_motor_encoder(BP.PORT_B))/2
            # print("actual degree rotation: %s" % actual_degree_rotation)
            # self.print_robot_stats(BP.PORT_A)
            # self.print_robot_stats(BP.PORT_B)
            time.sleep(0.02)
            # print("Motion complete")
        
        # Update particle set.
        self.state.update_line(final_x, final_y)
        self.state.update_particle_set_line(0, 1, 0, 0.01, distance)
        
        # Update robot position and orientation estimates.
        self.set_estimate_location()
        self.stop_robot()

    # Rotation on the spot.    
    def rotate(self, rad_amount, speed_dps):
        # Negate the speed.
        if(rad_amount >0):
            speed_dps = -speed_dps

        # Rotate robot left by 90 degrees.
        actual_degree = 0
        BP.set_motor_dps(BP.PORT_A, -speed_dps)
        BP.set_motor_dps(BP.PORT_B, speed_dps)
        target_rotation = get_rotation_amount(abs(rad_amount))
        # print("target rotation %s" % target_rotation)
        print("target_rotation %s\n" %target_rotation)
        while actual_degree < target_rotation:
            # Actual degree negate
            actual_degree = abs(BP.get_motor_encoder(BP.PORT_B))
            # print_robot_stats(BP.PORT_A)
            # print_robot_stats(BP.PORT_B)
            time.sleep(0.02)
            # print("Rotation complete")
        
        # Update the particle set and location estimates.
        self.state.update_particle_set_angle(0, 0.015, rad_amount)
        self.set_estimate_location()
        
    # Move to a specific coordinate by : 
    #    1 - rotating.
    #    2 - go straight.
    def navigate_to_waypoint(self, x, y):
        # Compute coordinate differences.
        x = x * 100
        y = y * 100
        x_diff = x - self.estimate_location[0]
        y_diff = y - self.estimate_location[1]
        rotation_amount = 0;
        
        # We need to find theta either way, so if x_diff is zero, we need to rotate by 
        if not (x_diff < 0.5 and x_diff > -0.5) and not (y_diff < 0.5 and y_diff > -0.5) :
            rotation_amount = math.atan(y_diff/x_diff) - self.estimate_location[2]
            if (x < 0):
                rotation_amount = rotation_amount + math.pi
        #If the point lies in the same line, we can rotate by (-theta) in order to point in the right position
        elif (x_diff < 0.5 and x_diff > -0.5) :
            if (y_diff >=  0): 
                # 90 - theta
                print(self.estimate_location[2])
                rotation_amount = math.pi / 2 - self.estimate_location[2]
            else:
                # 270 - theta
                rotation_amount = 3 * math.pi / 2 - self.estimate_location[2]
        elif (y_diff < 0.5 and y_diff > -0.5) :
            if (x_diff >= 0) :
                rotation_amount = - self.estimate_location[2]
            else:
                rotation_amount = math.pi - self.estimate_location[2]
        
       # if(rotation_amount >= math.pi * 2):
        #    rotation_amount = rotation_amount - math.pi * 2
            
        # Step1: rotation.
        # print("rotation %s\n" %rotation_amount)
        self.rotate(rotation_amount, 90)       
        
        # Compute distance to travel.
        distance_amount = math.sqrt(pow(x_diff, 2) + pow(y_diff, 2))
        
        print("distance amount: %s", distance_amount)
                
        # Step2: go forward.
        # print("distance amount %s \n" % distance_amount);
        self.go_forward(distance_amount, 180, x, y)
        
        # Print particles and line status.
        self.state.print_set()
    
    # Debugging.
    def print_robot_stats(self, port):
        print("Port: %s Flag: %s Power: %s Position: %s Velocity: %s" % (port, BP.get_motor_status(port) [0], BP.get_motor_status(port) [1], BP.get_motor_status(port) [2], \
        BP.get_motor_status(port) [3]))
    
    # Stop the robot.
    def stop_robot(self):
        BP.reset_all()
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    

##################################################
#STARTING SCRIPT
##################################################

try:
    r = robot()
    while(1):    
    	x = input("Specify coordinate x\n")
    	y = input("Specify coordinate y\n")
    	r.navigate_to_waypoint(x, y)
    	r.stop_robot()
# Keyboard Interrupt.    
except KeyboardInterrupt:
    r.stop_robot()
