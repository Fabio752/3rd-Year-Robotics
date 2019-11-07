
# Imports.
import brickpi3
import time
import random
import math
import particleDataStructures as world_map #Contains lib for the preset environment and displaying of particles in map

BP = brickpi3.BrickPi3()
NUMBER_OF_PARTICLES = 100
general_weight = 1/NUMBER_OF_PARTICLES
standard_dev = 1

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
    

    # Update particle weights (using likelihood).
    def calculate_likelihood(x, y, theta, z):
        #z = sonar measurement

        #Find out which wall the sonar should be pointing to, and the expected distance m
        
        # Line of sight will interesect with wall
        # Find equation of line of sight
        # y = m(x-x_0) + y_0 where m is theta, (x_0, y_0) is the current pos of robot
        # los_y = lambda in_x: theta(in_x - x) + y
        # los_x = lambda in_y: (in_y - y)/m + x if m != 0 else x
        # for wall in world_map.Map.walls:
        #     #If the lines intersect, then we can equate them
        #     spec_x = 0
        #     grad = (wall[3] - wall[1])/(wall[2] = wall[0])
        #     wall_y = grad*(spec_x - wall[0]) + wall[1]
        
        #Once wall is found, compute m
        chosen_wall = (84,210,168,210) #Dummy, need to find wall first
        # Ax = chosen_wall[0], Ay = chosen_wall[1], Bx = chosen_wall[2], By = chosen_wall[3]
        m = ((chosen_wall[3] - chosen_wall[1])*(chosen_wall[0] - x) - (chosen_wall[2] - chosen_wall[0])*(chosen_wall[1] - y)) /\
            ((chosen_wall[3] - chosen_wall[1])*math.cos(theta) - (chosen_wall[2] - chosen_wall[0])*math.sin(theta))
        #Return the new particle weight (acording to likelihood function)
        return math.exp((-math.pow((z-m),2) / 2*math.pow(standard_dev,2)))

      # Graphics.
    def print_set(self):
        # Initialise scaling values.
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
        
        while actual_degree_rotation < target_degree_rotation:
            actual_degree_rotation = -(BP.get_motor_encoder(BP.PORT_A) + BP.get_motor_encoder(BP.PORT_B)) / 2
            time.sleep(0.02)        

        # Update particle set.
        self.state.update_line(final_x, final_y)
        self.state.update_particle_set_line(0, 1, 0, 0.01, distance)
        
        # Update robot position and orientation estimates.
        self.set_estimate_location()
        self.stop_robot()

    # Rotation on the spot.    
    def rotate(self, rad_amount, speed_dps):
        # Negate the speed if has to turn left, keep it for turning right.
        if(rad_amount > 0):
            speed_dps = -speed_dps

        # Set opposite wheels speed.
        actual_degree = 0
        BP.set_motor_dps(BP.PORT_A, -speed_dps)
        BP.set_motor_dps(BP.PORT_B, speed_dps)
        target_rotation = get_rotation_amount(abs(rad_amount))
        
        # Rotation on the spot.
        while actual_degree < target_rotation:
            actual_degree = abs(BP.get_motor_encoder(BP.PORT_B))
            time.sleep(0.02)
        
        # Update the particle set and location estimates.
        self.state.update_particle_set_angle(0, 0.015, rad_amount)
        self.set_estimate_location()
        
    # Move to a specific coordinate by : 
    #    1 - rotating.
    #    2 - go straight.
    def navigate_to_waypoint(self, x, y):
        # Compute coordinate differences and scale meters->centimeters.
        x = x * 100
        y = y * 100
        x_diff = x - self.estimate_location[0]
        y_diff = y - self.estimate_location[1]
        rotation_amount = 0;
        
        # Standard case, just compute the angle.
        if not (x_diff < 0.5 and x_diff > -0.5) and not (y_diff < 0.5 and y_diff > -0.5) :
            rotation_amount = math.atan(y_diff/x_diff) - self.estimate_location[2]
            if (x < 0): # atan returns the wrong value if the angle is in the 2nd/4th quadrant.
                rotation_amount = rotation_amount + math.pi

        # If the point lies roughly on the same line we have special cases.
        # Case on the same vertical.
        elif (x_diff < 0.5 and x_diff > -0.5) :
            if (y_diff >=  0): 
                rotation_amount = math.pi / 2 - self.estimate_location[2]
            else :
                rotation_amount = 3 * math.pi / 2 - self.estimate_location[2]
        
        # Case on the same horizontal.
        elif (y_diff < 0.5 and y_diff > -0.5) :
            if (x_diff >= 0) :
                rotation_amount = - self.estimate_location[2]
            else:
                rotation_amount = math.pi - self.estimate_location[2]
        
        # Step1: rotate.
        self.rotate(rotation_amount, 90)       
        
        # Compute distance to travel.
        distance_amount = math.sqrt(pow(x_diff, 2) + pow(y_diff, 2))
                        
        # Step2: go forward.
        # print("distance amount %s \n" % distance_amount);
        self.go_forward(distance_amount, 180, x, y)
        
        # Print particles and line status.
        self.state.print_set()
    
    # Debugging Function.
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
    # Keep inputting coordinates, the robot will go there.
    r = robot()
    while(1):    
    	x = input("Specify coordinate x\n")
    	y = input("Specify coordinate y\n")
    	r.navigate_to_waypoint(x, y)
    	r.stop_robot()

# Keyboard Interrupt.    
except KeyboardInterrupt:
    r.stop_robot()
