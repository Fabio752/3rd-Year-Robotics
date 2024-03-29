
# Imports.
import brickpi3
import time
import random
import math
import copy

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)
BP.reset_all()

NUMBER_OF_PARTICLES = 100.0
general_weight = 1.0/NUMBER_OF_PARTICLES
distance_var_ratio = 0.10 # previously set to 1 (higher in order to show MCL working
standard_dev_angle = 0.015 # previously set to 0.015
likelihood_standard_dev = 3
CARPET = "doc"
USE_MCL = True
INITIAL_POSITION = [84,30, 0] # assessment done on a robot starting not from the origin
debug = True

# General purpose functions.
def sensor_instantiation():
    #Initialise sonar sensor
    #BP.reset_all()
    BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.NXT_ULTRASONIC)

def get_encode_length(distance):
    return (distance / 39.4) * 819.5

def get_rotation_amount (rad):
    degree_amount = rad * 57.296
    return (degree_amount / 141.5) * 360 if CARPET == "doc" else (degree_amount / 110.0) * 360

# Canvas of the environment
class Canvas:
    def __init__(self, map_size = 210):
        self.map_size    = map_size     # in cm
        self.canvas_size = 768          # in pixels
        self.margin      = 0.05 * map_size
        self.scale       = self.canvas_size / (map_size + 2 * self.margin)

    def drawLine(self, line):
        x1 = self.__screenX(line[0])
        y1 = self.__screenY(line[1])
        x2 = self.__screenX(line[2])
        y2 = self.__screenY(line[3])
        print ("drawLine:" + str((x1, y1, x2, y2)))

    def drawParticles(self,data):
        display = [(self.__screenX(d[0]), self.__screenY(d[1])) + d[2:] for d in data]
        print ("drawParticles:" + str(display))

    def __screenX(self, x):
        return (x + self.margin) * self.scale

    def __screenY(self, y):
        return (self.map_size + self.margin - y) * self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = []
        self.canvas = Canvas(50)
    def add_wall(self, wall):
        self.walls.append(wall)

    def clear(self):
        self.walls = []

    def draw(self):
        for wall in self.walls:
            self.canvas.drawLine(wall)

# A state has a particle set and a line.
class state:
    # Initialise member variables.
    # Note that 400,500 are the coordinates' initial offsets needed for displaying on the graphic interface.
    def __init__(self):
        self.line = [INITIAL_POSITION[0], INITIAL_POSITION[1], INITIAL_POSITION[0], INITIAL_POSITION[1]]
        PARTICLE_SET = []

        for i in range(int(NUMBER_OF_PARTICLES)):
            particle = [INITIAL_POSITION[0], INITIAL_POSITION[1], INITIAL_POSITION[2], general_weight]
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
            # Weights updated seperately
            self.PARTICLE_SET[i] = (x_new, y_new, theta_new, particle[3])
            i = i + 1

    # Rotation updater.
    def update_particle_set_angle(self, mean, stand_dev, alpha):
        i = 0
        for particle in self.PARTICLE_SET:
            theta_new = particle[2] + alpha + random.gauss(mean, stand_dev)
            # Weights are updated seperately
            self.PARTICLE_SET[i] = (particle[0], particle[1], theta_new, particle[3])
            i = i + 1

    # Weight updater.
    def update_particle_set_weights(self, robo_map, sonar_measurement):
        if debug:
            print("Sonar measurement(reading + offset) ", sonar_measurement)
        total_weight_sum = 0
        for i in range(len(self.PARTICLE_SET)):
            particle = self.PARTICLE_SET[i]
            weight_scale = self.calculate_likelihood(robo_map, particle[0], particle[1], particle[2], sonar_measurement)
            new_weight = particle[3] * weight_scale
            if debug:
                pass
               # print("particle %s new weight %s, weight scaling %s"% (particle, new_weight, weight_scale))
            total_weight_sum += new_weight
            particle = (particle[0], particle[1], particle[2], new_weight)
            self.PARTICLE_SET[i] = particle

        # Normalise
        for i in range(len(self.PARTICLE_SET)):
            particle = self.PARTICLE_SET[i]
            particle = (particle[0], particle[1], particle[2], particle[3] / total_weight_sum)
            self.PARTICLE_SET[i] = particle
            if debug:
                pass
                #print(i, ": new rescaled weight = ", particle[3])

    # Resampling of particles
    def resample_particle_set(self):
        new_particle_set = []
        # Create cumulative weight array
        cumulative_weight = 0
        cumulative_weight_arr = []
        for particle in self.PARTICLE_SET:
            cumulative_weight += particle[3]
            cumulative_weight_arr.append(cumulative_weight)

        #Use random num gen to pick particle
        for i in range(len(self.PARTICLE_SET)):
            rand_num = random.uniform(0, 1)
            # print("random_num", rand_num)
            particle_idx = None
            for j in range(0, len(cumulative_weight_arr)):
                if rand_num < cumulative_weight_arr[j]:
                    # Value is in range of previous particle
                    particle_idx = j
                    break
            # Reset weight
            if debug:
                pass
                #print("Old particle chosen: ", particle_idx)
            old_particle = self.PARTICLE_SET[particle_idx]
            new_particle = (old_particle[0], old_particle[1], old_particle[2], 1/NUMBER_OF_PARTICLES)
            new_particle_set.append(new_particle)

        self.PARTICLE_SET = new_particle_set

    # Line updater.
    def update_line(self, final_x, final_y):
        self.line = (self.line[2], self.line[3], final_x, final_y)

    # Update particle weights (using likelihood).
    def calculate_likelihood(self, robo_map, x, y, theta, z):
        # z = sonar measurement

        # Find out which wall the sonar should be pointing to, and the expected distance m
        min_distance = 255
        chosen_wall = None #tuple of chosen wall + m value
        for wall in robo_map.walls:
            # Compute m
            # Ax = chosen_wall[0], Ay = chosen_wall[1], Bx = chosen_wall[2], By = chosen_wall[3]
            m = ((wall[3] - wall[1]) * (wall[0] - x) - (wall[2] - wall[0]) * (wall[1] - y)) /\
                ((wall[3] - wall[1]) * math.cos(theta) - (wall[2] - wall[0]) * math.sin(theta))
            if debug:
                #print("Wall: ",wall,"m: ",m)
                pass
            # Compute intersect x and y to wall
            intersect_x = x + m * math.cos(theta)
            intersect_y = y + m * math.sin(theta)

            # Sub intersect x into wall equation to see if the correct y is found
            # grad = ((wall[3] - wall[1]) / (wall[2] - wall[0])) if wall[2] - wall[0]
            # wall_y = grad*(intersect_x - wall[0]) + wall[1]
            intersects = False
            if m > 0:
                if wall[2] - wall[0] == 0:
                    # Is vertical wall
                    intersects = (intersect_y < max(wall[1], wall[3]) and intersect_y > min (wall[1], wall[3]))
                else:
                    intersects = (intersect_x < max(wall[0],wall[2]) and intersect_x > min(wall[0], wall[2]))
            if intersects:
                # Choose closest wall
                if chosen_wall is None:
                    chosen_wall = (wall[0],wall[1],wall[2],wall[3],m)
                if m < chosen_wall[4]:
                    chosen_wall = (wall[0],wall[1],wall[2],wall[3],m)
                #debug print
                if debug:
                    #print("Chosen wall: ", chosen_wall)
                    pass

        if chosen_wall is not None:
            if debug:
                pass
                #print("Difference betweeen sensor and m: ", z - chosen_wall[4])
            # Compute the angle between the sonar direction and the normal to the wall
            sonar_normal_angle = math.acos( \
                (math.cos(theta)*(chosen_wall[1] - chosen_wall[3]) + math.sin(theta) * (chosen_wall[2] - chosen_wall[0]))/\
                (math.sqrt(math.pow(chosen_wall[1]-chosen_wall[3],2) + math.pow(chosen_wall[2] - chosen_wall[0],2))) )
            if debug:
                pass
                #print("sonar normal angle", sonar_normal_angle)
            # If the sonar angle is too great, ignore
            if sonar_normal_angle > 0.4: #Limit in radians
                return 1 #Don't modify the particle weight
            # Return the new particle weight (acording to likelihood function)
            value = math.exp(- math.pow((z - chosen_wall[4]), 2) / (2 * math.pow(likelihood_standard_dev, 2))) + 0.0001 #Offset to make robust likelihood
            if debug:
                pass
                #print("likelihood value", value)
            return value
        else:
            print ("Warning: no wall detected, weights will not be updated")
            return 1

# A robot has a state and an estimated location.
class robot:
    # Initialise member variables.
    def __init__(self):
        self.state = state()
        self.map = Map()
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
    def go_forward(self, distance, speed_dps, final_x, final_y, sonar_offset = 7):

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

        self.stop_robot()
        # Update particle set.
        self.state.update_line(final_x, final_y)
        var_distance = distance * distance_var_ratio
        standard_dev_distance  = 1
        self.state.update_particle_set_line(0, standard_dev_distance, 0, 0.01, distance)
        
        while USE_MCL:
            try:
                # Reset sensor type (hack to make it work)
                BP.reset_all()
                BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.NXT_ULTRASONIC)
                time.sleep(0.5)
                if debug:
                    print("Sensor value", BP.get_sensor(BP.PORT_2))
                self.state.update_particle_set_weights(self.map, BP.get_sensor(BP.PORT_2) + sonar_offset)
                self.state.resample_particle_set()
                break
            except IOError as e:
                print("Warning: IO Error, skipping MCL...")
                print(e)
            except brickpi3.SensorError as e:
                #print(e)
                pass
        # Update robot position and orientation estimates.
        self.set_estimate_location()
        self.stop_robot()

    # Rotation on the spot.
    def rotate(self, rad_amount, speed_dps, sonar_offset=7):
        # Negate the speed if has to turn left, keep it for turning right.
        if(rad_amount > 0):
            speed_dps = -speed_dps

        if debug:
            print("rad amount: ", rad_amount)
        # Set opposite wheels speed.
        actual_degree = 0
        BP.set_motor_dps(BP.PORT_A, -speed_dps)
        BP.set_motor_dps(BP.PORT_B, speed_dps)
        target_rotation = get_rotation_amount(abs(rad_amount))

        # Rotation on the spot.
        while actual_degree < target_rotation:
            actual_degree = abs(BP.get_motor_encoder(BP.PORT_B))
            time.sleep(0.02)

        self.stop_robot()
        # Update the particle set and location estimates.
        #We should not randomize unless we are not using MCL
        self.state.update_particle_set_angle(0, standard_dev_angle, rad_amount)
        while USE_MCL:
            try:
                # Reset sensor type (hack to make it work)
                BP.reset_all()
                BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.NXT_ULTRASONIC)
                if debug:
                    print("Sensor value: ", BP.get_sensor(BP.PORT_2))
                self.state.update_particle_set_weights(self.map, BP.get_sensor(BP.PORT_2) + sonar_offset)
                self.state.resample_particle_set()
                break
            except IOError:
                print("Warning: IO Error, skipping MCL...")
            except brickpi3.SensorError as e:
                pass
                #print(e)
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
        threshold = 5

        if debug:
            print("navigation to: (x,y)", x, y)
            print("xdiff: ", x_diff, "ydiff: ", y_diff, "self_estimate location (x,y,theta)", self.estimate_location)
        # Standard case, just compute the angle.
        #TODO: Chance the thresholds to the unfinessed version
        if not (x_diff < threshold and x_diff > -threshold) and not (y_diff < threshold and y_diff > -threshold) :
            rotation_amount = math.atan2(y_diff,x_diff) #- self.estimate_location[2]
            if debug:
                print ("took x & y big diff and turned: ", rotation_amount)
            if (x_diff < 0): # atan returns the wrong value if the angle is in the 2nd/4th quadrant.
                rotation_amount = rotation_amount + math.pi

        # If the point lies roughly on the same line we have special cases.
        # Case on the same vertical.
        elif (x_diff < threshold and x_diff > -threshold) :
            if (y_diff >=  0):
                rotation_amount = math.pi / 2 - self.estimate_location[2]
                if debug:
                    print("took x diff small and y diff positive and turned: ", rotation_amount)
            else :
                rotation_amount = 3 * math.pi / 2 - self.estimate_location[2]
                if debug:
                    print("took x diff small and y diff neg and turned: ", rotation_amount)

        # Case on the same horizontal.
        elif (y_diff < threshold and y_diff > -threshold) :
            if (x_diff >= 0) :
                rotation_amount = - self.estimate_location[2]
                if debug:
                    print("took y diff small and x diff pos and turned: ", rotation_amount)
            else:
                rotation_amount = math.pi - self.estimate_location[2]
                if debug:
                    print("took y diff small and x diff neg and turned: ", rotation_amount)
        
        if abs(rotation_amount) < 0.001:
            rotation_amount = 0
            print("ignored rotation")

        
        # Step1: rotate by the amount we are off by.
        self.rotate(rotation_amount, 90)

        # Compute distance to travel.
        distance_amount = math.sqrt(pow(x_diff, 2) + pow(y_diff, 2))

        # Step2: go forward.
        self.go_forward(distance_amount, 180, x, y)
        time.sleep(0.5)
        #recalculate particle distribution
        self.map.canvas.drawParticles(self.state.PARTICLE_SET)
    
    
    def rotate_sonar(self, total_angle, intervals):
        rad_amount = abs(total_angle)
        rad_interval = rad_amount / intervals
        speed_dps = -45 if total_angle > 0 else 45
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
        print("encoder value: ", BP.get_motor_encoder(BP.PORT_B))
        # Rotate robot left by 90 degrees.
        for interval_count in range(intervals):
            #Read sensor value
            BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)
            time.sleep(1)
            try:
                dis = BP.get_sensor(BP.PORT_1)
                print("successful reading: ", dis)
            except brickpi3.SensorError:
                dis = 255
            #Use sensor reading to print line
            #Origin at 500, 500
            origin = 15
            line_scale = 0.2
            #Fetch rotation angle

            #Print lines
            current_angle = interval_count *rad_interval
            print("current angle ", math.degrees(current_angle))
            new_line = (origin,origin, origin + line_scale* dis * math.cos(current_angle), origin + line_scale * dis * math.sin(current_angle))
            print(new_line)
            self.map.canvas.drawLine(new_line)


            #Move sonar
            actual_degree = 0
            BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
            rad_amount = rad_amount - rad_interval
            #BP.set_motor_dps(BP.PORT_A, -speed_dps)
            #BP.set_motor_dps(BP.PORT_B, speed_dps)
            #BP.set_motor_position(BP.PORT_B, 30)
            print("rad amount %s inteval %s" % (rad_amount, rad_interval))
            #target_rotation = get_rotation_amount(rad_interval)
            target_rotation = math.degrees(rad_interval) * 0.9 #Arbitrary scaling to get to same spot
            print("target rotation %s" % target_rotation)
            while actual_degree < target_rotation:
                actual_degree = abs(BP.get_motor_encoder(BP.PORT_B))
                #print("actual degree ", actual_degree)
                BP.set_motor_dps(BP.PORT_B, speed_dps)
                time.sleep(0.02)
               
                #print("Rotation complete")
            #Measurement and calculations should be done here
            #print("rotation value %s" % BP.get_motor_encoder(BP.PORT_B))
            self.stop_robot()

            
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
# waypoints = [\
#     (84, 30),\
#     (180, 30),\
#     (180, 54),\
#     (138, 54),\
#     (138, 168),\
#     (114, 168),\
#     (114, 84),\
#     (84, 84),\
#     (84, 30)\
#     ]

waypoints = [(1.04,0.3), (1.24, 0.3), (1.44, 0.3), (1.64, 0.3), (1.8,0.3), 
        (1.8, 0.42), (1.8, 0.54),
        (1.6, 0.54), (1.38, 0.54),
        (1.38, 0.74), (1.38, 0.94), (1.38, 1.14), (1.38, 1.34), (1.38, 1.54), (1.38, 1.68),
        (1.14, 1.68),
        (1.14, 1.48), (1.14, 1.28), (1.14, 1.08), (1.14, 0.84),
        (0.99, 0.84), (0.84, 0.84),
        (0.84, 0.64), (0.84, 0.44), (0.84, 0.3)]
# Definitions of walls for test
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
# mymap.add_wall((0,0,0,168))         # a
# mymap.add_wall((0,168,84,168))      # b
# mymap.add_wall((84,126,84,210))     # c
# mymap.add_wall((84,210,168,210))    # d
# mymap.add_wall((168,210,168,84))    # e
# mymap.add_wall((168,84,210,84))     # f
# mymap.add_wall((210,84,210,0))      # g
# mymap.add_wall((210,0,0,0))         # h
# mymap.draw()
#Our test walls


try:
    # Initialise a robot object.
    r = robot()
    #r.rotate_top(3.455555,45)
    r.rotate_sonar(2*math.pi, 10)
    r.rotate_sonar(-2*math.pi, 1) #Rotate sonar back around to untangle
    r.stop_robot()
# Keyboard Interrupt.
except KeyboardInterrupt:
    r.stop_robot()
