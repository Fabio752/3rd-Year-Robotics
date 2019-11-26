
# Imports.
import brickpi3
import time
import random
import math
import copy
#import statistics
import sys

BP = brickpi3.BrickPi3()
#SPECIFY PORTS
SONAR_MOTOR = BP.PORT_C
LEFT_MOTOR = BP.PORT_A
RIGHT_MOTOR = BP.PORT_B

SONAR_SENSOR = BP.PORT_4
LEFT_BUMPER = BP.PORT_3
RIGHT_BUMPER = BP.PORT_1

#BP.set_sensor_type(SONAR_SENSOR, BP.SENSOR_TYPE.NXT_ULTRASONIC)
BP.reset_all()

NUMBER_OF_PARTICLES = 100.0
general_weight = 1.0/NUMBER_OF_PARTICLES
distance_var_ratio = 0.10 # previously set to 1 (higher in order to show MCL working
standard_dev_angle = 0.015 # previously set to 0.015
likelihood_standard_dev = 3
CARPET = "doc"

INITIAL_POSITION = [84, 30, 0] # assessment done on a robot starting not from the origin
debug = True

# General purpose functions.
def sensor_instantiation():
    # Initialise sonar sensor
    # BP.reset_all()
    #BP.set_sensor_type(SONAR_SENSOR, BP.SENSOR_TYPE.NXT_ULTRASONIC)
    BP.set_sensor_type(LEFT_BUMPER, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(RIGHT_BUMPER, BP.SENSOR_TYPE.TOUCH)

def clear_encoders():
    BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))
    BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))

def get_encode_length(distance):
    return (distance / 47.0) * 819.5

def get_rotation_amount (rad):
    degree_amount = rad * 57.296
    return (degree_amount / 180.0) * 449 if CARPET == "doc" else (degree_amount / 110.0) * 360

def get_distance(encoder_val): 
    return (encoder_val /819.5) * 47.0

def get_angle(encoder_angle_amount):
    return (encoder_angle_amount * 180.0) / 444.0


def measure_sonar_global():
        BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.NXT_ULTRASONIC)
        while True:
            # read and display the sensor value
            # BP.get_sensor retrieves a sensor value.
            # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
            # BP.get_sensor returns the sensor value (what we want to display).
            try:
                # BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.NXT_ULTRASONIC)
                value = BP.get_sensor(BP.PORT_4)
                print(value)
                break # print the distance in CM
            except brickpi3.SensorError as error:
                #print(error)
                #BP.reset_all()
                # reset all sensors

                #BP.set_sensor_type(BP.PORT_1 + BP.PORT_2 + BP.PORT_3 + BP.PORT_4, BP.SENSOR_TYPE.NONE)
                BP.set_sensor_type(SONAR_SENSOR, BP.SENSOR_TYPE.NONE)
                time.sleep(0.02)
                BP.set_sensor_type(SONAR_SENSOR, BP.SENSOR_TYPE.NXT_ULTRASONIC)
                time.sleep(0.02)
                #BP.set_sensor_type(LEFT_BUMPER, BP.SENSOR_TYPE.TOUCH)
                #BP.set_sensor_type(RIGHT_BUMPER, BP.SENSOR_TYPE.TOUCH)
                        
            #time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.
        
        return value


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
        self.canvas = Canvas()
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

    def change_particle_set_angle(self, alpha):
        i = 0
        for particle in self.PARTICLE_SET:
            theta_new = alpha 
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

    
    def find_wall(self, robo_map, x, y, theta):
        chosen_wall = None #tuple of chosen wall + m value
        for wall in robo_map.walls:
            # Compute m
            # Ax = chosen_wall[0], Ay = chosen_wall[1], Bx = chosen_wall[2], By = chosen_wall[3]
            print("particle values (x,y,theta): %s %s %s" % (x,y,theta))
            m = ((wall[3] - wall[1]) * (wall[0] - x) - (wall[2] - wall[0]) * (wall[1] - y)) /\
                ((wall[3] - wall[1]) * math.cos(theta) - (wall[2] - wall[0]) * math.sin(theta))
            if debug: print("Wall: ",wall,"m: ",m)
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
        
        return chosen_wall

    # Update particle weights (using likelihood).
    def calculate_likelihood(self, robo_map, x, y, theta, z):
        # z = sonar measurement
 
        # Find out which wall the sonar should be pointing to, and the expected distance m
        chosen_wall = self.find_wall(robo_map, x, y, theta)

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

    def use_MCL(self, wall):
        #Create new map object with chosen wall
        single_wall_map = Map()
        single_wall_map.add_wall(wall)
        self.state.update_particle_set_weights(single_wall_map, self.measure_sonar())
        self.state.resample_particle_set()
       
    # Motion functions.
    # Straight motion.
    def go_forward(self, distance, speed_dps):
        reversing = False
        if speed_dps < 0:
            reversing = True
        # Negating speed.
        speed_dps = -speed_dps
        clear_encoders()
        # Initializations.
        target_degree_rotation = get_encode_length(distance)
        actual_degree_rotation = 0
        time.sleep(0.5)
        BP.set_motor_dps(LEFT_MOTOR, speed_dps)
        BP.set_motor_dps(RIGHT_MOTOR, speed_dps)
        time.sleep(0.5)

        left_hit = False
        right_hit = False
        print("distance %s, target degree %s"% (distance, target_degree_rotation))

        while actual_degree_rotation < target_degree_rotation:
            if reversing:
                actual_degree_rotation = (BP.get_motor_encoder(LEFT_MOTOR) + BP.get_motor_encoder(RIGHT_MOTOR)) / 2
            else:
                actual_degree_rotation = -(BP.get_motor_encoder(LEFT_MOTOR) + BP.get_motor_encoder(RIGHT_MOTOR)) / 2

        
        self.stop_robot()
        # Update particle set.
        
        print("distance amount: ", distance)
        standard_dev_distance  = 1
        self.state.update_particle_set_line(0, standard_dev_distance, 0, 0.01, -distance if reversing else distance)
        #self.map.canvas.drawParticles(self.state.PARTICLE_SET)



       # Update robot position and orientation estimates.
        self.set_estimate_location()
        self.stop_robot()
    
    def forward_till_hit(self, distance, speed_dps):
        print("Location before forward till hit: ", self.get_estimate_location())

        # Negating speed.
        speed_dps = -speed_dps
        clear_encoders()
        # Initializations.
        target_degree_rotation = get_encode_length(distance)
        actual_degree_rotation = 0
        time.sleep(0.5)
        BP.set_motor_dps(LEFT_MOTOR, speed_dps)
        BP.set_motor_dps(RIGHT_MOTOR, speed_dps)
        time.sleep(0.5)

        left_hit = False
        right_hit = False
        rotation_due_to_hit = 0
        print("target degree", target_degree_rotation)

        while actual_degree_rotation < target_degree_rotation:
            print("Actual degree: ", actual_degree_rotation)
            actual_degree_rotation = -(BP.get_motor_encoder(LEFT_MOTOR) + BP.get_motor_encoder(RIGHT_MOTOR)) / 2
            sensor_instantiation()

            #print("actual degree", actual_degree_rotation, "target degree", target_degree_rotation)
            while True:
                try:
                    #Note: when the robot hits the bottle (especially from the side), it often gets rotated a little
                    #the encoders will detect this, hence we should account for this change in the robot orientation
                    #right_hit = BP.get_sensor(RIGHT_BUMPER)
                    if BP.get_sensor(RIGHT_BUMPER):
                        right_hit = True
                        print("before: ",BP.get_motor_encoder(LEFT_MOTOR))
                        encoder_before_hit = (abs(BP.get_motor_encoder(LEFT_MOTOR)) + abs(BP.get_motor_encoder(RIGHT_MOTOR)))/2
                        BP.reset_all()
                        print("After: ", BP.get_motor_encoder(LEFT_MOTOR))
                        rotation_due_to_hit = (abs(BP.get_motor_encoder(LEFT_MOTOR)) + abs(BP.get_motor_encoder(RIGHT_MOTOR)))/2 - encoder_before_hit
                        break
                    time.sleep(0.02)
                    #left_hit = BP.get_sensor(LEFT_BUMPER)
                    if BP.get_sensor(LEFT_BUMPER):
                        left_hit = True
                        print("before: ",BP.get_motor_encoder(LEFT_MOTOR))
                        BP.reset_all()
                        print("After: ", BP.get_motor_encoder(LEFT_MOTOR))
                        break
                        self.stop_robot()
                    #print((right_hit, left_hit))
                    break
                except brickpi3.SensorError as error:
                    pass
                    #print(error)
                    #print("sensor touch")

                #Weirdest bug: You need to have to have this structure to be able to correctly read from the rouch sensors

            if left_hit or right_hit:
                hit = left_hit if left_hit else right_hit
                print("HIT MOFOS!", hit)
                break     

        #Account for possible rotation when hitting
        self.state.update_particle_set_angle(0, standard_dev_angle, rotation_due_to_hit)
        #Update distance in case robot hits early
        distance = get_distance(abs(BP.get_motor_encoder(LEFT_MOTOR) + BP.get_motor_encoder(RIGHT_MOTOR))/2)\
             if (left_hit or right_hit) else distance
        print("distance amount: ", distance)
        standard_dev_distance  = 1
        self.state.update_particle_set_line(0, standard_dev_distance, 0, 0.01, distance)
        #self.map.canvas.drawParticles(self.state.PARTICLE_SET)

        self.set_estimate_location()

        print("Location after forward till hit: ", self.get_estimate_location())


        return (left_hit or right_hit)

    def forward_till_sonar(self, speed_dps, sonar_threshold):
             # Negating speed
            speed_dps = -speed_dps
            clear_encoders()
            # Initializations.
            target_degree_rotation = get_encode_length(100)
            actual_degree_rotation = 0
            time.sleep(0.5)
            BP.set_motor_dps(LEFT_MOTOR, speed_dps)
            BP.set_motor_dps(RIGHT_MOTOR, speed_dps)
            time.sleep(0.5)
    
            met_threshold = False
            print("target degree", target_degree_rotation)
    
            while actual_degree_rotation < target_degree_rotation:
                actual_degree_rotation = -(BP.get_motor_encoder(LEFT_MOTOR) + BP.get_motor_encoder(RIGHT_MOTOR)) / 2
                sensor_instantiation()
                BP.set_motor_dps(LEFT_MOTOR, speed_dps)
                BP.set_motor_dps(RIGHT_MOTOR, speed_dps)
    
                #print("actual degree", actual_degree_rotation, "target degree", target_degree_rotation)
                while True:
                    try:    
                        sonar_value = self.measure_sonar(2)
                        if sonar_value < sonar_threshold:
                            print ("object found alongside")
                            met_threshold = True
                            BP.reset_all()
                            break
                        break
                        #print((right_hit, left_hit))
                    except brickpi3.SensorError as error:
                        pass
                        #print(error)
                        #print("sensor touch")
    
                    #Weirdest bug: You need to have to have this structure to be able to correctly read from the rouch sensors
    
                if met_threshold:
                    print("Close enough to wall")
                    break     
    
            #Update distance in case robot hits early
            distance = get_distance(abs(BP.get_motor_encoder(LEFT_MOTOR) + BP.get_motor_encoder(RIGHT_MOTOR))/2)\
                 if (met_threshold) else distance
            print("distance amount: ", distance)
            standard_dev_distance  = 1
            self.state.update_particle_set_line(0, standard_dev_distance, 0, 0.01, distance)
            #self.map.canvas.drawParticles(self.state.PARTICLE_SET)
    
            self.set_estimate_location()
    
            print("Location after forward till hit: ", self.get_estimate_location())    
            return met_threshold

    def calculate_angle_to_wall(self, wall):
        beta = math.acos(math.cos(self.estimate_location [2])*(wall[1] - wall[3]) + math.sin(self.estimate_location [2])*(wall[2] - wall[0]) / 
        math.sqrt(math.pow((wall[1] - wall[3]), 2) + math.pow((wall[2] - wall[0]), 2)))
        return math.degrees(beta)


    def forward_till_hit_w_sonar(self, distance, speed_dps, sonar_threshold):
        print("Location before forward till hit: ", self.get_estimate_location()) 
        # Negating speed
        speed_dps = -speed_dps
        clear_encoders()
        # Initializations.
        target_degree_rotation = get_encode_length(distance)
        actual_degree_rotation = 0
        time.sleep(0.5)
        BP.set_motor_dps(LEFT_MOTOR, speed_dps)
        BP.set_motor_dps(RIGHT_MOTOR, speed_dps)
        time.sleep(0.5)

        left_hit = False
        right_hit = False
        seen_object = False
        rotation_due_to_hit = 0
        previous_value = 0
        current_value = 0
        cumulative_error = 0
        print("target degree", target_degree_rotation)

        while actual_degree_rotation < target_degree_rotation:
            actual_degree_rotation = -(BP.get_motor_encoder(LEFT_MOTOR) + BP.get_motor_encoder(RIGHT_MOTOR)) / 2
            sensor_instantiation()
            BP.set_motor_dps(LEFT_MOTOR, speed_dps)
            BP.set_motor_dps(RIGHT_MOTOR, speed_dps)

            #print("actual degree", actual_degree_rotation, "target degree", target_degree_rotation)
            while True:
                try:
                    #Note: when the robot hits the bottle (especially from the side), it often gets rotated a little
                    #the encoders will detect this, hence we should account for this change in the robot orientation
                    #right_hit = BP.get_sensor(RIGHT_BUMPER)
                    if BP.get_sensor(RIGHT_BUMPER):
                        right_hit = True
                        print("before: ",BP.get_motor_encoder(LEFT_MOTOR))
                        encoder_before_hit = (abs(BP.get_motor_encoder(LEFT_MOTOR)) + abs(BP.get_motor_encoder(RIGHT_MOTOR)))/2
                        BP.reset_all()
                        print("After: ", BP.get_motor_encoder(LEFT_MOTOR))
                        rotation_due_to_hit = (abs(BP.get_motor_encoder(LEFT_MOTOR)) + abs(BP.get_motor_encoder(RIGHT_MOTOR)))/2 - encoder_before_hit
                        break
                    time.sleep(0.02)
                    #left_hit = BP.get_sensor(LEFT_BUMPER)
                    if BP.get_sensor(LEFT_BUMPER):
                        left_hit = True
                        print("before: ",BP.get_motor_encoder(LEFT_MOTOR))
                        BP.reset_all()
                        print("After: ", BP.get_motor_encoder(LEFT_MOTOR))
                        break
                    time.sleep(0.02)

                    sonar_value = self.measure_sonar(2)

                    if sonar_value < sonar_threshold:
                        print ("object found alongside")
                        seen_object = True
                        #Account for sonar measuring things too early
                        time.sleep(0.5)
                        BP.reset_all()
                        break
                    break


                    #print((right_hit, left_hit))
                except brickpi3.SensorError as error:
                    pass
                    #print(error)
                    #print("sensor touch")

                #Weirdest bug: You need to have to have this structure to be able to correctly read from the rouch sensors

            if left_hit or right_hit or seen_object:
                hit = left_hit if left_hit else right_hit
                print("HIT MOFOS!", hit)
                break     

        #Account for possible rotation when hitting
        self.state.update_particle_set_angle(0, standard_dev_angle, rotation_due_to_hit)
        #Update distance in case robot hits early
        distance = get_distance(abs(BP.get_motor_encoder(LEFT_MOTOR) + BP.get_motor_encoder(RIGHT_MOTOR))/2)\
             if (left_hit or right_hit or seen_object) else distance
        print("distance amount: ", distance)
        standard_dev_distance  = 1
        self.state.update_particle_set_line(0, standard_dev_distance, 0, 0.01, distance)
        #self.map.canvas.drawParticles(self.state.PARTICLE_SET)

        self.set_estimate_location()

        print("Location after forward till hit: ", self.get_estimate_location())


        return seen_object


    def peek_left(self, old_sonar): #why is rotation so inaccurate?
        self.rotate_sonar(-0.4, 90)
        time.sleep(0.5) #to remove later
        new_sonar = self.measure_sonar()
        if debug:
            print("peeked left. old sonar:", old_sonar,  "new sonar:", new_sonar)

        #time.sleep(1)
        if new_sonar <  old_sonar:
            self.rotate(0.4, 90)

        self.rotate_sonar(-0.4, 90)

        return

    def measure_sonar(self, threshold = 5):
        #THRESHOLD = 10 #number of readings before value converges
        values =[]
        sensor_value = 0
        count = 0
        while count < threshold:
            sensor_value = measure_sonar_global()
            if (sensor_value < 180) and (sensor_value != 21) and (sensor_value != 20):
                count += 1
                values.append(sensor_value)


        #sensor_value = statistics.median(values)
        sensor_value = values[len(values) // 2]
        print("Sonar: ", sensor_value)
        return sensor_value

    def go_to_bottle_1(self):
        sensor_value = self.measure_sonar(5)
        cum_turn = 0 
        #bottle is close
        if sensor_value < 40:
            self.forward_till_hit(40, 200)
            print("Location: ", self.get_estimate_location())


        #bottle is in field of vision of sonar
        elif sensor_value < 75:
            self.go_forward(sensor_value - 40, 500)
            sensor_value = self.measure_sonar(4)
            print("Location: ", self.get_estimate_location())
            if sensor_value < 40:
                self.peek_left(sensor_value)
                self.forward_till_hit(sensor_value - 10, 200)
                print("Location: ", self.get_estimate_location())
            else:
                if debug:
                    print("new sonar reading > 40, turning")
                self.rotate(0.5,90)
                cum_turn += 0.5
                print("Location: ", self.get_estimate_location())
            self.forward_till_hit(40, 200)
            print("Location: ", self.get_estimate_location())
        
    #bottle outside field of vision
        else:
            self.rotate(0.5, 90)
            cum_turn += 0.5 
            print("Location: ", self.get_estimate_location())
            sensor_value = self.measure_sonar(4)
            if sensor_value > 90:
                self.rotate(0.4, 90)
                cum_turn += 0.4
                print("Location: ", self.get_estimate_location())
                self.forward_till_hit(40, 200)
                print("Location: ", self.get_estimate_location())
                return
            else:
                if sensor_value < 40:
                    self.forward_till_hit(sensor_value - 10, 200)
                else:

                    self.go_forward(sensor_value - 40, 500)
                    print("Location: ", self.get_estimate_location())
                    sensor_value = self.measure_sonar(4)
                    if sensor_value < 40:
                        #self.peek_left(sensor_value)
                        self.forward_till_hit(sensor_value - 10, 200)
                        print("Location: ", self.get_estimate_location())
                    else:
                        if debug:
                            print("new sonar reading > 40, turning")
                        self.rotate(0.5, 90)
                        cum_turn += 0.5
                        print("Location: ", self.get_estimate_location())
                        self.forward_till_hit(40, 200)
                        print("Location: ", self.get_estimate_location())
        return cum_turn

    def go_to_bottle_2(self):
        # TODO Check if object dead ahead
        
        print("GOING TO BOTTLE 2")
        threshold = 40
        forward_val = r.measure_sonar()
        if forward_val < threshold:
            r.go_forward(forward_val - 40, 300)
            r.forward_till_hit (40, 250)
            seen_object = False
        else:
            r.rotate_sonar(math.pi/2, 270)
            #Take measurement to make correction
            #initial_distance = self.measure_sonar(4)
            #r.go_forward(15, 180)
            #final_distance = self.measure_sonar(4)
            #print("initial - final: ", initial_distance - final_distance)
            #if -5 < (initial_distance - final_distance) and (initial_distance - final_distance) < 0:
            #    r.rotate(-0.05, 90)
   
            seen_object = r.forward_till_hit_w_sonar(190-r.get_estimate_location () [1], 300, 40)
            r.rotate_sonar(math.pi/2, -270)
        
        if seen_object:
            r.rotate(-math.pi/2, 90)
            r.forward_till_hit(30, 250)
            r.go_forward(15, -350)
            r.rotate(-math.pi/2, 90)
            r.forward_till_sonar(350, 85)
            r.rotate(-math.pi/2, 90)
        else:
            r.go_forward(5, -350)
            r.rotate_sonar(math.pi, 270)
            r.forward_till_sonar(-350, 85)
            r.rotate_sonar(-math.pi, 270)
            r.rotate(math.pi/2, 90)

        return

    def go_to_bottle_3(self):
        print("GOING TO BOTTLE 3")
        self.state.change_particle_set_angle(math.pi/2)
        self.set_estimate_location()
        cum_turn = 0
        #Get to Area3     
        sensor_value = self.measure_sonar(5)
        print("Sensor value : ", sensor_value)
        
        #bottle is close
        if sensor_value < 100:
            #do algorithm 1
            self.forward_till_hit(40, 200)
            new_measurement = self.measure_sonar(5)
            if new_measurement > (sensor_value - 40):
                self.rotate(-0.5, 180)
                cum_turn += 0.5
            self.forward_till_hit(new_measurement, 250)




            #include return to not run rest of code

        #bottle outside field of vision
        else:
            self.go_forward(sensor_value - 70, 450)
            seen_object = r.forward_till_hit_w_sonar(70,  300, 40)
            self.rotate_sonar(math.pi/2, -270)


        if seen_object:
            self.rotate(-math.pi/2, 90)
            self.forward_till_hit(80, 250)
            self.go_forward(15, -350)
            #Need to turn sonar by 180
            self.rotate_sonar(math.pi, 270)
            #Turn the sensor and start measuring
            y_coord = r.measure_sonar()
            #BP.offset_motor_encoder(SONAR_MOTOR, BP.get_motor_encoder(SONAR_MOTOR))
            r.rotate_sonar(-math.pi/2, 270)
            x_coord = self.measure_sonar()
            self.go_forward(y_coord - 30, i-300)
            self.rotate(-math.pi/2, 90)
            self.go_forward(84 - x_coord, 300)

        return cum_turn



    # Rotation on the spot.
    def rotate(self, rad_amount, speed_dps): 
        # Negate the speed if has to turn left, keep it for turning right.
        if rad_amount > 0: speed_dps = -speed_dps
        if debug:
            print("rad amount: ", rad_amount)
        # Set opposite wheels speed.
        actual_degree = 0
        #time.sleep(0.5)
        target_rotation = get_rotation_amount(abs(rad_amount))
        clear_encoders()


        # Rotation on the spot.
        while True:
            BP.set_motor_dps(LEFT_MOTOR, -speed_dps)
            BP.set_motor_dps(RIGHT_MOTOR, speed_dps)
            actual_degree = (abs(BP.get_motor_encoder(RIGHT_MOTOR)) + abs(BP.get_motor_encoder(LEFT_MOTOR)))/2
            if  actual_degree >= target_rotation:
                self.stop_robot()      
                break
            time.sleep(0.02)

        # Update the particle set and location estimates.
        # We should not randomize unless we are not using MCL
        self.state.update_particle_set_angle(0, standard_dev_angle, rad_amount)
        self.set_estimate_location()

    def search_and_go_to_obj (self):
            time.sleep(1)
            sensor_instantiation() 

            robot_angle = 0
            target_found = False

            self.rotate(math.pi/2, 30)

            BP.set_motor_dps(LEFT_MOTOR, 30)
            BP.set_motor_dps(RIGHT_MOTOR, -30)

            print("setting motor dps worked")

            while robot_angle < 90:
                print("getting the estimate location")
                robot_angle = math.degrees(self.get_estimate_location() [2])
                print("The robot angle is", robot_angle)
                #Measurement every 10 degrees (+-1 degree)
                if abs(robot_angle % 10) < 1:
                    while True:
                        try:
                            #dis = BP.get_sensor(SONAR_SENSOR)
                            #print("distance computed", dis)
                            break
                        except IOError as e:
                            print("Warning: IO Error, skipping MCL...")
                            print(e)
                        except brickpi3.SensorError as e:
                            #print("Going forward")
                            print(e)
                            #BP.reset_all()
                            #sensor_instantiation()

                    time.sleep(0.02)
                    
                    #Spread particles
                    self.state.update_particle_set_angle(0, standard_dev_angle, 10)
                    
                    #Figure out which wall it should be facing
                    #TODO we need to add to the angle the value of robot anglei
                    chosen_wall = self.state.find_wall(self.map, self.get_estimate_location() [0], self.get_estimate_location() [1], self.get_estimate_location() [2])
                    print("chosen wall is", chosen_wall)
                    #Object is at least 20 cm away from any wall, check if difference greater than this
                    print(abs(dis - chosen_wall[3]))
                    if abs(dis - chosen_wall[3]) > 15:
                        #Object hit, need to rotate robot by the sonar angle
                        target_found = True
                        break

                self.stop_robot()

                if target_found:
                    #Move the robot forward by the sonar reading
                    self.go_forward(dis, 50, False)
                else:
                    #TODO do we scan again? no
                    #TODO swipe through the entire section lol (intelligently)
                    print("Target not found")

    # Move to a specific coordinate by :
    #    1 - rotating.
    #    2 - go straight.
    def navigate_to_waypoint(self, x, y): 
        # Compute coordinate differences and scale meters->centimeters.
        x = x * 100
        y = y * 100
        x_diff = x - self.estimate_location[0]
        y_diff = y - self.estimate_location[1]
        rotation_amount = 0
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
                '''
            if (x_diff < 0): 
                rotation_amount = rotation_amount + math.pi'''

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

        
        print("rotation amount ", rotation_amount)
        # Step1: rotate by the amount we are off by.
        self.rotate(rotation_amount, 90)

        # Compute distance to travel.
        distance_amount = math.sqrt(pow(x_diff, 2) + pow(y_diff, 2))

        # Step2: go forward.
        self.go_forward(distance_amount, 350)
        #recalculate particle distribution
        #self.map.canvas.drawParticles(self.state.PARTICLE_SET)
    
    def rotate_sonar(self, radians, speed_dps):
        BP.offset_motor_encoder(SONAR_MOTOR, BP.get_motor_encoder(SONAR_MOTOR))
        if radians < 0:
            speed_dps = - speed_dps

        actual_rotation = 0
        target_rotation = math.degrees(abs(radians)) * 1
        while actual_rotation < target_rotation:
            actual_rotation = abs(BP.get_motor_encoder(SONAR_MOTOR))
            BP.set_motor_dps(SONAR_MOTOR, speed_dps)
            time.sleep(0.02)

        BP.set_motor_dps(SONAR_MOTOR, 0)
        return


    # Debugging Function.
    def print_robot_stats(self, port):
        print("Port: %s Flag: %s Power: %s Position: %s Velocity: %s" % (port, BP.get_motor_status(port) [0], BP.get_motor_status(port) [1], BP.get_motor_status(port) [2], \
        BP.get_motor_status(port) [3]))

    # Stop the robot.
    def stop_robot(self):
        BP.reset_all()
        clear_encoders()


##################################################
#STARTING SCRIPT
##################################################
# KEY IDEA :
# 1. Drive to the centre of each section
# 2. If during the drive you bump into something update the particle set with new particles in that estimate location and go back to 1.
# 3. Else, do a 360 on the spot looking for a weird measurement and infer the x and y of the object
# 4. Drive into the object and register the bump
# 5. Update the particle set as in point 2., back off a bit and proceed according to 1.
# 6. Win this moth*****ing competition

waypoints = [(1.55,0.3)] #hardcode centers of sections

try:
    r = robot()
    r.map.add_wall((0,0,0,168))
    r.map.add_wall((0,168,84,168))
    r.map.add_wall((84,126,84,210))
    r.map.add_wall((84,210,168,210))
    r.map.add_wall((168,210,168,84))
    r.map.add_wall((168,84,210,84))
    r.map.add_wall((210,84,210,0))
    r.map.add_wall((210,0,0,0))
    r.map.draw()

    test_sonar = True

    if test_sonar:
        while True:
            r.measure_sonar()
        sys.exit()

    #Instantianting the sensor used
    sensor_instantiation()

    # BOTTLE 1 (TO OPTIMIZE)
    r.navigate_to_waypoint(1.16, 0.3)
    theta = r.go_to_bottle_1() 

    # Reverse and reposition after hit
    r.go_forward(15, -350)
    r.rotate(-theta, 90)
    #backward value depending on distance
    backward_speed  = -350
    x = r.estimate_location [0]
    if (x < 93):
        backward_speed = 350
   
    r.go_forward(abs(101 - x), backward_speed)
    # Here we should check with sonar that we are parallel to the wall

        #BOTTLE 2 (TO OPTIMIZE)
    r.rotate((math.pi/2)-0.055, 90)
    r.go_forward(84- r.estimate_location[1],350)
    #r.navigate_to_waypoint(1.04, 1.04)
    r.go_to_bottle_2()

    bottle_3_turn_correction = r.go_to_bottle_3()
    print("Bottle 3 turn correction: ", bottle_3_turn_correction)
    r.go_forward(5, -350)
    r.rotate(bottle_3_turn_correction, 90)
    r.rotate(math.pi/2, 90)
    r.go_forward(30, 300)

    y_coord = r.measure_sonar()
    #BP.offset_motor_encoder(SONAR_MOTOR, BP.get_motor_encoder(SONAR_MOTOR))
    r.rotate_sonar(math.pi/2, 270)
    print("sonar rotated")
    x_coord = r.measure_sonar()

    r.go_forward(y_coord - 30, 300)
    r.rotate(math.pi/2, 90)
    r.go_forward(84 - x_coord, 300)

    #r.navigate_to_waypoint(0.84,0.30)
   

    r.stop_robot()

# Keyboard Interrupt.
except KeyboardInterrupt:
    r.stop_robot()
