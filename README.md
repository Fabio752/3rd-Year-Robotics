### Assignment 1

**Investigating And Understanding Motor Control**

*LEGO-Motor_DPS.py*

* Setting a power limit forces the power of the motor to be bounded at a certain value.Because the power is proportional to the current supplied, the velocity is proportional to the power and hence will not increase beyond a limit. Hence, the velocity reading was read in a range of 475-525 for power cap 50.

* The encoder value changes according to how fast the wheel is going, so at very low power, when velocity is low, it increases by 0 or 1 every time that the motor status is displayed.
In the case of power = 50, when the velocity is higher, it increases by three/four encoder values at a time.

* If we apply resistance to the powered wheel, the power applied to the corresponding motor will incrase in an attempt to keep the velocity stable, due to the PID control.
The PID controller will detect the error (difference) between the nominal velocity of the wheel and the actual velocity, and will therefore increase the power supplied to the motors. We see that there is little change in the velocity when we resist the wheels, but there are power increses when the wheels encounter resistance. This type of PID control setup allows the robot tp have steady velocities even if it encounters resistance when performing tasks like going up a steep piece of ground. 
When we suddenly release the resistance imposed to the wheel, the wheels will spin much faster for a few moments given the higher power being supplied but will quickly adjust, again because of the PID Compensation.

*LEGO-Motor_Power.py*

* In this script, since we are not demanding a certain velocity (velocity is not PID controlled), if we resist a wheel, the power supplied will not increase, thus the velocity will decrease.
Everything else is the same as the LEGO-Motor_DPS.py file


*LEGO-Motor_Position.py*

* Moving one wheel makes the encoder value change, this causes the motor driving the other wheel to move forward until the two encoder values are the same

**Calculating the Covariance Matrix**

| Try   |  X(cm)  | Y(cm)    |  
|---|---|---|
|  1 | 0.2  | -0.1  |   
|  2 |  -0.8 | 0 |   
|  3 |  -1.7 | -0.8  |   
|  4 |  0.5  |  0.5 |   
|  5 |  1.4 |  0.8 |   
|  6 |  0.4 |  1.1 |   
|  7 |  0.2 |  1.2 |  
|  8 |  0 | 1.6  |  
|  9 |  0.6 |  1.9 |   
|  10 |  1.2 |  2.2 |   


E(X) = 0.2
Var(X) = 0.738

E(Y) = 0.84
Var(Y) = 0.8144

Cov(X,Y) = 0.549

Standard Deviation of X = 0.859
Standard Deviation of Y = 0.902

Given the definition of the Covariance Matrix:

| Covariance  |  Matrix  | 
|---|---|
|  Var(X) = 0.738  | Cov(X,Y) = 0.549  |    
|  Cov(X,Y) = 0.549 |  Var(Y) = 0.8144  |


**For further thought**

*Which causes a larger effect on your robot, imprecision in drive distance or rotation angle?*

* The thing which can cause the largest imprecision in the drive in the robot in our case is the fact that the initial trajectory is not perfectly straight. This means that the initial rotation is not perfectly precise, we will have an error that will compound through the next two turns and will make the final distance to the point larger. In our case, the rotation angle causes a bigger effect in our robot that the rotation angle. 
* 

--------------------------


### Assignment 3 : Probabilistic Motion and Sensing

**Sonar Investigation**

*1)When placed facing and perpendicular to a smooth surface such as a wall, what are the minimum and maximum depths that the sensor can reliably measure?*

The ultrasonic sensor functions by sending a pulse of ultrasonic sound (which is above the human hearing range in the frequency spectrum) and measuring the amount of time it takes for the pulse to be reflected and be measured by the sensor again. By making use of our knowledge of the speed of sound, we can easily calculate the distance to an object based on the time it takes for the waveform to come back.

The distance the sensor displays in the example program is measured in cm, and encoded using a single byte, meaning that the minimum distance measured is 0cm and the maximum distance is in theory 255cm. However, the number 255 is kept for objects that are too far away to measure. In reality, the minimum distance that is measured in reality is about 4 cm. 

According to the specification of the sensor, the sensor is able to measure distances from 0 to 2.5 meters with a precision of +/-3 cm. Also, large-sized objects with hard surfaces provide the best readings. Objects made from soft fabrics, from curved objects (e.g. a ball), or from very thin and small objects can be diffi cult for the sensor to read.

Minimum distance: 4cm
Maximum distance: 165cm

*2)Move the sonar so that it faces the wall at a non-orthogonal incidence angle. What is the maximum angular deviation from perpendicular to the wall at which it will still give sensible readings?*

We placed the robot at a fixed distance of 30 cm from an arbitrary point on the wall. Varying the angle, we made the following measurements:

|Angle (degrees)| Measurement(cm) | % Error|
|---|---|---|
| 0 | 30 | | 0 |
| 15 | 29 | 3.3 |
| 20 | 29 | 3.3 |
| 25 | 28 | 6.6 |
| 30 | 26 | 13.2 |
| 45 | 24 | 20 |
| 50 | 22 | 26.4 |
| 55 | 21 | 30 |
| 60 | 255 | NA |

If we set the threshold for "sensibility" to be that readings must be within an accuracy of 5% of the ground truth, we can observe that the maximum acceptable angular deviation is between 20 - 25 degrees. Due to human and environmental errors we were unable to obtain a tighter bound on this. Maximum deviation also depends on the threshold used to define "sensible".


*3)Do your sonar depth measurements have any systematic (non-zero mean) errors? To test this, set up the sensor at a range of hand-measured depths (20cm, 40cm, 60cm, 80cm, 100cm) from a wall and record depth readings. Are they consistently above or below what they should be?*

Readings are in the txt files, created by piping output of the example NXT-ultrasonic program. By performind simple statistical analysis on all 120 readings per distance amount, we obtained the following results.

|Distance(cm)| Measurement average(cm) (1dp) | Measured standard deviation(cm) | % inaccuracy (based on mean) |
|---|---|---|---|
| 20 | 20.0 | 0 | 0 |
| 40 | 40.0 | 0.05 | 0 | 
| 60 | 60.0 | 0.07 | 0 |
| 80 | 80.0 | 0.09 | 0 |
| 100 | 100.9 | 0.29 | 0.91 |



*4)What is the the accuracy of the sonar sensor and does it depend on depth? At each of two chosen hand-measured depths (40cm and 100cm), make 10 separate depth measurements (each time picking up and replacing the sensor) and record the values. Do you observe the same level of scatter in each case?*

The sonar sensor does not seem to have a fixed percentage accuracy - there is however a +-1cm error to be expected in the measurement of all distances <150cm (based on our observations). 

Readings obtianed for gound truth of 40cm: (40, 39, 39, 40, 40, 40, 40, 40, 40, 41)
Readings obtained for ground truth of 100cm: (100, 100, 99, 99, 99, 99, 99, 99, 101, 101)

There seens to be a slightly greater spread for the 100cm ground truth case, but again this can be put down to environmental and human factors. The absolute error in both cases is 1cm, which corresponds to 2.5% in the 40cm case and 1% in the 100cm case.

*5)In a range of general conditions for robot navigation, what fraction of the time do you think your sonar gives garbage readings very far from ground truth?*

The accuracy of robot naviagtion depends on the environment it is placed in. If the environment is such that conditions are changing (through the presence of other bodies/moving objects/EM interference), or that there are points in the environment that exceed the Sonar's threshold for reliable readings (ie some walls are more than 165 cm away, while some objects are <4 cm to the robot), then the percentage of time that the robot is exposed to these conditions would correspond with the fraction of time that the sonar gives garbage readings. 

However, in a closed and reasonable environment (reasonable relative to the quality of hardware available to us) we can safely assume that the sonar will not give garbage values. There may be some slight deviation from gound truth (as explored above) but this can be mitigated with software.

### Assignment 4 : Monte Carlo Localisation

#### Sonar Likelihood and Measurement Update

As described in the specification, the function *calculate_likelihood*  was implented, doing the following:

* For each particle, read the x,y and angle measurements
* Using the sonar measurement, compute the distance *m* in the direction of the vector represented by the particle values to the  (if the wall were an infinite line).
* Check if robot is actually facing the wall, by seeing if the x and y interesects as calculated via the distance *m* map to the segment representing the wall.
  * If the robot coulding be facing multiple valid wall segments, take the closest one.
* Calculate angle between the line of the robot facing the wall and the normal to the wall, rejecting angles that are too large (and hence would give an innacurate reading).
* Compute the difference between the theoretical distance *m* and the actual sonar measurement, and use with likelihood function to scale the weight of the particle according to how close it was to the robot's measured position.

#### Normalising and Resampling

The *calculate_likelihood* function is called within the *update_particle_set_weights*, which after scaling all the weights in the particle set, will normalise them by summing up all the weights and dividing each particle wieght by this total, such that the sum of the weights is equal to 1 again.

Resampling is done in a seperate *resample_particle_set*, which works in the following way:

* Generate a cumulative weight array, where the index represented the particles of the particle set (in order) and the value is a cumulative weight, such that the last element will have value 1
* Generate a random number between 0 and 1
* Use the random number to select the index of the array, whose value is greater than this (indicating it is in the weight range of the particle)
* Store the chosen particle into a new particle set
* Repeat until a entirely new set is generated

The idea of this is that the particles with a higher weight are more likely to be present in the resampled set. Since the weights were previously scaled depending on how different they were from the robot's inital positon, the particles that were less accurate are less likely to be present in the new set. Hence, the resampled set will be a lot more concentrated around the actual location of the robot.


