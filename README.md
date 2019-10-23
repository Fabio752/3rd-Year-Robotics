### Assignement 1

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
Try         X(cm)    Y(cm)    
1           0.2      -0.1
2           -0.8      0 
3           -1.7     -0.8
4           0.5       0.5
5           1.4       0.8
6           0.4       1.1
7           0.2       1.2
8           0         1.6
9           0.6       1.9
10          1.2       2.2

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
