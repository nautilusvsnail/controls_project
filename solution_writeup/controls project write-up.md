**Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.**
>The writeup / README should include a statement and supporting figures / images that explain how each rubric item was addressed, and specifically where in the code each step was handled.


# Body Rate Control
**Implement Body Rate Control in C++**
>The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.


## Statement
Implement the following formula for proportional control of body roll rates in each of three (body frame) axes, using `V3F` type to store moment commands and roll errors in each axis.
$$M_c = K_p * I * e(t)$$


## Code

``` C++
V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
	V3F momentCmd;
  
  V3F pqrError = pqrCmd - pqr;
  momentCmd.x = Ixx * kpPQR.x * pqrError.x;
  momentCmd.y = Iyy * kpPQR.y * pqrError.y;
  momentCmd.z = Izz * kpPQR.z * pqrError.z;
  
  return momentCmd;
}
```


# Roll/Pitch Control
**Implement Roll Pitch Control in C++**
>The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles.


## Statement
Obtain commanded acceleration in NED frame by dividing thrust command by -mass.
Obtain the transformation element of the tilt constraint by taking its cosine
Obtain the transformation elements for commanded tilt in the x- and y-directions (equivalent to the cosine of the tilt angles) by dividing commanded acceleration in the x and y by net acceleration.
Constrain the commanded x- and y-tilt elements by the maximum tilt cosine
Calculate a commanded rate of change of each rotational element using a proportional controller:
$$\dot b_c = K_p * (b_c - b_a)$$
Map the commanded rotational elements to rotational body velocities using the estimated attitude matrix R:
$$\begin{bmatrix} p_c \\ q_c \end{bmatrix}=\frac{1}{R_{33}}\begin{bmatrix}R_{21}&-R_{11}\\R_{22}&-R_{12}\end{bmatrix}\begin{bmatrix}\dot b^x_c\\\dot b^y_c\end{bmatrix}$$


## Code

```C++
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  V3F pqrCmd(0,0,0);
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  float c_acc = -collThrustCmd / mass; // collective acceleration due to thrust
  
  float max_tilt_cos = cos(maxTiltAngle);
  
  float bx_c = accelCmd.x / c_acc;
  float by_c = accelCmd.y / c_acc;
  
  bx_c = CONSTRAIN(bx_c, -max_tilt_cos, max_tilt_cos); // commanded x-tilt [transformation element]
  by_c = CONSTRAIN(by_c, -max_tilt_cos, max_tilt_cos); // commanded y-tilt [transformation element]
  
  // set commanded bank rates
  float bdot_xc = kpBank * (bx_c - R(0,2));
  float bdot_yc = kpBank * (by_c - R(1,2));
  
  // convert to p and q in body frame
  pqrCmd.x = (1 / R(2,2)) * ((R(1,0) * bdot_xc) - (R(0,0) * bdot_yc));
  pqrCmd.y = (1 / R(2,2)) * ((R(1,1) * bdot_xc) - (R(0,1) * bdot_yc));

  return pqrCmd;
}
```


# Altitude Control
**Implement altitude controller in C++**
>The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.
>
>Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4.


## Statement
Constrain z velocity command by max ascent and descent rates
Generate a commanded vertical acceleration using a PID controller with feedforward:
$$\bar u(t)=K_pe(t)+K_i\int_0^te(\tau)d\tau+K_d\dot e(t)+\ddot z_{targ}(t)$$
Constrain integral term to a preset value to limit integral windup
Use the vehicle's estimated attitude and mass to convert desired vertical acceleration to a thrust target:
$$t_c=\frac{(G-\ddot z_{targ})*m}{R_{22}}$$


## Code

```C++
float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
	Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;
  
  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
  float z_err = (posZCmd - posZ);
  float p_term = kpPosZ * z_err;
  integratedAltitudeError += z_err * dt;
  integratedAltitudeError = CONSTRAIN(integratedAltitudeError,-integratorConstraint, integratorConstraint);
  float i_term = KiPosZ * integratedAltitudeError;
  float d_term = kpVelZ * (velZCmd - velZ);
  float zdd_targ = p_term + i_term + d_term + accelZCmd;
  thrust = ((CONST_GRAVITY - zdd_targ) * mass) / R(2,2);
  
  return thrust;
}
```


# Lateral Position Control
**Implement lateral position control in C++**
>The controller should use the local NE position and velocity to generate a commanded local acceleration.


## Statement
Use a cascaded p-controller (instead of a PD controller) to command lateral acceleration:
$$\ddot u=K_v(K_p(u_c-u_a)-\dot u_a)+\ddot u_{targ}$$
Constrain lateral velocity and acceleration at each additive step


## Code

```C++
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;
  
  velCmd += kpPosXY * (posCmd - pos);
  velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
  velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);
  
  accelCmd += kpVelXY * (velCmd - vel);
  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);

  return accelCmd;
}
```


# Yaw Control
**Implement yaw control in C++**
>The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).


## Statement
Constrain yaw command and error values to $[-2\pi:2\pi]$
Use a proportional controller to command yaw rate

## Code

```C++
float QuadControl::YawControl(float yawCmd, float yaw)
{
  float yawRateCmd=0;
  float yawCmd_2pi = 0;
  
  // correct for yaw cmd outside -2pi:2pi
  if (yawCmd > 0)
  {
    yawCmd_2pi = fmodf(yawCmd, 2 * F_PI);
  }
  else
  {
    yawCmd_2pi = -fmodf(-yawCmd, 2 * F_PI);
  }

  // error
  float err = yawCmd_2pi - yaw;

  // correct for error value outside -2pi:2pi
  if (err > F_PI)
  {
    err -= 2 * F_PI;
  }
  if (err < -F_PI)
  {
    err += 2 * F_PI;
  }

  // calculate rate command
  yawRateCmd = kpYaw * err;

  return yawRateCmd;
}
```


# Motor Commands
**Implement calculating the motor commands given commanded thrust and moments in C++**
>The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

## Statement
Calculate the orthogonal distance along each axis to the rotors (this is done in `QuadControl::Init()`)
Convert the pqr moment commands into net forces across the xyz body axes by dividing by orth_l (x,y) and kappa (z)
Implement the solution to the following system of equations to calculate required motor thrust (don't mix up your motor mapping):
$$
\begin{pmatrix} 1 & 1 & 1 & 1 \\ 1 & -1 & -1 & 1 \\ 1 & 1 & -1 & -1\\ -1 & 1 & -1 & 1 \end{pmatrix} \times \begin{pmatrix} t_0 \\ t_1 \\ t_2\\ t_3 \end{pmatrix} = \begin{pmatrix} \bar{c} \\ \bar{f_x} \\ \bar{f_y} \\ \bar{f_z} \end{pmatrix}
$$

## Code

```C++
VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  float f_c = collThrustCmd;
  float f_p = momentCmd.x / orth_l;
  float f_q = momentCmd.y / orth_l;
  float f_r = momentCmd.z / kappa;
  
  // where does it tell you motor spin direction???? it's opposite the drone in the exercises
  // (used first - incorrect - solve)
  
  cmd.desiredThrustsN[0] = (f_c + f_p + f_q + -f_r) / 4.f; // forward port
  cmd.desiredThrustsN[1] = (f_c + -f_p + f_q + f_r) / 4.f; // forward starboard
  cmd.desiredThrustsN[2] = (f_c + f_p + -f_q + f_r) / 4.f; // aft port
  cmd.desiredThrustsN[3] = (f_c + -f_p + -f_q + -f_r) / 4.f; // aft starboard
  
  // desired thrust commands are constrained in quaddynamics

  return cmd;
}
```


# Flight Evaluation
**Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.**
>Ensure that in each scenario the drone looks stable and performs the required task. Specifically check that the student's controller is able to handle the non-linearities of scenario 4 (all three drones in the scenario should be able to perform the required task with the same control gains used).


## Gains

```
# Position control gains
kpPosXY = 3
kpPosZ = 10	
KiPosZ = 70

# Velocity control gains
kpVelXY = 12
kpVelZ = 24

# Angle control gains
kpBank = 12
kpYaw = 2

# Angle rate gains
kpPQR = 69, 69, 5

# limits
integratorConstraint = .3
```


## Scenario 2
![[scenario_2_pass.png]]


## Scenario 3

![[scenario_3_pass.png]]


## Scenario 4

![[scenario_4_pass.png]]


## Scenario 5

![[scenario_5_pass.png]]


