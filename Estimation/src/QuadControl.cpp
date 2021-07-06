#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

#include <iostream>
using namespace std;

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float length_perp = L / sqrtf(2.f);
  float f1 = momentCmd.x / length_perp;
  float f2 = momentCmd.y / length_perp;
  float f3 = momentCmd.z * (-1) / kappa;

  cmd.desiredThrustsN[0] = (f1 + f2 + f3 + collThrustCmd)/4.f;  
  cmd.desiredThrustsN[1] = (-f1 + f2 - f3 + collThrustCmd)/4.f;
  cmd.desiredThrustsN[2] = (f1 - f2 - f3 + collThrustCmd)/4.f ; 
  cmd.desiredThrustsN[3] = (-f1 - f2 + f3 + collThrustCmd)/4.f;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes (Mx, My, Mz)

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  V3F pqr_error = pqrCmd - pqr; //(desired - current)
  // implement a proportional controller
  V3F Inertia;
  Inertia.x = Ixx;
  Inertia.y = Iyy;
  Inertia.z = Izz;
  momentCmd = Inertia * kpPQR * pqr_error;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first


  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  pqrCmd.x = 0; // p =
  pqrCmd.y = 0; // q =
  pqrCmd.z = 0; // r =
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  if (collThrustCmd >0)
  {  float accel_coll = - collThrustCmd / mass; 
  // Implementing the equations
     float b_c_x = CONSTRAIN(accelCmd.x/accel_coll, -maxTiltAngle, maxTiltAngle);
     float b_c_xp = kpBank * (b_c_x - R(0,2));

     float b_c_y = CONSTRAIN(accelCmd.y/accel_coll, -maxTiltAngle, maxTiltAngle);
     float b_c_yp = kpBank * (b_c_y - R(1,2));

     pqrCmd.x = 1 / R(2,2) * ( R(1,0) * b_c_xp - R(0,0) * b_c_yp) ;
     pqrCmd.y = 1 / R(2,2) * ( R(1,1) * b_c_xp - R(0,1) * b_c_yp) ;
  }
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float error_Z_prop = posZCmd - posZ;
  float error_Z_derived = velZCmd - velZ;
  integratedAltitudeError = integratedAltitudeError + error_Z_prop * dt;

  float kp_Z = kpPosZ * error_Z_prop;
  float kd_Z = kpVelZ * error_Z_derived + velZ;
  float ki_Z = KiPosZ * integratedAltitudeError;

  float acc_Z = kp_Z + kd_Z + ki_Z + accelZCmd;
  float acc_Z_body = (acc_Z - 9.81)/R(2,2);
  
  float acc_Z_body_inrange = CONSTRAIN(acc_Z_body, -maxAscentRate/dt, maxAscentRate/dt);
  thrust = mass * acc_Z_body_inrange;
  thrust = thrust * (-1);
 
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;
  accelCmd.z = 0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////


  velCmd[0] = CONSTRAIN(velCmd[0], -maxSpeedXY,maxSpeedXY);
  velCmd[1] = CONSTRAIN(velCmd[1], -maxSpeedXY,maxSpeedXY);

  V3F error_pos = posCmd - pos;
  V3F error_vel = velCmd - vel;
  V3F kp_pos = kpPosXY*error_pos;
  V3F kp_vel = kpPosXY*error_vel;
  accelCmd = kp_pos + kp_vel + accelCmdFF;

  accelCmd[0] = CONSTRAIN(accelCmd[0], -maxAccelXY,maxAccelXY);
  accelCmd[1] = CONSTRAIN(accelCmd[1], -maxAccelXY,maxAccelXY);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw
  
  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // fmodf computes the floating point operator of the division operation
  // therefore it converts radian to float


  float yawCmd_float= 0;
  if (yawCmd > 0 )
    {yawCmd_float =fmodf(yawCmd,2*3.14159);}
  else
    {yawCmd_float = fmodf(yawCmd,2*3.14159) * (-1);   } 
 
  //cout << "F_PI";
  //cout << F_PI;

  float error_yaw = yawCmd - yaw;

  if (error_yaw > 3.14159)
    error_yaw = error_yaw - 2 * 3.14159;
  else if (error_yaw < -3.14159)
    error_yaw = error_yaw + 2 * 3.14159;

  yawRateCmd = error_yaw * error_yaw;


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
