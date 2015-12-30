'''
fmu.py - Flight Management Unit class

    Copyright (C) 2014 Bipeen Acharya, Fred Gisa, and Simon D. Levy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

'''

# PID parameters ==================================================================

IMU_PITCH_ROLL_Kp  = .25
IMU_PITCH_ROLL_Kd  = 0.1

IMU_YAW_Kp 	       = 2.0
IMU_YAW_Kd 	       = 1.0 
IMU_YAW_Ki         = 0.1


# We don't need K_d because we use first derivative
ALTITUDE_Kp             = 10

# Empirical constants  ============================================================

# Flight Forces
ROLL_DEMAND_FACTOR      = 0.1
PITCH_DEMAND_FACTOR     = 0.1
YAW_DEMAND_FACTOR       = 0.5
THRUST_HOVER          = 5.4580394

# Essential imports ================================================================

from pidcontrol import Stability_PID_Controller, Yaw_PID_Controller
from math import sqrt, pi

# Helpers ==========================================================================

def safe(value):

    return 0 if abs(value) > 1 else value

def angleNegate(theta):

    return theta + pi if theta < 0 else theta - pi

# Quadrotor class ==================================================================

class FMU(object):

    def __init__(self, logfile=None):
        '''
        Creates a new Quadrotor object with optional logfile.
        '''

        # Store logfile handle
        self.logfile = logfile

        # Create PD controllers for pitch, roll based on angles from Inertial Measurement Unit (IMU)
        self.pitch_Stability_PID = Stability_PID_Controller(IMU_PITCH_ROLL_Kp, IMU_PITCH_ROLL_Kd)      
        self.roll_Stability_PID  = Stability_PID_Controller(IMU_PITCH_ROLL_Kp, IMU_PITCH_ROLL_Kd)

        # Special handling for yaw from IMU
        self.yaw_IMU_PID   = Yaw_PID_Controller(IMU_YAW_Kp, IMU_YAW_Kd, IMU_YAW_Ki)

        # Create PD controller for altitude-hold
        #self.altitude_PID = Hover_PID_Controller(ALTITUDE_Kp)

    def getMotors(self, imuAngles, controllerInput, timestep, extraData):
        '''
        Gets motor thrusts based on current telemetry:

            imuAngles         IMU pitch, roll, yaw angles in radians (positive = nose up, right down, nose right)
            controllerInput   (pitchDemand, rollDemand, yawDemand, throttleDemand, switch) 
            timestep          timestep in seconds
            extraData    extra data for mission
        '''
        # Convert flight-stick controller input to demands
        pitchDemand = controllerInput[0] * PITCH_DEMAND_FACTOR
        rollDemand  = controllerInput[1] * ROLL_DEMAND_FACTOR
        yawDemand   = -controllerInput[2] * YAW_DEMAND_FACTOR
        throttleDemand = 4*sqrt(sqrt(controllerInput[3])) + 2

        # Grab value of three-position switch
        #switch = controllerInput[4]

        # PID control for pitch, roll based on angles from Inertial Measurement Unit (IMU)
        imuPitchCorrection = self.pitch_Stability_PID.getCorrection(imuAngles[0], timestep)      
        imuRollCorrection  = self.roll_Stability_PID.getCorrection(-imuAngles[1], timestep)

        # Overall pitch, roll correction is sum of stability and hover-in-place
        pitchCorrection = imuPitchCorrection
        rollCorrection  = imuRollCorrection

        # Normal Flight Parameters
        yawCorrection = self.yaw_IMU_PID.getCorrection(imuAngles[2], yawDemand, timestep)
        thrust = throttleDemand

        # Overall thrust is baseline plus throttle demand plus correction from PD controller
        # received from the joystick and the # quadrotor model corrections. A positive 
        # pitch value means, thrust increases for two back propellers and a negative 
        # is opposite; similarly for roll and yaw.  A positive throttle value means thrust 
        # increases for all 4 propellers.
        psign = [+1, -1, -1, +1]    
        rsign = [-1, -1, +1, +1]
        ysign = [+1, -1, +1, -1]

        thrusts = [0]*4

        for i in range(4):
            thrusts[i] = (thrust + rsign[i]*rollDemand + psign[i]*pitchDemand + ysign[i]*yawDemand) * \
                    (1 + rsign[i]*rollCorrection + psign[i]*pitchCorrection + ysign[i]*yawCorrection) 

        return thrusts
