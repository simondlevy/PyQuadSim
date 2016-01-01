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

IMU_PITCH_ROLL_Kp  = .1
IMU_PITCH_ROLL_Kd  = .025
IMU_PITCH_ROLL_Ki  = 0

IMU_YAW_Kp 	       = 0.2
IMU_YAW_Kd 	       = 0.1 
IMU_YAW_Ki         = 0.01

ALTITUDE_Kp        = 0.25
ALTITUDE_Kd        = 0.1

# Flight Forces
ROLL_DEMAND_FACTOR      = 0.1
PITCH_DEMAND_FACTOR     = 0.1
YAW_DEMAND_FACTOR       = 0.5

# Essential imports ================================================================

from pidcontrol import Stability_PID_Controller, Yaw_PID_Controller, Hover_PID_Controller
import math

# FMU class ========================================================================

class FMU(object):

    def __init__(self, logfile=None):
        '''
        Creates a new Quadrotor object with optional logfile.
        '''

        # Store logfile handle
        self.logfile = logfile

        # Create PD controllers for pitch, roll based on angles from Inertial Measurement Unit (IMU)
        self.pitch_Stability_PID = Stability_PID_Controller(IMU_PITCH_ROLL_Kp, IMU_PITCH_ROLL_Kd, IMU_PITCH_ROLL_Ki)      
        self.roll_Stability_PID  = Stability_PID_Controller(IMU_PITCH_ROLL_Kp, IMU_PITCH_ROLL_Kd, IMU_PITCH_ROLL_Ki)

        # Special handling for yaw from IMU
        self.yaw_IMU_PID   = Yaw_PID_Controller(IMU_YAW_Kp, IMU_YAW_Kd, IMU_YAW_Ki)

        # Create PD controller for altitude-hold
        self.altitude_PID = Hover_PID_Controller(ALTITUDE_Kp, ALTITUDE_Kd)

        # For altitude hold
        self.switch_prev = 0
        self.target_altitude = 0

    def getMotors(self, imuAngles, controllerInput, timestep, extraData):
        '''
        Gets motor thrusts based on current telemetry:

            imuAngles         IMU pitch, roll, yaw angles in radians (positive = nose up, right down, nose right)
            controllerInput   (pitchDemand, rollDemand, yawDemand, throttleDemand, switch) 
            timestep          timestep in seconds
            extraData         extra sensor data for mission
        '''

        # Convert flight-stick controller input to demands
        pitchDemand = controllerInput[0]
        rollDemand  = controllerInput[1]
        yawDemand   = -controllerInput[2]
        throttleDemand = controllerInput[3] 

        # Grab value of three-position switch
        switch = controllerInput[4]

        # Grab altitude from sonar
        altitude = extraData[0]

        # Lock onto altitude when switch goes on
        if switch == 1:
            if self.switch_prev == 0:
                self.target_altitude = altitude
        else:
            self.target_altitude = 0
        self.switch_prev = switch

        # Get PID altitude correction if indicated
        altitudeCorrection = self.altitude_PID.getCorrection(altitude, self.target_altitude, timestep) \
                if self.target_altitude > 0 \
                else 0

        # PID control for pitch, roll based on angles from Inertial Measurement Unit (IMU)
        imuPitchCorrection = self.pitch_Stability_PID.getCorrection(imuAngles[0], timestep)      
        imuRollCorrection  = self.roll_Stability_PID.getCorrection(-imuAngles[1], timestep)

        # Pitch, roll, yaw correction
        pitchCorrection = imuPitchCorrection
        rollCorrection  = imuRollCorrection
        yawCorrection = self.yaw_IMU_PID.getCorrection(imuAngles[2], yawDemand, timestep)

        # Climb demand defaults to throttle demand
        climbDemand = throttleDemand

        # Adjust climb demand for altitude correction
        if altitudeCorrection != 0: 

            # In deadband, maintain altitude
            if throttleDemand > 0.4 and throttleDemand < 0.6:
                climbDemand = 0.5 + altitudeCorrection

        # Baseline thrust is a nonlinear function of climb demand
        thrust = 4*math.sqrt(math.sqrt(climbDemand)) + 2

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
            thrusts[i] = (thrust + rsign[i]*rollDemand*ROLL_DEMAND_FACTOR + \
                    psign[i]*PITCH_DEMAND_FACTOR*pitchDemand + \
                    ysign[i]*yawDemand*YAW_DEMAND_FACTOR) * \
                    (1 + rsign[i]*rollCorrection + psign[i]*pitchCorrection + ysign[i]*yawCorrection) 

        return thrusts
