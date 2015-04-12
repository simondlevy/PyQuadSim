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

# PID parameters (I is currently unused) ==========================================

IMU_PITCH_ROLL_Kp       = .25
IMU_PITCH_ROLL_Kd       = 0.1

IMU_YAW_Kp 	            = 1.0
IMU_YAW_Kd 	            = 0.4

# We don't need K_d because we use first derivative
ALTITUDE_Kp             = 10

# This is large because it is based on decimal degrees
GPS_PITCH_ROLL_Kp       = 5000

# Empirical constants  ============================================================

THRUST_BASELINE 	    = 5.335
ROLL_DEMAND_FACTOR      = 0.1
PITCH_DEMAND_FACTOR     = 0.1
YAW_DEMAND_FACTOR       = 0.5
CLIMB_DEMAND_FACTOR     = 0.5

# Imports =========================================================================

from pidcontrol import Stability_PID_Controller, Yaw_PID_Controller, Hover_PID_Controller
from geometry import rotate

import math

# FMU class ==================================================================

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
        self.yaw_IMU_PID   = Yaw_PID_Controller(IMU_YAW_Kp, IMU_YAW_Kd)

        # Create PD controllers for pitch, roll based on GPS groundspeed (position-hold; hover-in-place)
        self.latitude_PID   = Hover_PID_Controller(GPS_PITCH_ROLL_Kp)
        self.longitude_PID  = Hover_PID_Controller(GPS_PITCH_ROLL_Kp)

        # Create PD controller for altitude-hold
        self.altitude_PID = Hover_PID_Controller(ALTITUDE_Kp)

    def getMotors(self, imuAngles, altitude, gpsCoords, controllerInput, timestep):
        '''
        Gets motor thrusts based on current telemetry:

            imuAngles      IMU pitch, roll, yaw angles in radians
            altitude       altitude in meters
            gpsCoords      GPS coordinates (latitude, longitude) in degrees
            controllInput  (pitchDemand, rollDemand, yawDemand, climbDemand) in interval [-1,+1]
                           (altitudeHold, positionHold, autopilot) flags
            timestep       timestep in seconds
        '''

        # Convert flight-stick stickDemands
        stickDemands = controllerInput[0]
        pitchDemand = stickDemands[0] * PITCH_DEMAND_FACTOR
        rollDemand  = stickDemands[1] * ROLL_DEMAND_FACTOR
        yawDemand   = stickDemands[2] * YAW_DEMAND_FACTOR
        climbDemand = stickDemands[3] * CLIMB_DEMAND_FACTOR

        # Combine pitch and roll demand into one value for PID control
        pitchrollDemand = math.sqrt(pitchDemand**2 + rollDemand**2)
        gpsLatCorrection  = self.latitude_PID.getCorrection(gpsCoords[0], pitchrollDemand, timestep=timestep)
        gpsLongCorrection = self.longitude_PID.getCorrection(gpsCoords[1], pitchrollDemand, timestep=timestep)

        # Compute GPS-based pitch, roll correction if we want position-hold
        stickFlags = controllerInput[1]
        gpsPitchCorrection, gpsRollCorrection = 0,0
        if stickFlags[1]:
            gpsPitchCorrection, gpsRollCorrection = rotate((gpsLatCorrection, gpsLongCorrection), -imuAngles[2])
            gpsPitchCorrection = -gpsPitchCorrection

        # Compute altitude hold if we want it
        altitudeHold = 0
        if stickFlags[0]:
            altitudeHold = self.altitude_PID.getCorrection(altitude, climbDemand, timestep=timestep)

        # PID control for pitch, roll based on angles from Inertial Measurement Unit (IMU)
        imuPitchCorrection = self.pitch_Stability_PID.getCorrection(imuAngles[0], timestep)      
        imuRollCorrection  = self.roll_Stability_PID.getCorrection(-imuAngles[1], timestep)

        # Simple autopilot demo: yaw 
        if stickFlags[2]:
            yawDemand = 0.05

        # Special PID for yaw
        yawCorrection   = self.yaw_IMU_PID.getCorrection(imuAngles[2], yawDemand, timestep)
              
        # Overall pitch, roll correction is sum of stability and position-hold 
        pitchCorrection = imuPitchCorrection + gpsPitchCorrection
        rollCorrection  = imuRollCorrection  + gpsRollCorrection
        
        # Overall thrust is baseline plus climb demand plus correction from PD controller
        thrust = THRUST_BASELINE + climbDemand + altitudeHold

        # Change the thrust values depending upon the pitch, roll, yaw and climb values 
        # received from the joystick and the # quadrotor model corrections. A positive 
        # pitch value means, thrust increases for two back propellers and a negative 
        # is opposite; similarly for roll and yaw.  A positive climb value means thrust 
        # increases for all 4 propellers.

        psign = [+1, -1, -1, +1]    
        rsign = [-1, -1, +1, +1]
        ysign = [+1, -1, +1, -1]

        thrusts = [0]*4
 
        for i in range(4):

            thrusts[i] = (thrust + rsign[i]*rollDemand +     psign[i]*pitchDemand +      ysign[i]*yawDemand) * \
                         (1 +      rsign[i]*rollCorrection + psign[i]*pitchCorrection +  ysign[i]*yawCorrection) 

        return thrusts
