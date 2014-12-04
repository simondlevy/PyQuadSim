#!/usr/bin/env python

'''
pyquadsim_server.py - Automatically-launched Python server script for PyQuadSim

Translates simulation values from V-REP to sensor values for quadrotor model

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

# Import your controller here =====================================================

from quadstick.axial.game.logitech import ExtremePro3D as Controller
#from quadstick.axial.game.sony import PS3 as Controller
#from quadstick.axial.rc.spektrum import DX8 as Controller
#from quadstick.axial.rc.frsky import TH9X as Controller
#from quadstick.keyboard import Keyboard as Controller

# Simulation parameters ===========================================================

# XXX Unrealistic GPS simulation (perfect accuracy
GPS_NOISE_METERS = 0

# Timeout for receiving data from client
TIMEOUT_SEC      = 1.0

# Other imports ===================================================================

from sys import argv, exit
from math import pi
import struct
import time
import random

from socket_server import serve_socket
from quadrotor import Quadrotor
from coordinates import CoordinateCalculator
from geometry import rotate

# Helper functions ================================================================

def sendFloats(client, data):

    client.send(struct.pack('%sf' % len(data), *data))

def unpackFloats(msg, nfloats):

    return struct.unpack('f'*nfloats, msg)

def receiveFloats(client, nfloats):
 
    # We use 32-bit floats
    msgsize = 4 * nfloats

    # Implement timeout
    start_sec = time.time()
    remaining = msgsize
    msg = ''
    while remaining > 0:
        msg += client.recv(remaining)
        remaining -= len(msg)
        if (time.time()-start_sec) > TIMEOUT_SEC:
            return None

    return unpackFloats(msg, nfloats)
    
def receiveString(client):
    
    return client.recv(int(receiveFloats(client, 1)[0]))

def scalarTo3D(s, a):

    return [s*a[2], s*a[6], s*a[10]]
    
# LogFile class ==========================================================================================================

class LogFile(object):

    def __init__(self, directory):
 
        self.fd = open(directory + '/' + time.strftime('%d_%b_%Y_%I_%M_%S') + '.log', 'w')

    def writeln(self, string):

        self.fd.write(string + '\n')
        self.fd.flush()

    def close(self):

        self.fd.close()

# Initialization ==========================================================================================================

# Require controller
controller = Controller()

# Serve a socket on the port indicated in the first command-line argument
client = serve_socket(int(argv[1]))

# Receive working directory path from client
pyquadsim_directory = receiveString(client)

# Receive particle info from client
particleInfo = receiveFloats(client, 6)
particleSizes = particleInfo[0:4]
particleDensity = particleInfo[4]
particleCountPerSecond = particleInfo[5]

# Open logfile named by current date, time
logfile = LogFile(pyquadsim_directory + '/logs') 

# Create a quadrotor object for  pitch, roll, yaw, altitude correction.  
# Pass it the logfile object in case it needs to write to the logfile.
quad = Quadrotor(logfile)

# Create coordinate calculator for GPS simulation
coordcalc = CoordinateCalculator()

# Loop ==========================================================================================================

# Forever loop will be halted by VREP client or by exception
while True:

    try:

        # Get core data from client
        clientData = receiveFloats(client, 55)

        if not clientData:
            break

        # Quit on timeout
        if not clientData: exit(0)

        # Unpack IMU data        
        timestepSeconds = clientData[0]
        positionXMeters = clientData[1]
        positionYMeters = clientData[2]
        positionZMeters = clientData[3]
        alphaRadians    = clientData[4]
        betaRadians     = clientData[5]
        gammaRadians    = clientData[6]    

        # Unpack propeller matrices
        propellerMatrices = [[0]*12 for i in range(4)]
        propellerMatrices[0] = clientData[7 :19]
        propellerMatrices[1] = clientData[19:31]
        propellerMatrices[2] = clientData[31:43]
        propellerMatrices[3] = clientData[43:55]

        # Add some Guassian noise to position
        positionXMeters = random.gauss(positionXMeters, GPS_NOISE_METERS)
        positionYMeters = random.gauss(positionYMeters, GPS_NOISE_METERS)
            
        # Convert Euler angles to pitch, roll, yaw
        # See http://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft) for positive/negative orientation
        rollAngleRadians, pitchAngleRadians = rotate((alphaRadians, betaRadians), gammaRadians)
        pitchAngleRadians = -pitchAngleRadians
        yawAngleRadians   = -gammaRadians

        # Get altitude directly from position Z
        altitudeMeters = positionZMeters

        # Poll controller
        demands = controller.poll()

        # Get motor thrusts from quadrotor model
        thrusts = quad.getMotors((pitchAngleRadians, rollAngleRadians, yawAngleRadians), altitudeMeters, \
                                  coordcalc.metersToDegrees(positionXMeters, positionYMeters),\
                                  demands,  timestepSeconds)

        # Force is a function of particle count
        particleCount = int(particleCountPerSecond * timestepSeconds)
     
        # Compute force and torque for each propeller
        forcesAndTorques = [0]*24
        for i in range(4):

            force  = particleCount * particleDensity * thrusts[i] * pi * particleSizes[i]**3 / (6*timestepSeconds)
            torque = ((-1)**(i+1))*.002 * thrusts[i]

            # Convert forces and torques to 3D
            j = i * 6
            forcesAndTorques[j:j+3]   = scalarTo3D(force,  propellerMatrices[i])
            forcesAndTorques[j+3:j+6] = scalarTo3D(torque, propellerMatrices[i])

        # Send forces and torques to client
        sendFloats(client, forcesAndTorques)

    except Exception:

        # Inform and exit on exception
        controller.error()
        exit(0)
