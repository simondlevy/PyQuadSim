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

#from quadstick import ExtremePro3D as Controller
from quadstick import PS3 as Controller
#from quadstick.rc.spektrum import DX8 as Controller
#from quadstick.rc.frsky import Taranis as Controller
#from quadstick.keyboard import Keyboard as Controller

# Mission-specific data ===========================================================

from pyquadsim_server_extra import getAdditionalData

# Simulation parameters ===========================================================

# Timeout for receiving data from client
TIMEOUT_SEC      = 1.0

# Other imports ===================================================================

from sys import argv, exit
from math import pi
import struct
import time
import random
import os

from socket_server import serve_socket
from fmu import FMU

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
    
# LogFile class ======================================================================================================

class LogFile(object):

    def __init__(self, directory):
 
        self.fd = open(directory + '/' + time.strftime('%d_%b_%Y_%H_%M_%S') + '.csv', 'w')

    def writeln(self, string):

        self.fd.write(string + '\n')
        self.fd.flush()

    def close(self):

        self.fd.close()

# Initialization =====================================================================================================

# Require controller
controller = Controller(('Stabilize', 'Hold Altitude', 'Unused'))

# Serve a socket on the port indicated in the first command-line argument
client = serve_socket(int(argv[1]))

# Receive working directory path from client
pyquadsim_directory = receiveString(client)

# Create logs folder if needed
logdir = pyquadsim_directory + '/logs'
if not os.path.exists(logdir):
    os.mkdir(logdir)

# Open logfile named by current date, time
logfile = LogFile(logdir)

# Create an FMU object, passing it the logfile object in case it needs to write to the logfile.
fmu = FMU(logfile)

# Loop ==========================================================================================================

# Forever loop will be halted by VREP client or by exception
while True:

    try:

        # Get core data from client
        coreData = receiveFloats(client, 4)

        # Quit on timeout
        if not coreData: exit(0)

        # Get extra data from client
        extraData = getAdditionalData(client, receiveFloats)

        # Unpack IMU data        
        timestep = coreData[0]  # seconds
        pitch    = coreData[1]  # positive = nose up
        roll     = coreData[2]  # positive = right down
        yaw      = coreData[3]  # positive = nose right

        # Poll controller
        demands = controller.poll()

        # Get motor thrusts from quadrotor model
        thrusts = fmu.getMotors((pitch, roll, yaw), demands,  timestep, extraData)

        # Send thrusts to client
        sendFloats(client, thrusts)

    except Exception:

        # Inform and exit on exception
        controller.error()
        exit(0)
