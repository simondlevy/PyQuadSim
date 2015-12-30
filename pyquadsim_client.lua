--[[
pyquadsim_client.lua - Lua code for the main V-REP child script in PyQuadSim

    Sends simulation variable values to pyquadsim_server.py script and retrieves
    propeller force and torque.

    Modified from the corresponding script in V-REP's quadricopter model

    Copyright (C) 2014 Bipeen Acharya, Fred Gisa, and Simon D. Levy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.


'--]]

-- Modify PYQUADSIM_HOME variable in script from .ttt
SERVER_EXECUTABLE = PYQUADSIM_HOME..'/pyquadsim_server.py'

-- Helpers =====================================================================

function sendFloats(server, data)
    server:send(simPackFloats(data))
end

function receiveFloats(server, count)
    data = server:receive(4*count)
    return simUnpackFloats(data)
end

function sendString(server, str)
    sendFloats(server, {string.len(str)})
    server:send(str)
end

function scalarTo3D(s, a)
    return {s*a[3], s*a[7], s*a[11]}
end

function rotate(x, y, theta)

   return {math.cos(theta)*x + math.sin(theta)*y, -math.sin(theta)*x + math.cos(theta)*y}
end

-- Initialization ==============================================================

if (sim_call_type==sim_childscriptcall_initialization) then 

    -- We wanna manually switch for synchronization purpose (and also not to waste processing time!)
    simSetThreadSwitchTiming(200) 

    -- We start the server on a port that is probably not used (try to always use a similar code):
    simSetThreadAutomaticSwitch(false)
    local portNb = simGetIntegerParameter(sim_intparam_server_port_next)
    local portStart = simGetIntegerParameter(sim_intparam_server_port_start)
    local portRange = simGetIntegerParameter(sim_intparam_server_port_range)
    local newPortNb = portNb+1
    if (newPortNb>=portStart+portRange) then
        newPortNb=portStart
    end

    simSetIntegerParameter(sim_intparam_server_port_next,newPortNb)
    simSetThreadAutomaticSwitch(true)

    -- Set the last argument to 1 to see the console of the launched server
    serverResult = simLaunchExecutable(SERVER_EXECUTABLE,portNb,0) 

    -- Attempt to launch the executable server script
    if (serverResult==-1) then
        simDisplayDialog('Error',
                          'Server '..SERVER_EXECUTABLE..' could not be launched. &&n'..
                          'Please make sure that it exists and is executable. Then stop and restart the simulation.',
                          sim_dlgstyle_message,false, nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        simPauseSimulation()
    end

    -- On success, attempt to connect to server
    while (serverResult ~= -1 and simGetSimulationState()~=sim_simulation_advancing_abouttostop) do

        -- The executable could be launched. Now build a socket and connect to the server:
        local socket=require("socket")
        server = socket.tcp()
        if server:connect('127.0.0.1',portNb) == 1 then
            break
        end

    end

    base = simGetObjectHandle('Quadricopter_base')

    propellerList = {}
    propellerRespondableList = {}

    -- Get the object handles for the propellers and respondables
    for i = 1, 4, 1 do
        propellerList[i]=simGetObjectHandle('Quadricopter_propeller'..i)
        propellerRespondableList[i]=simGetObjectHandle('Quadricopter_propeller_respondable'..i)

    end

    -- Get the particle behavior that the server needs to compute force and torque for each propeller
    particleCountPerSecond = simGetScriptSimulationParameter(sim_handle_self,'particleCountPerSecond')
    particleDensity = simGetScriptSimulationParameter(sim_handle_self,'particleDensity')

    baseParticleSize = simGetScriptSimulationParameter(sim_handle_self,'particleSize')
    timestep = simGetSimulationTimeStep()
    
        -- Compute particle sizes
    particleSizes = {}
    for i = 1, 4, 1 do
        propellerSizeFactor = simGetObjectSizeFactor(propellerList[i]) 
        particleSizes[i] = baseParticleSize*0.005*propellerSizeFactor
    end

    particleCount = math.floor(particleCountPerSecond * timestep)

    -- Send initialization data to server
    sendString(server, PYQUADSIM_HOME)

    -- Do any additional initialization
    dofile(PYQUADSIM_HOME..'pyquadsim_client_init.lua')

end -- Initialization 


-- Looping code =================================================================================
if (sim_call_type==sim_childscriptcall_actuation) then 

    -- Now don't waste time in this loop if the simulation time hasn't changed! 
    -- This also synchronizes this thread with the main script
    -- This thread will resume just before the main script is called again
    simSwitchThread() 

    -- Get Euler angles for IMU
    orientation = simGetObjectOrientation(base, -1)

    -- Convert Euler angles to pitch, roll, yaw
    -- See http://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft) for positive/negative orientation
    alpha = orientation[1]
    beta = orientation[2]
    gamma = orientation[3]
    rollpitch = rotate(alpha, beta, gamma)

     -- Send core data to server as pitch 
     -- (positive = nose up), roll (positive = right down), yaw (positive = nose right)
    sendFloats(server, {timestep, -rollpitch[2], rollpitch[1], -gamma})

    -- Do any additional sending to server
    dofile(PYQUADSIM_HOME..'pyquadsim_client_loop.lua')

    -- Receive thrust values from server
    thrusts = receiveFloats(server, 4)

    if thrusts == nil then

        simStopSimulation()

    else 

        -- Set forces and torques for each propeller
        for i = 1, 4, 1 do

            thrust = thrusts[i]

            force = particleCount* particleDensity * thrust * math.pi * math.pow(particleSizes[i],3) / (6*timestep)
            torque = math.pow(-1, i+1)*.002 * thrust

            -- Set float signals to the respective propellers, and propeller respondables
            simSetFloatSignal('Quadricopter_propeller_respondable'..i, propellerRespondableList[i])

            propellerMatrix = simGetObjectMatrix(propellerList[i],-1)

            forces = scalarTo3D(force,  propellerMatrix)
            torques = scalarTo3D(torque, propellerMatrix)

            -- Set force and torque for propeller
            for k = 1, 3, 1 do
                simSetFloatSignal('force'..i..k,  forces[k])
                simSetFloatSignal('torque'..i..k, torques[k])
            end

        end

    end
end
