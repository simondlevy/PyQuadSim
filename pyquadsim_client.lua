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

-- Initialization ==============================================================

if (simGetScriptExecutionCount()==0) then

    simSetThreadSwitchTiming(200) -- We wanna manually switch for synchronization purpose (and also not to waste processing time!)

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
    result=simLaunchExecutable(SERVER_EXECUTABLE,portNb,0) 

    -- Attempt to launch the executable server script
    if (serverResult==-1) then
        simDisplayDialog('Error',"Server "..SERVER_EXECUTABLE.." could not be launched. &&nSimulation will not run properly",
                      sim_dlgstyle_ok,true,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
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
    
    particleSizes = {}
    
    -- Compute particle sizes
    for i = 1, 4, 1 do
        propellerSizeFactor = simGetObjectSizeFactor(propellerList[i]) 
        particleSizes[i] = baseParticleSize*0.005*propellerSizeFactor
    end

    -- Send initialization data to server
    sendString(server, PYQUADSIM_HOME)
    sendFloats(server, {particleSizes[1], particleSizes[2], particleSizes[3], particleSizes[4], 
                        particleDensity, particleCountPerSecond})

end -- Initialization 


-- Looping code =================================================================================

-- Now don't waste time in this loop if the simulation time hasn't changed! 
-- This also synchronizes this thread with the main script
-- This thread will resume just before the main script is called again
simSwitchThread() 

-- Get position
position = simGetObjectPosition(base, -1)

-- Get Euler angles for IMU
orientation = simGetObjectOrientation(base, -1)

-- Build core data from timestep and angles
coreData = { timestep, position[1], position[2], position[3], orientation[1], orientation[2], orientation[3] } 

 -- Add all propeller matrices to core data
for i = 1, 4, 1 do
    propellerMatrix = simGetObjectMatrix(propellerList[i],-1)
    for j = 1, 12, 1 do
        table.insert(coreData, propellerMatrix[j])
    end
end

-- Send core data to server
sendFloats(server, coreData)

-- Receive 3D forces and torques from server
forcesAndTorques = receiveFloats(server, 24)

if forcesAndTorques == nil then

    simStopSimulation()

else 

    -- Set forces and torques for each propeller
    for i = 1, 4, 1 do

        -- Set float signals to the respective propellers, and propeller respondables
        simSetFloatSignal('Quadricopter_propeller_respondable'..i, propellerRespondableList[i])

        -- Set force and torque for propeller
        j = (i-1) * 6
        for k = 1, 3, 1 do
            simSetFloatSignal('force'..i..k,  forcesAndTorques[j+k])
            simSetFloatSignal('torque'..i..k, forcesAndTorques[j+k+3])
        end
        
    end

    simHandleChildScript(sim_handle_all_except_explicit)

end
