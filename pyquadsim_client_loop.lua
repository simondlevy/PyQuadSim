--[[
pyquadsim_client_loop.lua - Addditional Lua looping code for the main V-REP child script in PyQuadSim

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

result, distance = simReadProximitySensor(proximitySensorHandle)
if result <= 0 then distance = -1 end
sendFloats(server, {distance})



