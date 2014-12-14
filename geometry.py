'''
geometry.py - Computational geometry routines

    Copyright (C) 2014 Simon D. Levy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.


'''

import math

def rotate((x, y), theta):
   '''
   Returns point (X',Y') equal to (X,Y) rotated by angle theta in radians
   '''
    
   return math.cos(theta)*x + math.sin(theta)*y, -math.sin(theta)*x + math.cos(theta)*y

