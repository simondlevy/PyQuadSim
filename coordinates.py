#!/usr/bin/env python

'''
coordinates.py - Latitude/longitude to meters calculator

    Copyright (C) 2014 Simon D. Levy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

Revision history:

29-AUG-2014 | Simon D. Levy | Initial release
'''

from math import radians, cos
import re

class CoordinateCalculator(object):

    def __init__(self, coords=None):
     
      if coords:

          self.lat_deg = coords[0]
          self.long_deg = coords[1]

      else:

          try:

              import requests

              html = requests.get('http://www.geoiptool.com/').text

              self.lat_deg = self._coord_from_html(html, 'Latitude')
              self.long_deg = self._coord_from_html(html, 'Longitude')

          except:
 
              print('No GPS available; using coordinates (0,0)')

              self.lat_deg = 0
              self.long_deg = 0

      lat_rad = radians(self.lat_deg)

      # http://fmepedia.safe.com/articles/How_To/Calculating-accurate-length-in-meters-for-lat-long-coordinate-systems
      self.lat_meters_per_deg  = 111132.92 - 559.82 * cos(2*lat_rad) + 1.175*cos(4*lat_rad)
      self.long_meters_per_deg = 111412.84 * cos(lat_rad) - 93.5 * cos(3*lat_rad)

    def __str__(self):

        return '%6.4f %6.4f' % (self.lat_deg, self.long_deg)
      
    def metersToDegrees(self, lat_offset_meters, long_offset_meters):

        return self.lat_deg  + lat_offset_meters/self.lat_meters_per_deg, \
               self.long_deg + long_offset_meters/self.long_meters_per_deg

    def degreesToMeters(self, lat_degrees, long_degrees):

        return lat_degrees *  self.lat_meters_per_deg, \
               long_degrees * self.long_meters_per_deg

    def _coord_from_html(self, html, tag):

        return float(re.findall("-?\d+.\d+", html[html.find(tag):])[0])

# test =========================================================================
    
if __name__ == "__main__":

   
  print(CoordinateCalculator())
