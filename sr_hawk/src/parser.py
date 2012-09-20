#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import roslib; roslib.load_manifest('sr_hawk')
import rospy
import roslib

class HAWKParser(object):
    """
    Parses the csv which is the output of the HAWK system.
    """

    #this is the index of the line we read last
    current_index = -1
    def __init__(self, path = None):
        """
        """
        data_path = path
        if data_path == None:
            #use default path
            try:
                data_path = roslib.packages.get_pkg_dir("sr_hawk") + "/data/shadow.csv"
            except:
                rospy.logwarn("Couldnt find the sr_hawk package")

        f = open(data_path, 'r')
        data = f.readlines()
        f.close()

        self.hawk_data = []
        for line in data:
            split = line.split(",")
            line = []
            for datum in split:
                try:
                    line.append(float(datum.strip()))
                except:
                    #ignore the text
                    pass

            if len(line) != 0:
                self.hawk_data.append(line)

    def next_data(self):
        """
        Returns the next line of data (based on current index)
        """
        self.current_index += 1
        if self.current_index == len(self.hawk_data):
            self.current_index = 0

        return self.hawk_data[ self.current_index ]

