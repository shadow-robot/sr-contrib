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
import rospy, math, numpy
from std_msgs.msg import Float64

from parser import HAWKParser

class HAWK(object):
    """
    Uses the parser to replay the data from the HAWK system
    """

    def __init__(self, ):
        """
        """
        #mapping matrix between the hawk system and the hand
        self.hawk_to_hand_mapping = []
        self.hand_names = [ "FFJ0","FFJ3","FFJ4",
                            "MFJ0","MFJ3","MFJ4",
                            "RFJ0","RFJ3","RFJ4",
                            "LFJ0","LFJ3","LFJ4","LFJ5",
                            "THJ1","THJ2","THJ3","THJ4","THJ5",
                            "WRJ1","WRJ2" ]

        self.hawk_names = [ "wrist_fe", "wrist_rud",
                            "palmar_arch_rad_mid", "palmar_arch_mid_uln",
                            "ff_mcp","ff_pip","ff_dip","ff_abd",
                            "mf_mcp","mf_pip","mf_dip","mf_abd",
                            "rf_mcp","rf_pip","rf_dip","rf_abd",
                            "lf_mcp","lf_pip","lf_dip",
                            "th_mcp","th_ip", "th_abd","th_rot" ]

        self.init_mapping_()

        self.parser = HAWKParser()
        self.publishers = []
        self.init_pub_()

    def next_step(self, timer_event = None):
        """
        Compute and publishes the next set of targets for the hand
        using the hawk parser.

        Usually called from a timer at a given rate.
        """
        new_targets_hawk = self.parser.next_data()
        new_targets_hawk = numpy.array(new_targets_hawk)
        new_targets_hand = new_targets_hawk * self.hawk_to_hand_mapping

        msg = Float64()
        for target, publisher in zip(new_targets_hand.tolist()[0], self.publishers):
            msg.data = math.radians(target)
            publisher.publish(msg)

    def init_pub_(self):
        for joint in self.hand_names:
            pub_name = "/sh_"+joint.lower()+"_mixed_position_velocity_controller/command"
            self.publishers.append( rospy.Publisher(pub_name, Float64) )

    def init_mapping_(self):
        #initialize mapping with 0 matrix
        for i in self.hawk_names:
            row = []
            for j in self.hand_names:
                row.append(0.0)
            self.hawk_to_hand_mapping.append(row)

        """
        [ "wrist_fe", "wrist_rud",
        "palmar_arch_rad_mid", "palmar_arch_mid_uln",
        "ff_mcp","ff_pip","ff_dip","ff_abd",
        "mf_mcp","mf_pip","mf_dip","mf_abd",
        "rf_mcp","rf_pip","rf_dip","rf_abd",
        "lf_mcp","lf_pip","lf_dip",
        "th_mcp","th_ip","th_abd","th_rot" ]
        """
        #establish the correspondances between hawk names and hand names
        self.hawk_to_hand_mapping[ self.hawk_names.index("wrist_fe") ][ self.hand_names.index("WRJ1")] = 1.0
        self.hawk_to_hand_mapping[ self.hawk_names.index("wrist_rud") ][ self.hand_names.index("WRJ2")] = 1.0

        self.hawk_to_hand_mapping[ self.hawk_names.index("palmar_arch_rad_mid") ][ self.hand_names.index("LFJ5")] = 0.5
        self.hawk_to_hand_mapping[ self.hawk_names.index("palmar_arch_mid_uln") ][ self.hand_names.index("LFJ5")] = 0.5

        self.hawk_to_hand_mapping[ self.hawk_names.index("ff_mcp") ][ self.hand_names.index("FFJ3")] = 1.0
        self.hawk_to_hand_mapping[ self.hawk_names.index("ff_pip") ][ self.hand_names.index("FFJ0")] = 0.5
        self.hawk_to_hand_mapping[ self.hawk_names.index("ff_dip") ][ self.hand_names.index("FFJ0")] = 0.5

        self.hawk_to_hand_mapping[ self.hawk_names.index("mf_mcp") ][ self.hand_names.index("MFJ3")] = 1.0
        self.hawk_to_hand_mapping[ self.hawk_names.index("mf_pip") ][ self.hand_names.index("MFJ0")] = 0.5
        self.hawk_to_hand_mapping[ self.hawk_names.index("mf_dip") ][ self.hand_names.index("MFJ0")] = 0.5

        self.hawk_to_hand_mapping[ self.hawk_names.index("rf_mcp") ][ self.hand_names.index("RFJ3")] = 1.0
        self.hawk_to_hand_mapping[ self.hawk_names.index("rf_pip") ][ self.hand_names.index("RFJ0")] = 0.5
        self.hawk_to_hand_mapping[ self.hawk_names.index("rf_dip") ][ self.hand_names.index("RFJ0")] = 0.5

        self.hawk_to_hand_mapping[ self.hawk_names.index("lf_mcp") ][ self.hand_names.index("LFJ3")] = 1.0
        self.hawk_to_hand_mapping[ self.hawk_names.index("lf_pip") ][ self.hand_names.index("LFJ0")] = 0.5
        self.hawk_to_hand_mapping[ self.hawk_names.index("lf_dip") ][ self.hand_names.index("LFJ0")] = 0.5

        #no thj3? Not 100% sure about thumb mapping
        self.hawk_to_hand_mapping[ self.hawk_names.index("th_mcp") ][ self.hand_names.index("THJ2")] = 1.0
        self.hawk_to_hand_mapping[ self.hawk_names.index("th_ip") ][ self.hand_names.index("THJ1")] = 1.0
        self.hawk_to_hand_mapping[ self.hawk_names.index("th_abd") ][ self.hand_names.index("THJ4")] = 1.0
        self.hawk_to_hand_mapping[ self.hawk_names.index("th_rot") ][ self.hand_names.index("THJ5")] = 1.0

        #the abductions are the same as the one on the dataglove so we can't map them properly
        # we'll assume (but it's not true) that rfj4 is fixed
        self.hawk_to_hand_mapping[ self.hawk_names.index("ff_abd") ][ self.hand_names.index("FFJ4")] = 1.0
        self.hawk_to_hand_mapping[ self.hawk_names.index("mf_abd") ][ self.hand_names.index("MFJ4")] = 1.0
        self.hawk_to_hand_mapping[ self.hawk_names.index("rf_abd") ][ self.hand_names.index("LFJ4")] = -1.0

        #for debug purpose: write the mapping to /tmp/mapping.csv
        self.save_map_to_file_(self.hawk_to_hand_mapping)

        self.hawk_to_hand_mapping = numpy.matrix(self.hawk_to_hand_mapping)

    def save_map_to_file_(self, map, path="/tmp/mapping.csv"):
        mat = open(path, "w")
        rows = []
        row = ","
        for name in self.hand_names:
            row +=  name + ","
        rows.append(row)
        for i,hawk_name  in enumerate(self.hawk_names):
            row = hawk_name + ","
            for j, hand_name in enumerate(self.hand_names):
                row += str( map[i][j] ) + ","
            rows.append(row)

        for row in rows:
            mat.write(row + "\n")
        mat.close()


if __name__ == '__main__':
    rospy.init_node("hawk_to_hand")
    hawk = HAWK()

    #should be 100Hz
    rospy.Timer( rospy.Duration(0.1), hawk.next_step)
    rospy.spin()
