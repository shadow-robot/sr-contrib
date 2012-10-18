Description :
-------------
This package provides two IK solvers for the Shadow Arm

1.) One is iterative and done on the fly using KDL and can cope with any variations of the URDF. It depends on arm_kinematics package which can be found there
http://www.ros.org/wiki/arm_kinematics

or directly here
svn co https://wu-robotics.googlecode.com/svn/branches/stable/urdf_tools/arm_kinematics

We suggest you add this to your ros/electric/stacks folder 

2.) The second one is analytical but works only for one arm structure (used to compute the IK solver). The IK solver is computed with OpenRave IKFAST solver generator (version 0.8) and a .dae file to describe the arm structure. The result is packed into a library to be used by the inverse kinematics node.


Launch : 
--------

You can launch in normal (or standalone version)
* KDL standard iterative IK solver (reviewed) 
  roslaunch sr_arm_kinematics sr_arm_kinematics.launch
OR
* IKFAST analytical IK solver (unreviewed)
  roslaunch sr_arm_kinematics sr_arm_kinematicsIKFAST.launch
