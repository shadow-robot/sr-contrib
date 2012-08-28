#!/usr/bin/env python
# fake controller manager, only multiplexes several controller_manager services

import roslib; roslib.load_manifest('sr_utilities'); roslib.load_manifest('pr2_mechanism_msgs')
import rospy
import roslaunch
#from pr2_mechanism_msgs.srv import ListControllers,ListControllersResponse,SwitchController,SwitchControllerResponse
from pr2_mechanism_msgs.srv import *
import sys

class SrControllerManager:
    def __init__(self, names=[]):

        if( names == []):
            self.controller_manager_names = ['pr2_controller_manager']
        else:
            self.controller_manager_names = names

        #print self.controller_manager_names

        rospy.Service('/sr_controller_manager/list_controllers', ListControllers, self.list_controllers)
        rospy.Service('/sr_controller_manager/switch_controller',SwitchController,self.switch_controller)
        rospy.Service('/sr_controller_manager/load_controller',LoadController,self.load_controller)
        rospy.Service('/sr_controller_manager/unload_controller',UnloadController,self.unload_controller)

        rospy.init_node("sr_controller_manager")

        self.create_mapping()
        #print self.Controller2ManagerMapping
        rospy.spin()

    def create_mapping(self):
        # # associate controller to their manager
        self.Controller2ManagerMapping={}
        self.list_controllers_name_cache=[];
        self.list_controllers_state_cache=[];
        for cm_name in self.controller_manager_names:
            try:
                rospy.wait_for_service(cm_name + "/list_controllers")
                get_controllers_list=rospy.ServiceProxy(cm_name+"/list_controllers", ListControllers)
                controllers_list=get_controllers_list()
                for controller,state in zip(controllers_list.controllers,controllers_list.state):
                    #check if controller does not appear twice
                    try:
                        controller_name = self.Controller2ManagerMapping[controller]
                        self.list_controllers_name_cache.append(controller)
                        self.list_controllers_state_cache.append(state)
                    except:
                        self.Controller2ManagerMapping[controller]=cm_name
                        self.list_controllers_name_cache.append(controller)
                        self.list_controllers_state_cache.append(state)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    def list_controllers(self,req):
        myresponse=ListControllersResponse()
        self.create_mapping()
        #suppose the cache exists retrieve from cache
        for controller,state in zip(self.list_controllers_name_cache,self.list_controllers_state_cache):
            myresponse.controllers.append(controller)
            myresponse.state.append(state)
        return myresponse

    def load_controller(self,req):
        myresponse=LoadControllerResponse()
        for cm_name in self.controller_manager_names:
            try:
                load_controller_srv=rospy.ServiceProxy(cm_name+"/load_controller", LoadController)
                myresponse=load_controller_srv(req)
            except:
                print "Could not find service"
            if(myresponse.ok==True): # the current cm managed to load the controller
                break

        #controllers have changed, reload the mapping and cache
        self.create_mapping()
        return myresponse

    def unload_controller(self,req):
        myresponse=UnloadControllerResponse()
        try:
            cm_name=self.Controller2ManagerMapping[req.name]
            unload_controller_srv=rospy.ServiceProxy(cm_name+"/unload_controller", UnloadController)
            myresponse=unload_controller_srv(req)
        except:
            print "Could not find controller in cache"
            myresponse.ok=False;
            return myresponse
        # the current cm managed to load the controller
        if(myresponse.ok==True):
            self.create_mapping()
        return myresponse


    def switch_controller(self,req):
        myresponse=SwitchControllerResponse()
        # prepare request for each controller
        myrequest={}
        for cm_name in self.controller_manager_names:
            myrequest[cm_name]=SwitchControllerRequest()
        # separate controllers in the correct request
        for controller_to_start in req.start_controllers:
#           print "controller to start:",controller_to_start
            try:
                my_cm_name=self.Controller2ManagerMapping[controller_to_start]
#               print "found cm for this controller",my_cm_name
                myrequest[my_cm_name].start_controllers.append(controller_to_start)
            except:
                print "Could not find controller in mapping"
                myresponse.ok=False
                return myresponse
        for controller_to_stop in req.stop_controllers:
#           print "controller to stop:",controller_to_stop
            try:
                my_cm_name=self.Controller2ManagerMapping[controller_to_stop]
#               print "found cm for this controller",my_cm_name
                myrequest[my_cm_name].stop_controllers.append(controller_to_stop)
            except:
                print "Could not find controller in mapping"
                myresponse.ok=False
                return myresponse


#       print "pr2",myrequest["pr2_controller_manager"]
#       print "arm",myrequest["sr_arm_controller_manager"]

        for cm_name in self.controller_manager_names:
            try:
                switch_srv=rospy.ServiceProxy(cm_name+"/switch_controller", SwitchController)
                myrequest[cm_name].strictness=2
                myresponse=switch_srv(myrequest[cm_name])
            except:
                print "Could not find switch_controller service for ",cm_name
                myresponse.ok=False
                return myresponse
        # the current cm managed to load the controller
        if(myresponse.ok==True):
            self.create_mapping()

        return myresponse

if __name__ == '__main__':
    cm_names=[]
    args = rospy.myargv()
    if len(args) > 1:
        for i in range(len(args)-1):
            cm_names.append(args[i+1])
    SCM = SrControllerManager(cm_names)

