// NO license for the moment
// majority of this code comes from package urdf_tool/arm_kinematics
// written by David Lu!!
// modified by Walck Guillaume
// the IK comes from IKFAST (OpenRave), the FK is using KDL currently but can be faster using fk from IK6D

#include <cstring>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/KinematicSolverInfo.h>
#include <urdf/model.h>
#include <string>
#include "IK6D_arm6dof_upto_palm.h"

using std::string;

static const std::string IK_SERVICE = "get_ik";
static const std::string FK_SERVICE = "get_fk";
static const std::string IK_INFO_SERVICE = "get_ik_solver_info";
static const std::string FK_INFO_SERVICE = "get_fk_solver_info";

#define IK_EPS	1e-5

class Kinematics {
    public:
        Kinematics();
        bool init();

    private:
        ros::NodeHandle nh, nh_private;
        std::string root_name, tip_name;
        KDL::JntArray joint_min, joint_max;
        KDL::Chain chain;
        unsigned int num_joints;

        KDL::ChainFkSolverPos_recursive* fk_solver;
        ikfast::IKSolver *myIKfast;

        double epsilon;

        ros::ServiceServer ik_service,ik_solver_info_service;
        ros::ServiceServer fk_service,fk_solver_info_service;

        tf::TransformListener tf_listener;

        kinematics_msgs::KinematicSolverInfo info;

        bool loadModel(const std::string xml);
        bool readJoints(urdf::Model &robot_model);
        int getJointIndex(const std::string &name);
        int getKDLSegmentIndex(const std::string &name);
        float jointlimitCostfunction(KDL::JntArray jnt_pos_in);

        /**
         * @brief This is the basic IK service method that will compute and return an IK solution.
         * @param A request message. See service definition for GetPositionIK for more information on this message.
         * @param The response message. See service definition for GetPositionIK for more information on this message.
         */
        bool getPositionIK(kinematics_msgs::GetPositionIK::Request &request,
                           kinematics_msgs::GetPositionIK::Response &response);

        /**
         * @brief This is the basic kinematics info service that will return information about the kinematics node.
         * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
         * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
         */
        bool getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request,
                             kinematics_msgs::GetKinematicSolverInfo::Response &response);

        /**
         * @brief This is the basic kinematics info service that will return information about the kinematics node.
         * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
         * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
         */
        bool getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request,
                             kinematics_msgs::GetKinematicSolverInfo::Response &response);

        /**
         * @brief This is the basic forward kinematics service that will return information about the kinematics node.
         * @param A request message. See service definition for GetPositionFK for more information on this message.
         * @param The response message. See service definition for GetPositionFK for more information on this message.
         */
        bool getPositionFK(kinematics_msgs::GetPositionFK::Request &request,
                           kinematics_msgs::GetPositionFK::Response &response);
};



Kinematics::Kinematics(): nh_private ("~") {
}

bool Kinematics::init() {
    // Get URDF XML
    std::string urdf_xml, full_urdf_xml;
    nh.param("urdf_xml",urdf_xml,std::string("robot_description"));
    nh.searchParam(urdf_xml,full_urdf_xml);
    ROS_DEBUG("Reading xml file from parameter server");
    std::string result;
    if (!nh.getParam(full_urdf_xml, result)) {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return false;
    }

    // Get Root and Tip From Parameter Service
    if (!nh_private.getParam("root_name", root_name)) {
        ROS_FATAL("GenericIK: No root name found on parameter server");
        return false;
    }
    if(root_name!="shadowarm_base") {
			ROS_FATAL("Current solver can only resolve to root frame = shadowarm_base");
		return false;
		}

    if (!nh_private.getParam("tip_name", tip_name)) {
        ROS_FATAL("GenericIK: No tip name found on parameter server");
        return false;
    }

		if(tip_name.find("palm")==string::npos) {
			ROS_FATAL("Current solver can only resolve to one of the distal frames");
			return false;
		}
    
    // Load and Read Models
    if (!loadModel(result)) {
        ROS_FATAL("Could not load models!");
        return false;
    }

    // Get Solver Parameters
    int maxIterations;

    nh_private.param("maxIterations", maxIterations, 1000);
    nh_private.param("epsilon", epsilon, 1e-2);

    // Build Solvers
    fk_solver = new KDL::ChainFkSolverPos_recursive(chain); //keep the standard arm_kinematics fk_solver although we have ours.

    ROS_INFO("Advertising services");
    fk_service = nh_private.advertiseService(FK_SERVICE,&Kinematics::getPositionFK,this);
    ik_service = nh_private.advertiseService(IK_SERVICE,&Kinematics::getPositionIK,this);
    ik_solver_info_service = nh_private.advertiseService(IK_INFO_SERVICE,&Kinematics::getIKSolverInfo,this);
    fk_solver_info_service = nh_private.advertiseService(FK_INFO_SERVICE,&Kinematics::getFKSolverInfo,this);
    
    myIKfast = new ikfast::IKSolver;

    return true;
}

bool Kinematics::loadModel(const std::string xml) {
    urdf::Model robot_model;
    KDL::Tree tree;

    if (!robot_model.initString(xml)) {
        ROS_FATAL("Could not initialize robot model");
        return -1;
    }
    if (!kdl_parser::treeFromString(xml, tree)) {
        ROS_ERROR("Could not initialize tree object");
        return false;
    }
    if (!tree.getChain(root_name, tip_name, chain)) {
        ROS_ERROR("Could not initialize chain object");
        return false;
    }

    if (!readJoints(robot_model)) {
        ROS_FATAL("Could not read information about the joints");
        return false;
    }

    return true;
}

bool Kinematics::readJoints(urdf::Model &robot_model) {
    num_joints = 0;
    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint;

	urdf::Vector3 length;

    while (link && link->name != root_name) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (!joint) {
            ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
            return false;
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
            num_joints++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }

    joint_min.resize(num_joints);
    joint_max.resize(num_joints);
    info.joint_names.resize(num_joints);
    info.link_names.resize(num_joints);
    info.limits.resize(num_joints);

    link = robot_model.getLink(tip_name);
    unsigned int i = 0;
    while (link && i < num_joints) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

            float lower, upper;
            int hasLimits;
            if ( joint->type != urdf::Joint::CONTINUOUS ) {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
                hasLimits = 1;
            } else {
                lower = -M_PI;
                upper = M_PI;
                hasLimits = 0;
            }
            int index = num_joints - i -1;

            joint_min.data[index] = lower;
            joint_max.data[index] = upper;
            info.joint_names[index] = joint->name;
            info.link_names[index] = link->name;
            info.limits[index].joint_name = joint->name;
            info.limits[index].has_position_limits = hasLimits;
            info.limits[index].min_position = lower;
            info.limits[index].max_position = upper;
            i++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }
    return true;
    
}


int Kinematics::getJointIndex(const std::string &name) {
		for (unsigned int i=0; i < info.joint_names.size(); i++) {
        if (info.joint_names[i] == name)
            return i;
    }
    return -1;
}

int Kinematics::getKDLSegmentIndex(const std::string &name) {
    int i=0; 
    while (i < (int)chain.getNrOfSegments()) {
        if (chain.getSegment(i).getName() == name) {
            return i+1;
        }
        i++;
    }
    return -1;
}

float Kinematics::jointlimitCostfunction(KDL::JntArray jnt_pos_in)
{
		float sum=0;
	  for(unsigned int j=0; j<num_joints; j++)
		{
				float thetaminmax=joint_max(j)-joint_min(j);
				float thetaminangle=jnt_pos_in(j)-joint_min(j);
				float thetamaxangle=joint_max(j)-jnt_pos_in(j);
				
				sum+= (thetaminmax*thetaminmax)/(thetaminangle*thetamaxangle);
		}
		sum/=4;
		return sum;
}

bool Kinematics::getPositionIK(kinematics_msgs::GetPositionIK::Request &request,
                               kinematics_msgs::GetPositionIK::Response &response) {

	
		// get the 3D position of requested goal (forget about orientation, which is not relevant)
/*		geometry_msgs::PointStamped point_msg_in;
		point_msg_in.header.frame_id = request.ik_request.pose_stamped.header.frame_id;
		point_msg_in.header.stamp = ros::Time::now()-ros::Duration(1);
		point_msg_in.point=request.ik_request.pose_stamped.pose.position; 

		tf::Stamped<tf::Point> transform;
		tf::Stamped<tf::Point> transform_root;
    tf::Stamped<tf::Point> transform_finger_base;
    tf::pointStampedMsgToTF( point_msg_in, transform );
	
		// IK computation variables
		tf::Point p; //local req_point coordinates
		
		
		
		
		   //Convert F to our root_frame	
		ROS_DEBUG("sr_kin: Get point in root frame");
    try {
        tf_listener.transformPoint(root_name, transform, transform_root);
    } catch (...) {
        ROS_ERROR("Could not transform IK pose to frame: %s", root_name.c_str());
        response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
       return false;
    }
		ROS_DEBUG("root x,y,z:%f,%f,%f",transform_root.x(),transform_root.y(),transform_root.z());
		
		
		
		
		 */
		
		
		// get 6D pos in local frame
		geometry_msgs::PoseStamped pose_msg_in = request.ik_request.pose_stamped;
    tf::Stamped<tf::Pose> transform;
    tf::Stamped<tf::Pose> transform_root;
    tf::poseStampedMsgToTF( pose_msg_in, transform );	
		
		//Convert F to our root_frame
    try {
        tf_listener.transformPose(root_name, transform, transform_root);
    } catch (...) {
        ROS_ERROR("Could not transform IK pose to frame: %s", root_name.c_str());
        response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
       return false;
    }
		
		tf::Point p = transform_root.getOrigin();
		tf::Quaternion quat = transform_root.getRotation();
		btMatrix3x3 Rot(quat);
		tf::Point rotcol1 = Rot.getRow(0);
		tf::Point rotcol2 = Rot.getRow(1);
		tf::Point rotcol3 = Rot.getRow(2);
		
		int ik_valid = 0;
		float delta=0.01;
		
		//std::vector<ikfast6d::IKSolution> vsolutions;
    ikfast::IkSolutionList<IkReal> solutions;
  

		// Change to a local computation frame (at knuckle pos but with palm orientation)
		ROS_DEBUG("sr_kin: Convert Frame to local computation frame");
		
    ROS_DEBUG("x,y,z:%f,%f,%f",p.x(),p.y(),p.z());
    ROS_DEBUG("r0,r1,r2,r3,r4,r5,r6,r7,r8:\n%f %f %f\n%f %f %f\n%f %f %f\n",rotcol1.x(),rotcol1.y(),rotcol1.z(),rotcol2.x(),rotcol2.y(),rotcol2.z(),rotcol3.x(),rotcol3.y(),rotcol3.z());


		//prepare the IK input data.
    IkReal eerot[9],eetrans[3];
    eerot[0] = rotcol1.x(); eerot[1] = rotcol1.y(); eerot[2] = rotcol1.z(); eetrans[0] = p.x();
    eerot[3] = rotcol2.x(); eerot[4] = rotcol2.y(); eerot[5] = rotcol2.z(); eetrans[1] = p.y();
    eerot[6] = rotcol3.x(); eerot[7] = rotcol3.y(); eerot[8] = rotcol3.z(); eetrans[2] = p.z();
    
	    
    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out_temp;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(num_joints);
		jnt_pos_out.resize(num_joints);
		jnt_pos_out_temp.resize(num_joints);  
		
		float costlimit=999;
    ros::Time timer_start=ros::Time::now();
    
		bool bSuccess = myIKfast->ik(eetrans, eerot, NULL, solutions);
    
    ros::Time timer_stop=ros::Time::now();
    ros::Duration ik_duration=timer_stop-timer_start;
    ROS_DEBUG("IK computation time: %f sec",ik_duration.toSec());
    
		if( !bSuccess ) {
        ROS_DEBUG("Failed to get ik solution\n");
        ik_valid = 0;
    }
		else
		{
	 		ROS_DEBUG("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
      std::vector<IkReal> solvalues(GetNumJoints());

			unsigned int	outerlimit=0;
      for(int i = solutions.GetNumSolutions()-1; i >=0 ; i--)
    	{
        ik_valid = 1;
        
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        
        ROS_DEBUG("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);      
                
				// check if solution is within the limits
        for(unsigned int j=0; j<num_joints && j<solvalues.size(); ++j)
				{
						jnt_pos_out_temp(j)=solvalues[j];
						
					  if( jnt_pos_out_temp(j)>joint_max(j)+delta || jnt_pos_out_temp(j)<joint_min(j)-delta || std::isnan(jnt_pos_out_temp(j))  )
					  {
							ROS_DEBUG("pos %d, val: %f out of range [%f,%f]",j,jnt_pos_out_temp(j),joint_min(j),joint_max(j));
					  	outerlimit++;
							ik_valid=0;
							break; //do not need to go anyfurther
					  }
				}
				for(unsigned int j=0; j<num_joints; j++)
						{
							ROS_DEBUG("sol: %d pos %d, val: %f  (range [%f,%f])\n",i, j,jnt_pos_out_temp(j),joint_min(j),joint_max(j) );
						}
				
				// choose the solution that is farthest from the limit using a cost function
        
				if(ik_valid)
				{
          
        // /*
					float tempcostlimit=jointlimitCostfunction(jnt_pos_out_temp);
					if(tempcostlimit<costlimit)
					{
						costlimit=tempcostlimit; //*/
						for(unsigned int i=0; i<num_joints; i++)
						{
							jnt_pos_out(i)=jnt_pos_out_temp(i);
							ROS_DEBUG("Cost: %f, temp solution joint %d,%f",costlimit,i,jnt_pos_out(i));
						}
            // only if removing cost computation => uncomment // break; /*
            
					}
					else
					{
						ROS_DEBUG("Cost: %f > %f",tempcostlimit ,costlimit );
					}
          // */
				}
	        
    	}

			///*
      if(outerlimit>=solutions.GetNumSolutions())
				ik_valid=0;
			else
				ik_valid=1; //*/
      if(ik_valid==1)
      {
        for(unsigned int i=0; i<num_joints; i++)
        {
          ROS_DEBUG("Chosen solution joint %d,%f",i,jnt_pos_out(i));
        }
      }
		}

    	if (ik_valid > 0) {
        response.solution.joint_state.name = info.joint_names;
        response.solution.joint_state.position.resize(num_joints);
        for (unsigned int i=0; i < num_joints; i++) {
            response.solution.joint_state.position[i] = jnt_pos_out(i);
            ROS_DEBUG("IK Solution: %s %d: %f",response.solution.joint_state.name[i].c_str(),i,jnt_pos_out(i));
        }
        response.error_code.val = response.error_code.SUCCESS;
        return true;
    } else {
        ROS_DEBUG("An IK solution could not be found");
        response.error_code.val = response.error_code.NO_IK_SOLUTION;
        return true;
    }
    return true;
}

bool Kinematics::getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request,
                                 kinematics_msgs::GetKinematicSolverInfo::Response &response) {
    response.kinematic_solver_info = info;
    return true;
}

bool Kinematics::getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request,
                                 kinematics_msgs::GetKinematicSolverInfo::Response &response) {
    response.kinematic_solver_info = info;
    return true;
}

bool Kinematics::getPositionFK(kinematics_msgs::GetPositionFK::Request &request,
                               kinematics_msgs::GetPositionFK::Response &response) {
    KDL::Frame p_out;
    KDL::JntArray jnt_pos_in;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

		IkReal jin[num_joints];


		IkReal eerot[9],eetrans[3];


    jnt_pos_in.resize(num_joints);
    for (unsigned int i=0; i < num_joints; i++) {
        int tmp_index = getJointIndex(request.robot_state.joint_state.name[i]);
        if (tmp_index >=0)
				{
            //old request through KDL jnt_pos_in(tmp_index) = request.robot_state.joint_state.position[i];
						jin[i]=request.robot_state.joint_state.position[i];
				}
    }

   		myIKfast->fk(jin,eetrans, eerot);

      ROS_DEBUG("fk x,y,z:%f,%f,%f",eetrans[0],eetrans[1],eetrans[2]);
	    ROS_DEBUG("fk r0,r1,r2,r3,r4,r5,r6,r7:\n%f %f %f\n%f %f %f\n%f %f %f\n",eerot[0],eerot[1],eerot[2],eerot[3],eerot[4],eerot[5],eerot[6],eerot[7],eerot[8]);

    response.pose_stamped.resize(request.fk_link_names.size());
    response.fk_link_names.resize(request.fk_link_names.size());

    //bool valid = true;
    for (unsigned int i=0; i < request.fk_link_names.size(); i++) {
        int segmentIndex = getKDLSegmentIndex(request.fk_link_names[i]);
        ROS_DEBUG("End effector index: %d",segmentIndex);
        ROS_DEBUG("Chain indices: %d",chain.getNrOfSegments());
        //old request through KDL //if (fk_solver->JntToCart(jnt_pos_in,p_out,segmentIndex) >=0) {
            tf_pose.frame_id_ = root_name;
            tf_pose.stamp_ = ros::Time();
            //old request through KDL tf::PoseKDLToTF(p_out,tf_pose);
            //old request through KDL 
            /* try {
                tf_listener.transformPose(request.header.frame_id,tf_pose,tf_pose);
            } catch (...) {
                ROS_ERROR("Could not transform FK pose to frame: %s",request.header.frame_id.c_str());
                response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
                return false;
            }
            */
            //tf::poseStampedTFToMsg(tf_pose,pose);
            //store position
            pose.pose.position.x = (double)eetrans[0];
            pose.pose.position.y = (double)eetrans[1];
            pose.pose.position.z = (double)eetrans[2];
            
            //transform a rotation matrix into quaternion
            btMatrix3x3 Rot;
            Rot.setValue((double)eerot[0],(double)eerot[1],(double)eerot[2],(double)eerot[3],(double)eerot[4],(double)eerot[5],(double)eerot[6],(double)eerot[7],(double)eerot[8]);
            tf::Quaternion quat;
            Rot.getRotation(quat);
            ROS_DEBUG("Fk result Quat %8.4f %8.4f %8.4f %8.3f",quat.getX(),quat.getY(),quat.getZ(),quat.getW() );
            //store orientation
            pose.pose.orientation.x = quat.getX();
            pose.pose.orientation.y = quat.getY();
            pose.pose.orientation.z = quat.getZ();
            pose.pose.orientation.w = quat.getW();
            
            pose.header.frame_id="shadowarm_base";
            response.pose_stamped[i] = pose;
            
            response.fk_link_names[i] = request.fk_link_names[i];
            response.error_code.val = response.error_code.SUCCESS;
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sr_kinematics");
    Kinematics k;
    if (k.init()<0) {
        ROS_ERROR("Could not initialize kinematics node");
        return -1;
    }

    ros::spin();
    return 0;
}

