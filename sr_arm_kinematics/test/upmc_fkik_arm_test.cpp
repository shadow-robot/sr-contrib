/* upmc_fkik_arm_test.cpp
 *
 * test the forward/inverse kinematics for the Shadow Arm 
 *
 * 08.08.2012 - new file
 * Originaly developed for the Shadow Hand fingers by 
 * (C) 2012 fnh
 *  
 * Modified for the Arm by Guillaume WALCK (UPMC) October 2012
 */


#include <map>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
// #include <std_msgs/Float64.h>
// #include <pr2_mechanism_msgs/ListControllers.h>

#define DEG2RAD  (M_PI / 180.0)
#define RAD2DEG  (180.0 / M_PI)

// the maximum error (in radians) that we tolerate to count IK as a success
#define MAX_DELTA 0.01


double rand_range(double min_n, double max_n) {
  return (double)rand() / RAND_MAX * (max_n - min_n) + min_n;
}


void random_test_finger_fkik( ros::NodeHandle nh, int n_tests ,bool verbose=false) {
  // create joint-names
  std::vector<std::string> jointNames;
  jointNames.push_back( "ShoulderJRotate" ); 
  jointNames.push_back( "ShoulderJSwing" );
  jointNames.push_back( "ElbowJSwing" );
  jointNames.push_back( "ElbowJRotate" );
  jointNames.push_back( "WRJ2" );
  jointNames.push_back( "WRJ1" );


  // we also want a map from joint-name to index, because IK returns
  // the joints in a different order
  // 
  std::map<std::string,int> jointIndexMap; 
  for( unsigned int j=0; j < jointNames.size(); j++ ) {
    jointIndexMap[jointNames[j]] = j;
  }

  // check that the FK/IK services are available for the finger
  //
  ROS_INFO( "waiting for FK/IK service" );
  ros::service::waitForService("shadow_right_arm_kinematics/get_fk_solver_info");
  ros::service::waitForService("shadow_right_arm_kinematics/get_fk");
  ros::service::waitForService("shadow_right_arm_kinematics/get_ik_solver_info");
  ros::service::waitForService("shadow_right_arm_kinematics/get_ik");

  // create the service clients; note that we reuse them throughout
  // the whole test for a given finger
  //
  ros::ServiceClient fk = nh.serviceClient<kinematics_msgs::GetPositionFK>("shadow_right_arm_kinematics/get_fk", true);
  ros::ServiceClient ik = nh.serviceClient<kinematics_msgs::GetPositionIK>("shadow_right_arm_kinematics/get_ik", true);

  kinematics_msgs::GetPositionFK  fkdata;

  // generate n_tests random positions, check that pos(fk) == ik(joints)
  // hardcoded joint limits: J1=J2=J3: 0..90 degrees, J4 -25..25 degrees
  // 
  
  int n_matched = 0;
  int n_iksolved = 0;
  std::vector<double> jj;
  ros::Time time_start=ros::Time::now();
  ros::Time time_stop;
  ros::Duration loop_duration=ros::Duration(0);
  
  for( int i=0; i < n_tests; i++ ) {
    jj.resize(0);
     jj.push_back( rand_range( -45*DEG2RAD, 45*DEG2RAD ) ); // SR
      jj.push_back(rand_range( 0*DEG2RAD, 80*DEG2RAD ) );    // SS
      jj.push_back( rand_range(  30*DEG2RAD, 120*DEG2RAD ) ); // ES
      jj.push_back( rand_range( -80*DEG2RAD, 80*DEG2RAD )); // ER
      jj.push_back( rand_range( -30*DEG2RAD, 10*DEG2RAD ) ); // WRJ2
      jj.push_back( rand_range( -40*DEG2RAD, 20*DEG2RAD ) ); // WRJ1
    

    // call fk to get the fingertip <prefix+tip> position
    //
    fkdata.request.header.frame_id = "shadowarm_base";
    fkdata.request.header.stamp = ros::Time::now();
    fkdata.request.fk_link_names.clear();
    fkdata.request.fk_link_names.push_back( "palm" );
    fkdata.request.robot_state.joint_state.header.stamp = ros::Time::now();
    fkdata.request.robot_state.joint_state.header.frame_id = "";
    fkdata.request.robot_state.joint_state.name.resize( jointNames.size() );
    fkdata.request.robot_state.joint_state.position.resize( jointNames.size() );

    if(verbose)
      ROS_INFO( "FK req fk_link_names[0] is %s",  fkdata.request.fk_link_names[0].c_str() );
    for( unsigned int j=0; j < jointNames.size(); j++ ) {
      fkdata.request.robot_state.joint_state.name[j] = jointNames[j];
      fkdata.request.robot_state.joint_state.position[j] = jj[j];
      if(verbose)
        ROS_INFO( "FK req %d joint %d %s   radians %6.2f", i, j, jointNames[j].c_str(), jj[j] );
    }
    fk.call( fkdata ); // (fkeq, fkres );

    // arm_navigation_msgs::ArmNavigationErrorCodes status = fkdata.response.error_code.val;
    int status = fkdata.response.error_code.val;
    if(verbose)
      ROS_INFO( "FK returned status %d", (int) status );

    std::vector<geometry_msgs::PoseStamped> pp = fkdata.response.pose_stamped;
    geometry_msgs::Pose pose = pp[0].pose; // position.xyz orientation.xyzw
    // string[] fk_link_names
    if (status == fkdata.response.error_code.SUCCESS) {
      if(verbose)
      {
        ROS_INFO("FK angles in degree are %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f", jj[0]*RAD2DEG ,jj[1]*RAD2DEG ,jj[2]*RAD2DEG ,jj[3]*RAD2DEG ,jj[4]*RAD2DEG ,jj[5]*RAD2DEG );
        ROS_INFO( "FK pose is p:%8.4f %8.4f %8.4f, q:%8.4f %8.4f %8.4f %8.4f", 
                  pose.position.x,
                  pose.position.y,
                  pose.position.z,
                  pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w                  
                   );
      }
    }
    else {
      continue; // no use trying IK without FK success
    }

    // now try IK to reconstruct FK angles
    // 
    kinematics_msgs::GetPositionIK::Request  ikreq;
    kinematics_msgs::GetPositionIK::Response ikres;

    ikreq.ik_request.ik_link_name = "palm";
    ikreq.ik_request.pose_stamped.header.frame_id = "shadowarm_base";
    ikreq.ik_request.pose_stamped.pose.position.x = pose.position.x;
    ikreq.ik_request.pose_stamped.pose.position.y = pose.position.y;
    ikreq.ik_request.pose_stamped.pose.position.z = pose.position.z;

    ikreq.ik_request.pose_stamped.pose.orientation.x = pose.orientation.x;
    ikreq.ik_request.pose_stamped.pose.orientation.y = pose.orientation.y;
    ikreq.ik_request.pose_stamped.pose.orientation.z = pose.orientation.z;
    ikreq.ik_request.pose_stamped.pose.orientation.w = pose.orientation.w;

    ikreq.ik_request.ik_seed_state.joint_state.position.resize( jointNames.size() );
    ikreq.ik_request.ik_seed_state.joint_state.name.resize( jointNames.size() );
    for( unsigned int j=0; j < jointNames.size(); j++ ) {
      ikreq.ik_request.ik_seed_state.joint_state.name[j] = jointNames[j];
    }

    time_start=ros::Time::now();
    ik.call( ikreq, ikres );
    time_stop=ros::Time::now();
    loop_duration=loop_duration+(time_stop-time_start);
    
    if (ikres.error_code.val == ikres.error_code.SUCCESS) {
      n_iksolved++;
      if(verbose)
        ROS_INFO( "IK found a solution: " );
      for( unsigned int j=0; j < ikres.solution.joint_state.name.size(); j++) {
        if(verbose)
          ROS_INFO("Joint: %s %f", ikres.solution.joint_state.name[j].c_str(),
                                 ikres.solution.joint_state.position[j] );
      }
      bool ok = true;
      for( unsigned int j=0; j < ikres.solution.joint_state.name.size(); j++) {
        std::string name = ikres.solution.joint_state.name[j];
        int           ix = jointIndexMap[ name ];
        double         x = ikres.solution.joint_state.position[j];
        if (fabs( x - jj[ix] ) > MAX_DELTA) {
          if(verbose)
            ROS_INFO( "FK/IK mismatch for joint %d %s,  %6.4f  %6.4f", j, name.c_str(), x, jj[ix] );
          ok = false;
        }
      }
      if (ok) {
        n_matched++;
      }
      if(verbose)
        ROS_INFO( "FK/IK comparison %s", (ok ? "matched" : "failed" ));
    }
    else
    {
      if(verbose)
        ROS_INFO("NO IK solution found");
      continue;
    }   

    // call fk again but this time with the joint-angles from IK 
    //
    fkdata.request.header.frame_id = "shadowarm_base";
    fkdata.request.header.stamp = ros::Time::now();
    fkdata.request.fk_link_names.clear();
    fkdata.request.fk_link_names.push_back( "palm" );
    fkdata.request.robot_state.joint_state.header.stamp = ros::Time::now();
    fkdata.request.robot_state.joint_state.header.frame_id = "";
    fkdata.request.robot_state.joint_state.name.resize( jointNames.size() );
    fkdata.request.robot_state.joint_state.position.resize( jointNames.size() );
    if(verbose)
      ROS_INFO( "FK req fk_link_names[0] is %s",  fkdata.request.fk_link_names[0].c_str() );
    for( unsigned int j=0; j < ikres.solution.joint_state.name.size(); j++ ) {
      std::string name = ikres.solution.joint_state.name[j];
      int           ix = jointIndexMap[ name ];
      double         x = ikres.solution.joint_state.position[j];

      fkdata.request.robot_state.joint_state.name[ix] = jointNames[ix];
      fkdata.request.robot_state.joint_state.position[ix] = x;
      if(verbose)
        ROS_INFO( "FK req %d joint %d %s   radians %6.2f", i, j, jointNames[ix].c_str(), x );
    }
    fk.call( fkdata ); // (fkeq, fkres );

    status = fkdata.response.error_code.val;
    if(verbose)
      ROS_INFO( "FK returned status %d", (int) status );

    std::vector<geometry_msgs::PoseStamped> ppp = fkdata.response.pose_stamped;
    geometry_msgs::Pose pppose = ppp[0].pose; // position.xyz orientation.xyzw
    // string[] fk_link_names
    if (status == fkdata.response.error_code.SUCCESS) {
      if(verbose)
        ROS_INFO( "FK pose is %8.4f %8.4f %8.4f ,q:%8.4f %8.4f %8.4f %8.4f",
                  pppose.position.x,
                  pppose.position.y,
                  pppose.position.z,
                  pppose.orientation.x,
                  pppose.orientation.y,
                  pppose.orientation.z,
                  pppose.orientation.w);
    }
    if(verbose)
      ROS_INFO( "\n" );
  }

  ROS_WARN( "-#- tested %d random positions, %d IK solved, %d matched in %f sec", n_tests, n_iksolved, n_matched ,loop_duration.toSec());
}



/**
 * interface and selftest (fk==ik) of the forward/inverse kinematics solver
 * for the Shadow hand with J1/J2 finger-couplings, for the Shadow hand thumb,
 * and support for little-finger palm-arch (LFJ5).
 * The solvers start at the "palm" frame and work towards idealized 
 * "fftip" "mftip" "rftip" "lftip" and "thtip" frames, which are approximations
 * to the core of the fingertips (assuming spherical fingertip shape).
 * Actual contact points on the fingertips are therefore about one-half
 * finger radius outside of the positions specified by the *tip frames.
 */
int main(int argc, char **argv)
{
  ros::init (argc, argv, "upmc_fkik_test" );
  ros::NodeHandle nh;
  bool verbose;
  if( argc >1)
  {
    char argument =  (char)(argv[1][0]);
    if(argument=='v')
      verbose=true;
    else
      verbose=false;
  }
  
  int seed = 0;
  while( seed == 0 ) {
    seed = ros::Time::now().sec; // wait until /clock received
  }
  ROS_INFO( "-#- FK/IK test started, random seed is %d", seed );
  srandom( seed );

  random_test_finger_fkik( nh, 1000 ,verbose);

}
