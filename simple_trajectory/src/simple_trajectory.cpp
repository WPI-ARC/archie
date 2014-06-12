#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    // Fill up points
    trajectory_msgs::JointTrajectoryPoint point;
    

    double time =0.0;
    double dt = 4.0;
    
    // ---------------------------------------
    point.positions.clear();

    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    // Velocities
    point.velocities.resize( point.positions.size() );
    for (size_t j = 0; j < point.velocities.size(); ++j)
      point.velocities[j] = 0.0;

    // To be reached 1 second after starting along the trajectory
    time += dt;
    point.time_from_start = ros::Duration( time );
    goal.trajectory.points.push_back( point );

    // ---------------------------------------
    point.positions.clear();

    point.positions.push_back( -0.3 );
    point.positions.push_back( 0.2 );
    point.positions.push_back( -0.1 );
    point.positions.push_back( -1.2 ) ;
    point.positions.push_back( 1.5 );
    point.positions.push_back( -0.3 );
    point.positions.push_back( 0.5 );

    // Velocities
    point.velocities.resize( point.positions.size() );
    for (size_t j = 0; j < point.velocities.size(); ++j)
      point.velocities[j] = 0.0;

    // To be reached 1 second after starting along the trajectory
    time += dt;
    point.time_from_start = ros::Duration( time );
    goal.trajectory.points.push_back( point );

    // ---------------------------------------
    point.positions.clear();

    point.positions.push_back( -1.0542 );
    point.positions.push_back( 0.280 ) ;
    point.positions.push_back( -0.421 );
    point.positions.push_back( -1.2 );
    point.positions.push_back( 0.0 ) ;
    point.positions.push_back( -1.17 );
    point.positions.push_back( 0.0 );

    // Velocities
    point.velocities.resize( point.positions.size() );
    for (size_t j = 0; j < point.velocities.size(); ++j)
      point.velocities[j] = 0.0;

    // To be reached 1 second after starting along the trajectory
    time += dt;
    point.time_from_start = ros::Duration( time );
    goal.trajectory.points.push_back( point );

    // ---------------------------------------
    point.positions.clear();

    point.positions.push_back( -1.371 );
    point.positions.push_back( 0.0 );
    point.positions.push_back( 0.0 );
    point.positions.push_back( -0.96 );
    point.positions.push_back( 0.0 );
    point.positions.push_back( 0.0 );
    point.positions.push_back( 0.0 );

    // Velocities
    point.velocities.resize( point.positions.size() );
    for (size_t j = 0; j < point.velocities.size(); ++j)
      point.velocities[j] = 0.0;

    // To be reached 1 second after starting along the trajectory
    time += dt;
    point.time_from_start = ros::Duration( time );
    goal.trajectory.points.push_back( point );

    // ---------------------------------------
    point.positions.clear();

    point.positions.push_back( 0.0 );
    point.positions.push_back( 0.0 ) ;
    point.positions.push_back( 0.0 );
    point.positions.push_back( 0.0 );
    point.positions.push_back( 0.0 );
    point.positions.push_back( 0.0 );
    point.positions.push_back( 0.0 );

    // Velocities
    point.velocities.resize( point.positions.size() );
    for (size_t j = 0; j < point.velocities.size(); ++j)
      point.velocities[j] = 0.0;

    // To be reached 1 second after starting along the trajectory
    time += dt;
    point.time_from_start = ros::Duration( time );
    goal.trajectory.points.push_back( point );

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
}
