#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Pose.h>
//MOVE IT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

float positionX = -0.0023734011670250126;
float positionY = 0.026825106858739834;
float positionZ = 0.32402280551708873;

float orientationX = 0.4966690471541269;
float orientationY = -0.4966708043538031;
float orientationZ = -0.5033089303160498;
float orientationW = -0.5033071531040186;

void chatterCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	float new_positionX = msg->position.x;
	float new_positionY = msg->position.y;
	float new_positionZ = msg->position.z;

	ROS_INFO("saved_x: [%.9f], new:[%.9f]", positionX, new_positionX);
	ROS_INFO("saved_y: [%.9f], new [%.9f]", positionY, new_positionY);
	ROS_INFO("saved_z: [%.9f], new [%.9f]", positionZ, new_positionZ);
 	

	if (new_positionX!=positionX )
	{

	ROS_INFO("new pose");

	//planning group that we would like to control
	moveit::planning_interface::MoveGroupInterface group("arm");
	

	//raw pointers are used to refer to the planning group for improved performance
	const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup("arm");
	group.setEndEffectorLink("arm3");
	group.setPoseReferenceFrame("base");
	group.setPlannerId("base");
	group.setNumPlanningAttempts(50);
	group.setPlanningTime(50.0);
	group.allowReplanning(true);
	group.setGoalJointTolerance(0.0001);
	group.setGoalPositionTolerance(0.0001);
	group.setGoalOrientationTolerance(0.001);
	//group.setNamedTarget("random");
	group.setRandomTarget();
	group.move();  // WORKS FINE :)

	//  CUSTOM PLANNING
	geometry_msgs::Pose target_pose1;
	//NOTE: THIS IS THE VALID POSE FROM RANDOM NODE

	   
	target_pose1.orientation.w = msg->orientation.w;
	target_pose1.orientation.x = msg->orientation.x;
	target_pose1.orientation.y = msg->orientation.y;
	target_pose1.orientation.z = msg->orientation.z;
	
	target_pose1.position.x =  msg->position.x;
	target_pose1.position.y =  msg->position.y;
	target_pose1.position.z =  msg->position.z;

	group.setStartStateToCurrentState();
	group.setPoseTarget(target_pose1,"arm3");

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit_msgs::MotionPlanRequest response;

	group.plan(my_plan);
	group.execute(my_plan);

	
	ROS_INFO("pose updated");




	positionX = msg->position.x;
	positionY = msg->position.y;
	positionZ = msg->position.z;
	orientationX = msg->orientation.x;
	orientationY = msg->orientation.y;
	orientationZ = msg->orientation.z;
	orientationW = msg->orientation.w;
	
	}
	

}



int main(int argc, char **argv)
{

ros::init(argc, argv, "move_group_interface");
ros::NodeHandle nh;
ROS_INFO("initilize !");


// define user callback queue
  ros::CallbackQueue string_queue;
  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions ops =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
      "/direction", // topic name
      1000, // queue length
      chatterCallback, // callback
      ros::VoidPtr(), // tracked object, we don't need one thus NULL
      &string_queue // pointer to callback queue object
    );
  // subscribe
  ros::Subscriber sub2 = nh.subscribe(ops);


ros::AsyncSpinner spinner(1, &string_queue);
spinner.start();




ros::Rate r(10); // 10 hz
while (ros::ok())
  {
    ros::spinOnce();

    r.sleep();
  }

return 0;

}
