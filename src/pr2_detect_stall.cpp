#include <cmath>        // std::abs


#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Float32.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>


float prev_joint_value = 0.0;
float error = 0.0;

float prev_effort_value = 0.0;
float effort = 0.0;
void actualJointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{

	for(int i = 0; i < msg->name.size(); i++)
	{
		if(msg->name[i] == "l_elbow_flex_joint")
		{
			//std::cout << "l_elbow_flex_joint, position :" << msg->position[i] << std::endl;
			//std::cout << "l_elbow_flex_joint, timestamp :" << msg->header.stamp.sec << std::endl;

			error = (msg->position[i] - prev_joint_value) * (msg->position[i] - prev_joint_value)*100000000;
			prev_joint_value = msg->position[i];

			effort = (msg->effort[i] - prev_effort_value) * (msg->effort[i] - prev_effort_value);
			prev_effort_value = msg->effort[i];

			//std::cout << std::fixed << std::setprecision(6) << "Previous  :" << prev_joint_value << std::endl;

		  //std::cout << std::fixed << std::setprecision(6) << "ERROR  :" << error << std::endl;
		}
	}

}

void motionPlanRequestCallback(const moveit_msgs::MotionPlanRequestConstPtr& msg)
{

	for(int i=0; i < msg->goal_constraints.size(); i++)
	{
		if (msg->goal_constraints[0].joint_constraints[i].joint_name == "l_gripper_joint")
		{
		//std::cout << "l_gripper_joint position wanted :" << msg->goal_constraints[0].joint_constraints[i].position << std::endl;
		//std::cout << "l_gripper_joint position wanted timestamp :" << msg->workspace_parameters.header.stamp.sec << std::endl;
	  }
	}
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "pr2_stall_event");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	// Subscribe to joint_state for update on actual position of robot
	ros::Subscriber sub = nh.subscribe("/joint_states", 1000, actualJointStateCallback);
  	ros::Rate loop_rate(2);

	ros::Subscriber sub2 = nh.subscribe("/move_group/motion_plan_request", 1000, motionPlanRequestCallback);

	ros::Rate r(10); // 10 hz
	ros::Publisher l_arm_effort_value = nh.advertise<std_msgs::Float32>("l_arm_effort", 5);
	ros::Publisher l_arm_joint_value = nh.advertise<std_msgs::Float32>("l_arm_joint", 5);
	std_msgs::Float32 effortError;
	std_msgs::Float32 jointError;
	while(ros::ok())
	{
			effortError.data = error;
			l_arm_effort_value.publish(effortError);

			jointError.data = error;
			l_arm_joint_value.publish(jointError);
			r.sleep();
	}

	return 0;
}
