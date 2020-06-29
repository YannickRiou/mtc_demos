#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <cmath>        // std::abs

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>


using namespace moveit::task_constructor;

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi) {

	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.65;
	o.primitive_poses[0].position.y = 0.05;
	o.primitive_poses[0].position.z = 0.75+(0.23/2)+0.02;
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.00;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0]= 0.23;
	o.primitives[0].dimensions[1]= 0.03;
	psi.applyCollisionObject(o);


	o.id= "object_obstacle";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.75;
	o.primitive_poses[0].position.y = 0.05;
	o.primitive_poses[0].position.z = 0.75+(0.23/2)+0.02+0.25;
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.00;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.02;
	o.primitives[0].dimensions[1]= 0.15;
	o.primitives[0].dimensions[2]= 0.15;
	psi.applyCollisionObject(o);

	moveit_msgs::CollisionObject table;
	table.id= "tableLaas";
	table.header.frame_id= "base_footprint";
	table.primitive_poses.resize(1);
	table.primitive_poses[0].position.x = 0.95;
	table.primitive_poses[0].position.y = 0.0;
	table.primitive_poses[0].position.z = 0.75/2;
	table.primitive_poses[0].orientation.x =0.0;
	table.primitive_poses[0].orientation.y =0.0;
	table.primitive_poses[0].orientation.z =0.0;
	table.primitive_poses[0].orientation.w =1.0;
	table.primitives.resize(1);
	table.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	table.primitives[0].dimensions.resize(3);
	table.primitives[0].dimensions[0]= 0.85;
	table.primitives[0].dimensions[1]= 1.35;
	table.primitives[0].dimensions[2]= 0.75;
	psi.applyCollisionObject(table);
}

void planTest(Task &t) {

	t.loadRobotModel();

	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setProperty("jump_threshold", 0.0);
	cartesian_planner->setProperty("step_size",0.1);

	auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

	// planner used for connect
	auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>();
	pipeline_planner->setProperty("jump_threshold", 0.0);
	pipeline_planner->setPlannerId("RRTConnectkConfigDefault");


	// don't spill liquid
	moveit_msgs::Constraints upright_constraint;
	upright_constraint.name = "left_gripper:upright";
	upright_constraint.orientation_constraints.resize(1);
	{
		moveit_msgs::OrientationConstraint& c= upright_constraint.orientation_constraints[0];
		c.link_name= "l_gripper_tool_frame";
		c.header.frame_id= "base_footprint";
		c.orientation.x= 0;
		c.orientation.y= 0;
		c.orientation.z= 0;
		c.orientation.w= 1;
		c.absolute_x_axis_tolerance= 0.65;
		c.absolute_y_axis_tolerance= 0.65;
		c.absolute_z_axis_tolerance= M_PI;
		c.weight= 1.0;
	}



	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	t.add(std::move(initial));

  {
	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{"left_gripper", pipeline_planner}, {"left_arm", pipeline_planner}};
	auto connect = std::make_unique<stages::Connect>("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);
	t.add(std::move(connect));
  }

	{
		/*// grasp generator
		auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose left");
		//grasp_generator->setTopGraspEnable(true);
		grasp_generator->setAngleDelta(M_PI/4);
		grasp_generator->setPreGraspPose("left_open");
		grasp_generator->setGraspPose("left_close");
		grasp_generator->setMonitoredStage(current_state);*/


		auto stage = std::make_unique<stages::GeneratePose>("GeneratePickPose");
		geometry_msgs::PoseStamped p;
		p.header.frame_id= "object";
		p.pose.position.x=  -0.05;
		p.pose.position.y=  0;
		p.pose.position.z=  0;
		stage->setPose(p);
		stage->setProperty("pregrasp","left_open");
		stage->setProperty("grasp","left_close");
		stage->setProperty("object","object");
		stage->setProperty("eef","left_gripper");
		stage->setProperty("group","left_arm");
		stage->setMonitoredStage(current_state);

		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(stage));
		grasp->setIKFrame(Eigen::Isometry3d::Identity(), "l_gripper_tool_frame");
		grasp->properties().configureInitFrom(Stage::PARENT);
		grasp->setMaxIKSolutions(10);


		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp));
		pick->setProperty("eef", "left_gripper");
		pick->setProperty("object", std::string("object"));
		pick->properties().configureInitFrom(Stage::PARENT);

		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "l_wrist_roll_link";
		approach.twist.linear.x = 1.0;
		pick->setApproachMotion(approach, 0.01, 0.10);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "base_footprint";
		lift.twist.linear.z =  1.0;
		pick->setLiftMotion(lift, 0.05, 0.10);
		current_state = pick.get();
		t.add(std::move(pick));
	}

	{
		stages::Connect::GroupPlannerVector planners = {{"left_gripper", pipeline_planner}, {"left_arm", pipeline_planner}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->setPathConstraints(upright_constraint);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("place pose");
		geometry_msgs::PoseStamped p;
		p.header.frame_id= "base_footprint";
		p.pose.position.x=  0.90;
		p.pose.position.y=  0.05;
		p.pose.position.z=  0.90;
		stage->setPose(p);
		stage->properties().configureInitFrom(Stage::PARENT);

		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("place pose kinematics", std::move(stage));
		wrapper->setMaxIKSolutions(32);
		wrapper->setIKFrame(Eigen::Isometry3d::Identity(),"l_gripper_tool_frame");
		wrapper->setProperty("eef", "left_gripper");
		wrapper->setProperty("group","left_arm");
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		t.add(std::move(wrapper));
	}


  std::cerr << t << std::endl;
	t.plan(5);
}

void execute(Task &t)
{

	if(t.solutions().size() > 0)
	{
		actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> ac("execute_task_solution", true);
		ac.waitForServer();
		ROS_INFO("Executing solution trajectory");
		moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
		t.solutions().front()->fillMessage(execute_goal.solution);
		ac.sendGoal(execute_goal);
		//ros::Duration(1.5).sleep(); // sleep for half a seco
		//ac.cancelGoal();
		ac.waitForResult();
		moveit_msgs::MoveItErrorCodes execute_result = ac.getResult()->error_code;

		if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
			ROS_ERROR_STREAM("Task execution failed and returned: " << ac.getState().toString());
		}
	}

}

float prev_joint_value = 0.0;

void actualJointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
	float error = 0.0;
	for(int i = 0; i < msg->name.size(); i++)
	{
		if(msg->name[i] == "l_forearm_roll_joint")
		{
			std::cout << "l_forearm_roll_joint position :" << msg->position[i] << std::endl;
			std::cout << "l_forearm_roll_joint timestamp :" << msg->header.stamp.sec << std::endl;

			error = (msg->position[i] - prev_joint_value);
			prev_joint_value = msg->position[i];
			 std::cout << std::fixed << std::setprecision(6) << "Previous  :" << prev_joint_value << std::endl;

		  std::cout << std::fixed << std::setprecision(6) << "ERROR  :" << error << std::endl;
		}
	}

}

void motionPlanRequestCallback(const moveit_msgs::MotionPlanRequestConstPtr& msg)
{

	for(int i=0; i < msg->goal_constraints.size(); i++)
	{
		if (msg->goal_constraints[0].joint_constraints[i].joint_name == "l_gripper_joint")
		{
		std::cout << "l_gripper_joint position wanted :" << msg->goal_constraints[0].joint_constraints[i].position << std::endl;
		std::cout << "l_gripper_joint position wanted timestamp :" << msg->workspace_parameters.header.stamp.sec << std::endl;
	  }
	}
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "pr2");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	// Subscribe to joint_state for update on actual position of robot
	ros::Subscriber sub = nh.subscribe("/joint_states", 1000, actualJointStateCallback);
  ros::Rate loop_rate(2);

	ros::Subscriber sub2 = nh.subscribe("/move_group/motion_plan_request", 1000, motionPlanRequestCallback);

	/*const std::string RIGHT_ARM_PLANNING_GROUP = "right_arm";
	const std::string	LEFT_ARM_PLANNING_GROUP = "left_arm";

	const robot_state::JointModelGroup* right_arm_joint_model_group;
  const robot_state::JointModelGroup* left_arm_joint_model_group;

  moveit::planning_interface::MoveGroupInterface right_arm_move_group(RIGHT_ARM_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface left_arm_move_group(LEFT_ARM_PLANNING_GROUP);

	right_arm_joint_model_group =
		right_arm_move_group.getCurrentState()->getJointModelGroup(RIGHT_ARM_PLANNING_GROUP);
	left_arm_joint_model_group =
		left_arm_move_group.getCurrentState()->getJointModelGroup(LEFT_ARM_PLANNING_GROUP);

	// Set Arm to initial pose
	right_arm_move_group.setNamedTarget("RIGHT_ARM_INITIAL_POSE");
	left_arm_move_group.setNamedTarget("LEFT_ARM_INITIAL_POSE");*/

	// Move the arms to initial pose (away from robot vision)
	//right_arm_move_group.move();
	//left_arm_move_group.move();

	// Store the planning scene where the collision object will be add
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// Store the collision objects added to the scene
	std::vector<moveit_msgs::CollisionObject> collision_objects_vector;

	/*spawnObject(planning_scene_interface);

	Task t("myTask");
	try {
		planTest(t);

		std::cout << "waiting for any key + <enter>\n";
		char ch;
		std::cin >> ch;
	}
	catch (const InitStageException &e) {
		std::cerr << e;
		return EINVAL;
	}

	execute(t);*/

	while(ros::ok());

	return 0;
}
