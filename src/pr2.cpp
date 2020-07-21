#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace moveit::task_constructor;

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi) {

	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.6;
	o.primitive_poses[0].position.y = 0.4;
	o.primitive_poses[0].position.z = 0.75+(0.20/2)+0.01;
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.05;
	o.primitives[0].dimensions[1]= 0.05;
	o.primitives[0].dimensions[2]= 0.20;
	psi.applyCollisionObject(o);

	o.id= "obstacle";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.6;
	o.primitive_poses[0].position.y = -0.1;
	o.primitive_poses[0].position.z = 0.75+(0.40/2);
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.05;
	o.primitives[0].dimensions[1]= 0.05;
	o.primitives[0].dimensions[2]= 0.40;
	psi.applyCollisionObject(o);

	o.id= "obstacle2";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.6;
	o.primitive_poses[0].position.y = -0.3;
	o.primitive_poses[0].position.z = 0.75+(0.40/2);
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.05;
	o.primitives[0].dimensions[1]= 0.05;
	o.primitives[0].dimensions[2]= 0.40;
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

	auto cartesian = std::make_shared<solvers::CartesianPath>();
	cartesian->setProperty("jump_threshold", 0.0);

	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	t.add(std::move(initial));

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRT");

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{"left_gripper", pipeline}, {"left_arm", pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));
	}

	{
		// grasp generator
		auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose left");
		grasp_generator->setAngleDelta(M_PI/2);
		//grasp_generator->setTopGraspEnable(true);
		grasp_generator->setPreGraspPose("left_open");
		grasp_generator->setGraspPose("left_close");
		grasp_generator->setProperty("object", std::string("object"));
		grasp_generator->setMonitoredStage(current_state);
		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
		Eigen::Isometry3d tr = Eigen::Isometry3d::Identity();
		//tr.translation() = Eigen::Vector3d(0.0,0.0,0.0);
		grasp->setIKFrame(tr, "l_gripper_tool_frame");
		grasp->setMaxIKSolutions(10);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp),"pick left");
		pick->setProperty("eef", "left_gripper");
		pick->setProperty("group","left_arm");
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "l_gripper_tool_frame";
		approach.twist.linear.x = 1.0;
		pick->setApproachMotion(approach, 0.01, 0.10);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "base_footprint";
		lift.twist.linear.x =  0.0;
		lift.twist.linear.y =  0.0;
		lift.twist.linear.z =  1.0;
		pick->setLiftMotion(lift, 0.05, 0.10);
		current_state = pick.get();
		t.add(std::move(pick));
	}

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{"left_gripper", pipeline}, {"left_arm", pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));
	}

	{
		auto stage = std::make_unique<stages::GeneratePlacePose>("place pose");
		geometry_msgs::PoseStamped p;
		p.header.frame_id= "base_footprint";
		p.pose.position.x=  0.6;
		p.pose.position.y=  -0.2;
		p.pose.position.z=  0.75+(0.23/2)+0.01;
		stage->setPose(p);
		stage->setObject("object");
		stage->setProperty("eef", "left_gripper");
		stage->setProperty("group","left_arm");

		geometry_msgs::PoseStamped ik;
		ik.header.frame_id= "l_gripper_tool_frame";
		ik.pose.position.x=  0;
		ik.pose.position.y=  0;
		ik.pose.position.z=  0;
		stage->setProperty("ik_frame",ik);
		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("place pose kinematics", std::move(stage));
		wrapper->setMaxIKSolutions(10);
		wrapper->setIKFrame(Eigen::Isometry3d::Identity(),"l_gripper_tool_frame");
		wrapper->setProperty("eef", "left_gripper");
		wrapper->setProperty("group","left_arm");
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		t.add(std::move(wrapper));
	}


	{
		auto stage = std::make_unique<stages::MoveTo>("open left hand", pipeline);
		stage->setProperty("eef", "left_gripper");
		stage->setProperty("group", "left_gripper");
		stage->setGoal("left_open");
		t.add(std::move(stage));
	}

	/******************************************************
---- *          Detach Object                             *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
		stage->detachObject("object", "l_gripper_tool_frame");
		t.add(std::move(stage));
	}

	/******************************************************
---- *          Retreat Motion                            *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("retreat after place", pipeline);
		stage->setProperty("group", "left_arm");
		stage->setProperty("eef", "left_gripper");
		stage->setMinMaxDistance(.01, .05);
		stage->setIKFrame(Eigen::Isometry3d::Identity(),"l_gripper_tool_frame");
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = "l_gripper_tool_frame";
		vec.vector.x = -1.0;
		stage->setDirection(vec);
	  t.add(std::move(stage));
	}

  std::cerr << t << std::endl;
	t.plan(2);
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
		ac.waitForResult();
		moveit_msgs::MoveItErrorCodes execute_result = ac.getResult()->error_code;

		if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
			ROS_ERROR_STREAM("Task execution failed and returned: " << ac.getState().toString());
		}

	}

}

int main(int argc, char** argv){
	ros::init(argc, argv, "pr2");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	namespace rvt = rviz_visual_tools;

	// Store the planning scene where the collision object will be add
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// Store the collision objects added to the scene
	std::vector<moveit_msgs::CollisionObject> collision_objects_vector;

	spawnObject(planning_scene_interface);

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

	execute(t);

	return 0;
}
