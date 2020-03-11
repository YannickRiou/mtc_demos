#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/compute_ik.h>



#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace moveit::task_constructor;

void spawnObject() {
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.53;
	o.primitive_poses[0].position.y = 0.05;
	o.primitive_poses[0].position.z = 0.84;
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0]= 0.23;
	o.primitives[0].dimensions[1]= 0.03;
	psi.applyCollisionObject(o);

	moveit_msgs::CollisionObject o2;
	o2.id= "object2";
	o2.header.frame_id= "base_footprint";
	o2.primitive_poses.resize(1);
	o2.primitive_poses[0].position.x = 0.53;
	o2.primitive_poses[0].position.y = 0.40;
	o2.primitive_poses[0].position.z = 0.84;
	o2.primitive_poses[0].orientation.x =0.0;
	o2.primitive_poses[0].orientation.y =0.0;
	o2.primitive_poses[0].orientation.z =0.0;
	o2.primitive_poses[0].orientation.w =1.0;
	o2.primitives.resize(1);
	o2.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o2.primitives[0].dimensions.resize(2);
	o2.primitives[0].dimensions[0]= 0.23;
	o2.primitives[0].dimensions[1]= 0.03;
	psi.applyCollisionObject(o2);

	moveit_msgs::CollisionObject table;
	table.id= "table";
	table.header.frame_id= "base_footprint";
	table.primitive_poses.resize(1);
	table.primitive_poses[0].position.x = 0.53;
	table.primitive_poses[0].position.y = 0.05;
	table.primitive_poses[0].position.z = (0.84-(0.23/2))/2;
	table.primitive_poses[0].orientation.x =0.0;
	table.primitive_poses[0].orientation.y =0.0;
	table.primitive_poses[0].orientation.z =0.0;
	table.primitive_poses[0].orientation.w =1.0;
	table.primitives.resize(1);
	table.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	table.primitives[0].dimensions.resize(3);
	table.primitives[0].dimensions[0]= 0.3;
	table.primitives[0].dimensions[1]= 1.0;
	table.primitives[0].dimensions[2]= 0.84-(0.23/2);
	psi.applyCollisionObject(table);
}

void planPick(Task &t) {
	spawnObject();

	t.loadRobotModel();

	std::string side = "left";
	std::string tool_frame = side.substr(0,1) + "_gripper_tool_frame";
	std::string eef = side + "_gripper";
	std::string arm = side + "_arm";

	Stage* initial_stage = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	initial_stage = initial.get();
	t.add(std::move(initial));

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{eef, pipeline}, {arm, pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));
  }
	{
		// grasp generator
		auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
		//grasp_generator->setAngleDelta(M_PI / 12);
		grasp_generator->setPreGraspPose(side + "_open");
		grasp_generator->setGraspPose(side +"_close");
		grasp_generator->setMonitoredStage(initial_stage);

		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
		grasp->setIKFrame(Eigen::Isometry3d::Identity(), tool_frame);


		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp));
		pick->setProperty("eef", eef);
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "object";
		approach.twist.linear.x = -1.0;
		pick->setApproachMotion(approach, 0.03, 0.1);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "base_footprint";
		lift.twist.linear.z = 1.0;
		pick->setLiftMotion(lift, 0.03, 0.05);

		t.add(std::move(pick));
	}

	//t.plan();
}

void planTest(Task &t) {
	spawnObject();

	t.loadRobotModel();

	std::string tool_frame = "l_gripper_tool_frame";
	std::string eef = "left_gripper";
	std::string arm = "left_arm";


	auto cartesian = std::make_shared<solvers::CartesianPath>();
	cartesian->setProperty("jump_threshold", 0.0);

	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	t.add(std::move(initial));

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");

	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{eef, pipeline}, {arm, pipeline}};
	auto connect = std::make_unique<stages::Connect>("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);
	t.add(std::move(connect));


	{
		// grasp generator
		auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose left");
		grasp_generator->setTopGraspEnable(true);
		grasp_generator->setAngleDelta(.2);
		grasp_generator->setPreGraspPose("left_open");
		grasp_generator->setGraspPose("left_close");
		grasp_generator->setMonitoredStage(current_state);

		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
		grasp->setIKFrame(Eigen::Isometry3d::Identity(), tool_frame);
		grasp->setMaxIKSolutions(10);


		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp));
		pick->setProperty("eef", eef);
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "object";
		approach.twist.linear.z = 1.0;
		pick->setApproachMotion(approach, 0.03, 0.1);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "base_footprint";
		lift.twist.linear.z =  1.0;
		pick->setLiftMotion(lift, 0.03, 0.20);

		t.add(std::move(pick));
	}

	{
		auto handover = std::make_unique<stages::MoveTo>("move to handover", cartesian);
		handover->setProperty("group", "right_arm");
		handover->setProperty("eef", "right_gripper");

		// TODO: specify that attached object should move to a specific location
		geometry_msgs::PoseStamped target;
		target.header.frame_id = "base_footprint";
		target.pose.position.x =  0.583;
		target.pose.position.y = -0.228;
		target.pose.position.z =  0.989;
		target.pose.orientation.x = 0.725;
		target.pose.orientation.y = 0.675;
		target.pose.orientation.z = 0.096;
		target.pose.orientation.w = 0.098;
		handover->setGoal(target);
		current_state = handover.get();
		t.add(std::move(handover));
	}

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{"right_gripper", pipeline}, {"right_arm", pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));
	}

	{
		// grasp generator
		auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose right");
		grasp_generator->setAngleDelta(.2);
		grasp_generator->setPreGraspPose("right_open");
		grasp_generator->setGraspPose("right_close");
		grasp_generator->setProperty("object", std::string("object"));
		grasp_generator->setMonitoredStage(current_state);
		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
		grasp->setIKFrame(Eigen::Isometry3d::Identity(), "r_gripper_tool_frame");
		grasp->setMaxIKSolutions(8);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp));
		pick->setProperty("eef", "right_gripper");
		pick->setProperty("group","right_arm");
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "r_gripper_tool_frame";
		approach.twist.linear.y = 1.0;
		pick->setApproachMotion(approach, 0.03, 0.2);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "base_footprint";
		lift.twist.linear.x =  0.0;
		lift.twist.linear.y =  0.0;
		lift.twist.linear.z =  0.0;
		pick->setLiftMotion(lift, 0.00, 0.00);

		t.add(std::move(pick));
	}


	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("Detach object");
		stage->detachObject("object", "l_gripper_tool_frame");
		t.add(std::move(stage));
	}

	{
			auto stage = std::make_unique<stages::MoveTo>("open left hand", pipeline);
			stage->setProperty("eef", "left_gripper");
			stage->setProperty("group", "left_gripper");
			stage->setGoal("left_open");
			t.add(std::move(stage));
		}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("Attach object");
		stage->attachObject("object", "r_gripper_tool_frame");
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("move to home right", pipeline);
		stage->setProperty("group", "right_arm");
		stage->setProperty("eef", "right_gripper");
		stage->setGoal("RIGHT_ARM_INITIAL_POSE");
		current_state = stage.get();
		t.add(std::move(stage));
	}

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{"left_gripper", pipeline}, {"left_arm", pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));
	}

	{
		// grasp generator
		auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose left 2");
		grasp_generator->setAngleDelta(.2);
		grasp_generator->setPreGraspPose("left_open");
		grasp_generator->setGraspPose("left_close");
		grasp_generator->setProperty("object2", std::string("object2"));
		grasp_generator->setMonitoredStage(current_state);
		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
		grasp->setIKFrame(Eigen::Isometry3d::Identity(), "l_gripper_tool_frame");
		grasp->setMaxIKSolutions(8);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp));
		pick->setProperty("eef", "left_gripper");
		pick->setProperty("group","left_arm");
		pick->setProperty("object2", std::string("object2"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "l_gripper_tool_frame";
		approach.twist.linear.y = 1.0;
		pick->setApproachMotion(approach, 0.03, 0.2);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "base_footprint";
		lift.twist.linear.x =  0.0;
		lift.twist.linear.y =  0.0;
		lift.twist.linear.z =  0.0;
		pick->setLiftMotion(lift, 0.00, 0.00);

		t.add(std::move(pick));
	}



  std::cerr << t << std::endl;
	t.plan(10);
	t.enableIntrospection();
}

void execute(Task &t)
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

int main(int argc, char** argv){
	ros::init(argc, argv, "pr2");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	Task t("left");
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
