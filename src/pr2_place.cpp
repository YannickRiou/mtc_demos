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
	o.primitive_poses[0].position.x = 0.53;
	o.primitive_poses[0].position.y = 0.05;
	o.primitive_poses[0].position.z = 0.75+(0.23/2);
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

/*void planPick(Task &t) {
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
		grasp->setIKFrame(Eigen::Affine3d::Identity(), tool_frame);


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
}*/

void planTest(Task &t) {

	t.loadRobotModel();
	t.setProperty("object","object");

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
		grasp_generator->setAngleDelta(M_PI/4);
		grasp_generator->setPreGraspPose("left_open");
		grasp_generator->setGraspPose("left_close");
		grasp_generator->setMonitoredStage(current_state);

		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
		grasp->setIKFrame(Eigen::Affine3d::Identity(), "l_gripper_tool_frame");
		grasp->setMaxIKSolutions(10);


		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp));
		pick->setProperty("eef", eef);
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "l_wrist_roll_link";
		approach.twist.linear.x = 1.0;
		pick->setApproachMotion(approach, 0.01, 0.10);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "base_footprint";
		lift.twist.linear.z =  1.0;
		pick->setLiftMotion(lift, 0.01, 0.20);
		current_state = pick.get();
		t.add(std::move(pick));
	}

/* 	{
			auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian);
			stage->properties().set("marker_ns", "lower_object");
			stage->properties().set("link", "r_gripper_tool_frame");
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(.03, .13);
			// Set downward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "base_footprint";
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			t.add(std::move(stage));
	} */

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
		p.pose.position.x=  0.53;
		p.pose.position.y=  -0.1;
		p.pose.position.z=  0.75+(0.23/2)+0.05;
		stage->setPose(p);
		stage->setObject("object");
		stage->setProperty("eef", "left_gripper");
		stage->setProperty("group","left_arm");
		stage->setProperty("augment_rotations",true);

		geometry_msgs::PoseStamped ik;
		ik.header.frame_id= "l_gripper_tool_frame";
		ik.pose.position.x=  0;
		ik.pose.position.y=  0;
		ik.pose.position.z=  0;
		stage->setProperty("ik_frame",ik);
		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("place pose kinematics", std::move(stage));
		wrapper->setMaxIKSolutions(32);
		wrapper->setIKFrame(Eigen::Affine3d::Identity(),"l_gripper_tool_frame");
		wrapper->setProperty("eef", "left_gripper");
		wrapper->setProperty("group","left_arm");
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		t.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("release object", pipeline);
		stage->setProperty("eef", "left_gripper");
		stage->setProperty("group","left_gripper");
		stage->setGoal("left_open");
		t.add(std::move(stage));
	}
/*
	{
		auto handover = std::make_unique<stages::MoveTo>("move to handover", cartesian);
		handover->setProperty("group", "right_arm");
		handover->setProperty("eef", "right_gripper");

		// TODO: specify that attached object should move to a specific location
		geometry_msgs::PoseStamped target;
		target.header.frame_id = "base_footprint";
		target.pose.position.x =  0.137;
		target.pose.position.y = -0.695;
		target.pose.position.z =  1.309;
		target.pose.orientation.x = 0.957;
		target.pose.orientation.y = 0.058;
		target.pose.orientation.z = 0.281;
		target.pose.orientation.w = 0.054;
		handover->setGoal(target);
		current_state = handover.get();
		t.add(std::move(handover));
	}
*/
/*
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
		grasp->setIKFrame(Eigen::Affine3d::Identity(), "r_gripper_tool_frame");
		grasp->setMaxIKSolutions(10);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp));
		pick->setProperty("eef", "right_gripper");
		pick->setProperty("group","right_arm");
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "r_gripper_tool_frame";
		approach.twist.linear.x = 1.0;
		pick->setApproachMotion(approach, 0.01, 0.10);

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
		t.add(std::move(stage));
	}

*/
  std::cerr << t << std::endl;
	t.plan(10);
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

void markerCallback(const visualization_msgs::MarkerConstPtr& marker, moveit::planning_interface::PlanningSceneInterface& plan_scene,std::vector<moveit_msgs::CollisionObject>& collision_object_vector)
{
  // Add table as a fixed collision object.
  moveit_msgs::CollisionObject collisionObj;
  collisionObj.id = marker->text;

  collisionObj.header.frame_id = "base_footprint";
  shape_msgs::SolidPrimitive collisionObj_primitive;

  // Check the type of the marker. We search for CUBE and CYCLINDER only
  if(marker->type == visualization_msgs::Marker::CUBE)
  {
    collisionObj_primitive.type = collisionObj_primitive.BOX;
    collisionObj_primitive.dimensions.resize(3);
    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = marker->scale.x;
    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = marker->scale.y;
    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = marker->scale.z;
  }
  else if(marker->type == visualization_msgs::Marker::CYLINDER)
  {
    collisionObj_primitive.type = collisionObj_primitive.CYLINDER;
    collisionObj_primitive.dimensions.resize(2);
    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] =marker->scale.y /2.0;
    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = marker->scale.z;


    std::cout << "Object Size is :" << std::endl;
    std::cout << "Radius :" << marker->scale.x/2.0 << std::endl;
    std::cout << "Height :" <<  marker->scale.z << std::endl;
  }
  else
  {
    return;
  }

  // Define a pose for the object (specified relative to frame_id)
  geometry_msgs::Pose collisionObj_pose;
  collisionObj_pose.position.x = marker->pose.position.x;
  collisionObj_pose.position.y = marker->pose.position.y;
  collisionObj_pose.position.z = marker->pose.position.z;

  collisionObj_pose.orientation.x = marker->pose.orientation.x;
  collisionObj_pose.orientation.y = marker->pose.orientation.y;
  collisionObj_pose.orientation.z = marker->pose.orientation.z;
  collisionObj_pose.orientation.w = marker->pose.orientation.w;

  collisionObj.primitives.push_back(collisionObj_primitive);
  collisionObj.primitive_poses.push_back(collisionObj_pose);
  collisionObj.operation = collisionObj.ADD;

  collision_object_vector.push_back(collisionObj);

  // Now, let's add the collision object into the world
  // Little sleep necessary before adding it
  ros::Duration(0.2).sleep();
  // Add the remaining collision object
  plan_scene.addCollisionObjects(collision_object_vector);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "pr2");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	// Store the planning scene where the collision object will be add
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// Store the collision objects added to the scene
	std::vector<moveit_msgs::CollisionObject> collision_objects_vector;

	spawnObject(planning_scene_interface);


  // Subscribe to topic giving the marker (RoboSherlock) to show the object as boxes in Rviz
  ros::Subscriber marker_sub = nh.subscribe<visualization_msgs::Marker>("/visualization_marker", 1000, boost::bind(&markerCallback,_1,boost::ref(planning_scene_interface),boost::ref(collision_objects_vector)));
	ros::Duration(5).sleep();
	marker_sub.shutdown();

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
