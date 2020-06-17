#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
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
	o.id= "bar";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.6;
	o.primitive_poses[0].position.y = 0.25;
	o.primitive_poses[0].position.z = 0.75+(0.30/2)+0.02;
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.05;
	o.primitives[0].dimensions[1]= 0.05;
	o.primitives[0].dimensions[2]= 0.30;
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

	auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	t.add(std::move(initial));

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");

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
		grasp_generator->setProperty("object", std::string("bar"));
		grasp_generator->setMonitoredStage(current_state);
		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
		Eigen::Affine3d tr = Eigen::Affine3d::Identity();
		//tr.translation() = Eigen::Vector3d(0.0,0.0,0.0);
		grasp->setIKFrame(tr, "l_gripper_tool_frame");
		grasp->setMaxIKSolutions(100);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp));
		pick->setProperty("eef", "left_gripper");
		pick->setProperty("group","left_arm");
		pick->setProperty("object", std::string("bar"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "l_gripper_tool_frame";
		approach.twist.linear.x = 1.0;
		pick->setApproachMotion(approach, 0.01, 0.10);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "base_footprint";
		lift.twist.linear.x =  0.0;
		lift.twist.linear.y =  0.0;
		lift.twist.linear.z =  0.1;
		pick->setLiftMotion(lift, 0.00, 0.00);
		current_state = pick.get();
		t.add(std::move(pick));
	}

	{
		// connect to place
		stages::Connect::GroupPlannerVector planners = {{"left_arm", pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("pose to handover");
		geometry_msgs::PoseStamped p;
		p.header.frame_id= "base_footprint";
		p.pose.position.x=  0.6;
		p.pose.position.y=  0;
		p.pose.position.z=  0.75+(0.30/2)+0.02+0.15;
		stage->setPose(p);
		stage->properties().configureInitFrom(Stage::PARENT);

		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("to handover", std::move(stage) );
		wrapper->setMaxIKSolutions(32);
		wrapper->setIKFrame(Eigen::Affine3d::Identity(),"l_gripper_tool_frame");
		wrapper->setProperty("group","left_arm");
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		t.add(std::move(wrapper));
	}


	 {
		auto stage = std::make_unique<stages::ModifyPlanningScene>("Detach object");
		stage->detachObject("bar", "l_gripper_tool_frame");
		current_state = stage.get();
		t.add(std::move(stage));
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
		grasp_generator->setAngleDelta(90);
		grasp_generator->setPreGraspPose("right_open");
		grasp_generator->setGraspPose("right_close");
		grasp_generator->setProperty("object", std::string("bar"));
		grasp_generator->setMonitoredStage(current_state);
		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
		//grasp->setIKFrame(Eigen::Affine3d::Identity(), "r_gripper_tool_frame");
	  Eigen::Affine3d tr = Eigen::Affine3d::Identity();
		tr.translation() = Eigen::Vector3d(0.0,0.0,-0.1);
		grasp->setIKFrame(tr, "r_gripper_tool_frame");
		grasp->setMaxIKSolutions(100);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp));
		pick->setProperty("eef", "right_gripper");
		pick->setProperty("group","right_arm");
		pick->setProperty("object", std::string("bar"));
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
		auto stage = std::make_unique<stages::MoveTo>("open left hand", pipeline);
		stage->setProperty("eef", "left_gripper");
		stage->setProperty("group", "left_gripper");
		stage->setGoal("left_open");
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("move to home right", cartesian);
		stage->setProperty("group", "right_arm");
		stage->setProperty("eef", "right_gripper");
		stage->setGoal("RIGHT_ARM_INITIAL_POSE");
		t.add(std::move(stage));
	}


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
		ros::Duration(1.5).sleep(); // sleep for half a seco
		ac.cancelGoal();
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
