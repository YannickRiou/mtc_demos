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

  t.setProperty("group","left_arm");
  t.setProperty("eef","left_gripper");

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
			// Sample grasp pose
			auto stage = std::make_unique<stages::GeneratePose>("GeneratePickPose");
			geometry_msgs::PoseStamped p;
			p.header.frame_id= "object";
			p.pose.position.x=  -0.05;
			p.pose.position.y=  0;
			p.pose.position.z=  0;
			stage->setProperty("pregrasp","left_open");
			stage->setProperty("grasp","left_close");
			stage->setProperty("object","object");
			stage->setProperty("eef","left_gripper");
			stage->setProperty("group","left_arm");
			stage->setPose(p);
			stage->setMonitoredStage(current_state);  // Hook into current state

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(Eigen::Affine3d::Identity(), "l_gripper_tool_frame");
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			t.add(std::move(wrapper));
		}

		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
			stage->properties().set("link", "l_gripper_tool_frame");
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01,0.10);

			// Set hand forward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "object";
			vec.vector.x = 1.0;
			stage->setDirection(vec);
		  t.add(std::move(stage));
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



int main(int argc, char** argv)
{

	ros::init(argc, argv, "pr2");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	const std::string RIGHT_ARM_PLANNING_GROUP = "right_arm";
	const std::string
	LEFT_ARM_PLANNING_GROUP = "left_arm";

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
	left_arm_move_group.setNamedTarget("LEFT_ARM_INITIAL_POSE");

	// Move the arms to initial pose (away from robot vision)
	right_arm_move_group.move();
	left_arm_move_group.move();

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

	/*geometry_msgs::Pose target_pose3 = left_arm_move_group.getCurrentPose().pose;

	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(target_pose3);

	//target_pose3.position.z += 0.2;
	//waypoints.push_back(target_pose3);

	target_pose3.position.y -= 0.4;
	target_pose3.position.x += 0.25;
	waypoints.push_back(target_pose3);

	moveit_msgs::RobotTrajectory trajectory;

	double fraction = left_arm_move_group.computeCartesianPath(waypoints, 0.01, 0.00, trajectory);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	my_plan.trajectory_= trajectory;
	left_arm_move_group.execute(my_plan);*/

	return 0;
}
