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

#include "../stages/bimanual_grasp_pose.h"
#include <eigen_conversions/eigen_msg.h>

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace moveit::task_constructor;

// Objects used in the planning scene
const std::string obj_bar = "bar";
const std::string two_col_support = "two_col_support";

void addCubeShape(moveit_msgs::CollisionObject& co, const geometry_msgs::Pose p, const std::vector<double> size)
{
    // Create the cubic shape
    shape_msgs::SolidPrimitive shape;
    shape.type = shape_msgs::SolidPrimitive::BOX;
    shape.dimensions.resize(3, 0.1);
    for(size_t i=0; i<size.size(); i++)
        shape.dimensions[i] = size[i];
    co.primitives.push_back(shape);

    // Set the pose
    co.primitive_poses.push_back(p);
}

moveit_msgs::CollisionObject createCube(const std::string id, const geometry_msgs::PoseStamped p, const std::vector<double> size)
{
    moveit_msgs::CollisionObject co;
    co.id = id;
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.header.frame_id = p.header.frame_id;

    addCubeShape(co, p.pose, size);

    return co;
}



void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi) {

	// To hold information about a collision object
	moveit_msgs::CollisionObject co;

	// Create a two columns support and add it in the planning scene request
	geometry_msgs::PoseStamped p;
	double column_heigh = 0.8;
	double column_side = 0.5;
	std::vector<double> dimensions = {column_side, column_side, column_heigh};
	p.header.frame_id = "base_footprint";
	p.pose.position.x = 0.75;
	p.pose.position.y = 0.7;
	p.pose.position.z = column_heigh/2;
	p.pose.orientation.w = 1.;
	co = createCube(two_col_support, p, dimensions);
	p.pose.position.y *= -1;
	addCubeShape(co, p.pose, dimensions);
	//psi.applyCollisionObject(co);

	// Create a bar to manipulate and add it in the planning scene request
	double bar_radius = 0.03;
	p.pose.position.y = 0.;
	p.pose.position.z = column_heigh + bar_radius;
	dimensions = {bar_radius, 1., bar_radius};
	co = createCube(obj_bar, p, dimensions);
	psi.applyCollisionObject(co);

}

void planTest(Task &t) {

	t.loadRobotModel();

	std::string object = obj_bar;

	std::string eef_left = "left_gripper";
	std::string eef_right = "right_gripper";

	std::string arm_left = "left_arm";
	std::string arm_right = "right_arm";

	geometry_msgs::PoseStamped ik_right;
	ik_right.header.frame_id = "r_gripper_tool_frame";
	tf::poseEigenToMsg(Eigen::Translation3d(-0.05, 0.0, 0) *
	                   Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX()),
	                   ik_right.pose);


	 geometry_msgs::PoseStamped ik_left;
	 ik_left.header.frame_id = "l_gripper_tool_frame";
	 tf::poseEigenToMsg(Eigen::Translation3d(-0.05, 0.0, 0.0) *
		                  Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()),
		                  ik_left.pose);


	// cartesian planner
	auto cartesian = std::make_shared<solvers::CartesianPath>();

	// pipeline planner
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");

	Stage* referenced_stage = nullptr;
	{  // fetch initial state from move_group
		auto initial = new stages::CurrentState("current state");
		t.add(std::unique_ptr<Stage>(referenced_stage = initial));
	}

	{  // connect current state to pick
		stages::Connect::GroupPlannerVector planners = {{eef_left, pipeline}, {arm_left, pipeline},
		                                                {eef_right, pipeline}, {arm_right, pipeline}};
		auto connect = new stages::Connect("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::unique_ptr<Stage>(connect));
	}

	auto pick = new SerialContainer("pick");

	{  // approach
		geometry_msgs::TwistStamped twist;
		twist.twist.linear.x = 1.0;

		auto merger = new Merger("approach");
		for (const auto& eef : {eef_left, eef_right}) {
			auto move = new stages::MoveRelative("approach " + eef, cartesian);
			move->restrictDirection(stages::MoveRelative::BACKWARD);
			move->setProperty("marker_ns", std::string("approach"));
			const moveit::core::JointModelGroup* eef_jmg = t.getRobotModel()->getEndEffector(eef);
			const auto& group_link = eef_jmg->getEndEffectorParentGroup();
			move->setGroup(group_link.first);
			move->setIKFrame(group_link.second);
			twist.header.frame_id = group_link.second;
			ROS_WARN_STREAM("FRAME_ID FOR TWIST : " <<  group_link.second );
			move->setDirection(twist);
			move->setMinMaxDistance(0.05, 0.10);
			merger->insert(std::unique_ptr<Stage>(move));
		}
		pick->insert(std::unique_ptr<Stage>(merger));
	}

	{  // bimanual grasp generator
		auto gengrasp =  new stages::BimanualGraspPose("Bimanual_grasp");
		gengrasp->setMonitoredStage(referenced_stage);
		gengrasp->setObject(object);
		gengrasp->setEndEffectorPoses({{eef_left, "left_open"}, {eef_right, "right_open"}});

		// inner IK: right hand
		auto ik_inner = new stages::ComputeIK("compute ik right", std::unique_ptr<Stage>(gengrasp));
		ik_inner->setEndEffector(eef_right);
		ik_inner->properties().property("target_pose").configureInitFrom(Stage::INTERFACE, "target_pose_right");
		ik_inner->setIKFrame(ik_right);
		ik_inner->setForwardedProperties({"target_pose_left", "target_pose_right"});

		// outer IK: left hand
		auto ik_outer = new stages::ComputeIK("compute ik left", std::unique_ptr<Stage>(ik_inner));
		ik_outer->setEndEffector(eef_left);
		ik_outer->properties().property("target_pose").configureInitFrom(Stage::INTERFACE, "target_pose_left");
		ik_outer->setIKFrame(ik_left);

		pick->insert(std::unique_ptr<Stage>(ik_outer));
	}

	{  // allow touching the object
		auto allow_touch = new stages::ModifyPlanningScene("allow object collision");
		allow_touch->allowCollisions("bar", t.getRobotModel()->getJointModelGroup(eef_left)->getLinkModelNamesWithCollisionGeometry(), true);
		allow_touch->allowCollisions("bar", t.getRobotModel()->getJointModelGroup(eef_right)->getLinkModelNamesWithCollisionGeometry(), true);
		pick->insert(std::unique_ptr<Stage>(allow_touch));
	}

	{  // close grippers
		auto merger = new Merger("close grippers");
		for (const auto& eef : {eef_left, eef_right}) {
			auto move = new stages::MoveTo("close " + eef, pipeline);
			move->restrictDirection(stages::MoveTo::FORWARD);
			move->setGroup(t.getRobotModel()->getEndEffector(eef)->getName());
			if(eef == "right_gripper") move->setGoal("right_close");
			if(eef == "left_gripper") move->setGoal("left_close");
			merger->insert(std::unique_ptr<Stage>(move));
		}
		pick->insert(std::unique_ptr<Stage>(merger));
	}

	{  // attach object
			auto attach = new stages::ModifyPlanningScene("attach object");
			attach->attachObject("bar", "l_gripper_tool_frame");
			pick->insert(std::unique_ptr<Stage>(attach));
	}

	{  // lift
		geometry_msgs::TwistStamped twist;
		twist.twist.linear.z = 1.0;
		twist.twist.linear.y = 1.0;
		twist.twist.angular.x=0.5*M_PI;
		twist.header.frame_id = "base_footprint";

		auto merger = new Merger("lift");
		for (const auto& eef : {eef_left, eef_right}) {
			auto move = new stages::MoveRelative("lift " + eef, cartesian);
			move->restrictDirection(stages::MoveRelative::FORWARD);
			move->setProperty("marker_ns", std::string("lift"));
			const moveit::core::JointModelGroup* eef_jmg = t.getRobotModel()->getEndEffector(eef);
			const auto& group_link = eef_jmg->getEndEffectorParentGroup();
			move->setGroup(group_link.first);
			move->setIKFrame(group_link.second);
			move->setDirection(twist);
			move->setMinMaxDistance(0.03, 0.05);
			merger->insert(std::unique_ptr<Stage>(move));
		}
		pick->insert(std::unique_ptr<Stage>(merger));
	}

	t.add(std::unique_ptr<Stage>(pick));

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


int main(int argc, char** argv){
	ros::init(argc, argv, "pr2");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	// Store the planning scene where the collision object will be add
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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
