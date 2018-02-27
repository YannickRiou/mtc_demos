#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>


using namespace moveit::task_constructor;

void spawnObject(bool right) {
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = right ? -0.3 : 0.3;
	o.primitive_poses[0].position.y = 0.23;
	o.primitive_poses[0].position.z = 0.12;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.23;
	o.primitives[0].dimensions[1] = 0.03;
	psi.applyCollisionObject(o);
}

void fill(ParallelContainerBase &container, Stage* initial_stage, bool right_side) {
	std::string side = right_side ? "right" : "left";
	std::string tool_frame = side.substr(0,1) + "h_tool_frame";
	std::string eef = side.substr(0,1) + "a_tool_mount";
	std::string group = side + "_arm";

	auto grasp_generator = std::make_unique<stages::SimpleGrasp>();

	if (right_side)
		grasp_generator->setToolToGraspTF(Eigen::Translation3d(0,0,.05)*
		                                  Eigen::AngleAxisd(+0.5*M_PI, Eigen::Vector3d::UnitY()),
		                                  tool_frame);
	else
		grasp_generator->setToolToGraspTF(Eigen::Translation3d(0,0,.05)*
		                                  Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY()),
		                                  tool_frame);

	grasp_generator->setAngleDelta(.2);
	grasp_generator->setPreGraspPose("open");
	grasp_generator->setGraspPose("closed");
	grasp_generator->setMonitoredStage(initial_stage);

	auto pick = std::make_unique<stages::Pick>(std::move(grasp_generator), side);
	pick->setProperty("eef", eef);
	pick->setProperty("object", std::string("object"));
	geometry_msgs::TwistStamped approach;
	approach.header.frame_id = tool_frame;
	approach.twist.linear.z = 1.0;
	pick->setApproachMotion(approach, 0.05, 0.1);

	geometry_msgs::TwistStamped lift;
	lift.header.frame_id = "frame";
	lift.twist.linear.z = 1.0;
	pick->setLiftMotion(lift, 0.03, 0.05);

	auto move = std::make_unique<stages::MoveRelative>("twist object",
	                                                   std::make_shared<solvers::CartesianPath>());
	move->properties().set("group", group);
	move->setMinMaxDistance(0.1, 0.2);
	move->properties().set("marker_ns", std::string("lift"));
	move->properties().set("link", tool_frame);

	geometry_msgs::TwistStamped twist;
	twist.header.frame_id = "object";
	twist.twist.linear.y = 1;
	twist.twist.angular.y = 2;
	move->along(twist);
	pick->insert(std::move(move));

	container.insert(std::move(pick));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "bimodal");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	Task t;

	Stage* initial_stage = nullptr;
	auto initial = std::make_unique<stages::CurrentState>();
	initial_stage = initial.get();
	t.add(std::move(initial));

	auto parallel = std::make_unique<Alternatives>();
	fill(*parallel, initial_stage, true);
	fill(*parallel, initial_stage, false);

	t.add(std::move(parallel));

	try {
		char ch;
		spawnObject(true);
		t.plan();

		std::cout << "waiting for any key + <enter>\n";
		std::cin >> ch;

		spawnObject(false);
		t.plan();
		std::cout << "waiting for any key + <enter>\n";
		std::cin >> ch;
	}
	catch (const InitStageException &e) {
		std::cerr << e;
		return EINVAL;
	}

	return 0;
}
