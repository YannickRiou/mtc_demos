#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include<string>

#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace moveit::task_constructor;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

void createPlaceTask(Task &placeTask, const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian, const moveit::core::RobotModelPtr& robotModel, const std::string planGroup, const std::string object, const geometry_msgs::PoseStamped placePose)
{
	placeTask.setRobotModel(robotModel);

	// Property and variable definitions
	std::string eef;
	std::string ungrasp;
	std::string ikFrame;

	placeTask.setProperty("object",object);

	if(planGroup == "left_arm")
	{
		placeTask.setProperty("group",planGroup);
		placeTask.setProperty("eef","left_gripper");
	    eef = "left_gripper";
		ikFrame = "l_gripper_tool_frame";
		ungrasp = "left_open";
	}
	else if(planGroup == "right_arm")
	{
		placeTask.setProperty("group",planGroup);
		placeTask.setProperty("eef","right_gripper");
		eef = "right_gripper";
		ikFrame = "r_gripper_tool_frame";
		ungrasp = "right_open";
	}


	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	placeTask.add(std::move(initial));

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("Forbid collision (object,support)");
		stage->allowCollisions({ object }, "tableLaas", false);
		placeTask.add(std::move(stage));
	}

	{
		// connect to place
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		placeTask.add(std::move(connect));
	}


	{
		auto stage = std::make_unique<stages::GeneratePlacePose>("place pose");
		geometry_msgs::PoseStamped p;
		p.header.frame_id= "base_footprint";
		stage->setPose(placePose);
		stage->setObject(object);
		placeTask.properties().exposeTo(stage->properties(), { "eef", "group"});

		geometry_msgs::PoseStamped ik;
		ik.header.frame_id= ikFrame;
		ik.pose.position.x=  0;
		ik.pose.position.y=  0;
		ik.pose.position.z=  0;
		stage->setProperty("ik_frame",ik);
		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("place pose kinematics", std::move(stage));
		wrapper->setMaxIKSolutions(32);
		wrapper->setIKFrame(Eigen::Isometry3d::Identity(),ikFrame);
		wrapper->setProperty("eef", eef);
		wrapper->setProperty("group",planGroup);
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		placeTask.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("release object", pipeline);
		stage->setGroup(eef);
		stage->setGoal(std::string(ungrasp));
		placeTask.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
		stage->detachObject(object, ikFrame);
		placeTask.add(std::move(stage));
	}
}

void createMoveTask(Task &moveTask,const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian,const moveit::core::RobotModelPtr& robotModel,const std::string planGroup, const geometry_msgs::PoseStamped moveToPose)
{
	moveTask.setRobotModel(robotModel);
	std::string ikFrame;
		std::string eef;
	if(planGroup == "left_arm")
	{
		ikFrame = "l_gripper_tool_frame";
		eef = "left_gripper";
	}
	else if(planGroup == "right_arm")
	{
		ikFrame = "r_gripper_tool_frame";
		eef = "right_gripper";
	}

	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	moveTask.add(std::move(initial));

	{
		// connect
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipeline},{eef, pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		moveTask.add(std::move(connect));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("go to pose");
		stage->setProperty("group",planGroup);
		stage->setProperty("eef",eef);
		stage->setPose(moveToPose);
		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage) );
		wrapper->setMaxIKSolutions(32);
		wrapper->setIKFrame(ikFrame);
		wrapper->setProperty("group",planGroup);
		wrapper->setProperty("eef",eef);
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		moveTask.add(std::move(wrapper));
	}
}

void createPickTaskCustom(Task &pickTask,const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian,const moveit::core::RobotModelPtr& robotModel,const std::string planGroup,const std::string object, const geometry_msgs::PoseStamped graspPose)
{
	pickTask.setRobotModel(robotModel);

	// Property and variable definitions
	std::string eef;
	std::string ikFrame;
	std::string pregrasp;
	std::string postgrasp;

	pickTask.setProperty("object",object);

	if(planGroup == "left_arm")
	{
		pickTask.setProperty("group",planGroup);
		pickTask.setProperty("eef","left_gripper");
	    eef = "left_gripper";
		pregrasp = "left_open";
		postgrasp = "left_close";
		ikFrame = "l_gripper_tool_frame";
	}
	else if(planGroup == "right_arm")
	{
		pickTask.setProperty("group",planGroup);
		pickTask.setProperty("eef","right_gripper");
		eef = "right_gripper";
		pregrasp = "right_open";
		postgrasp = "right_close";
		ikFrame = "r_gripper_tool_frame";
	}


	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	pickTask.add(std::move(initial));


	// ---------------------- open Hand ---------------------- //
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand", pipeline);
		stage->setGroup(eef);
		stage->setGoal(pregrasp);
		current_state = stage.get();
		pickTask.add(std::move(stage));
	}

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{planGroup, pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->setTimeout(20.0);
		connect->properties().configureInitFrom(Stage::PARENT);
		pickTask.add(std::move(connect));
	}

	{
		auto grasp = std::make_unique<SerialContainer>("pick object");

		pickTask.properties().exposeTo(grasp->properties(), { "eef", "group"});
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "group"});

		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.15);
			stage->setIKFrame(ikFrame);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = object;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		{
			auto stage = std::make_unique<stages::GeneratePose>("go to grasp pose");
			stage->setPose(graspPose);
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setMonitoredStage(current_state);

			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage) );
			wrapper->setMaxIKSolutions(32);
			wrapper->setIKFrame(ikFrame);
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"}); // TODO: convenience wrapper
			grasp->insert(std::move(wrapper));
		}

		// ---------------------- Allow Collision (hand object) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(object, pickTask.getRobotModel()->getJointModelGroup(eef)->getLinkModelNamesWithCollisionGeometry(),true);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Close Hand ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", pipeline);
			stage->setGroup(eef);
			stage->setGoal(postgrasp);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Attach Object ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(object, ikFrame);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Allow collision (object support) ---------------------- //
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ object }, {"tableLaas","boite"}, true);
			grasp->insert(std::move(stage));
		}

			// ---------------------- Lift object ---------------------- //
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.01, 0.20);
			stage->setIKFrame(ikFrame);
			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = "base_footprint";
			vec.vector.x = -1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}
		pickTask.add(std::move(grasp));
	}

}
void createPickTask(Task &pickTask,const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian,const moveit::core::RobotModelPtr& robotModel, const std::string planGroup,const std::string object)
{
	pickTask.setRobotModel(robotModel);

	// Property and variable definitions
	std::string eef;
	std::string ikFrame;
	std::string pregrasp;
	std::string grasp;

	pickTask.setProperty("object",object);

	if(planGroup == "left_arm")
	{
		pickTask.setProperty("group",planGroup);
		pregrasp = "left_open";
		grasp = "left_close";
		pickTask.setProperty("eef","left_gripper");
	  eef = "left_gripper";
		ikFrame = "l_gripper_tool_frame";
	}
	else if(planGroup == "right_arm")
	{
		pickTask.setProperty("group",planGroup);
		pregrasp = "right_open";
		grasp = "right_close";
		pickTask.setProperty("eef","right_gripper");
		eef = "right_gripper";
		ikFrame = "r_gripper_tool_frame";
	}


	//Start state
	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	pickTask.add(std::move(initial));

	{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (arm,support)");
			stage->allowCollisions(
			    "tableLaas",
			    pickTask.getRobotModel()->getJointModelGroup(planGroup)->getLinkModelNamesWithCollisionGeometry(), false);
			pickTask.add(std::move(stage));
	}


	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{eef, pipeline}, {planGroup, pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		pickTask.add(std::move(connect));
	}

	{
		// grasp generator
		auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
		grasp_generator->setAngleDelta(M_PI/2);
		pickTask.properties().exposeTo(grasp_generator->properties(), { "group","eef"});
	  grasp_generator->setPreGraspPose(pregrasp);
		grasp_generator->setGraspPose(grasp);
		grasp_generator->setProperty("object", object);
		grasp_generator->setMonitoredStage(current_state);

		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
		Eigen::Isometry3d tr = Eigen::Isometry3d::Identity();
		pickTask.properties().exposeTo(grasp->properties(), { "group","eef","object"});
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "group","object"});
		grasp->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		//tr.translation() = Eigen::Vector3d(0.0,0.0,0.00);
		grasp->setIKFrame(tr, ikFrame);
		grasp->setMaxIKSolutions(10);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp),"pick");
		pickTask.properties().exposeTo(pick->properties(), { "group","eef","object" });
		pick->properties().configureInitFrom(Stage::PARENT, { "eef", "group","object"});
		//pick->setProperty("eef", "left_gripper");
		//pick->setProperty("group","left_arm");
		//pick->setProperty("object", "obj_0");
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = ikFrame;
		approach.twist.linear.x = 1.0;
		pick->setApproachMotion(approach, 0.10, 0.15);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "base_footprint";
		lift.twist.linear.x =  0.0;
		lift.twist.linear.y =  0.0;
		lift.twist.linear.z =  1.0;
		pick->setLiftMotion(lift, 0.05, 0.10);
		current_state = pick.get();
		pickTask.add(std::move(pick));
	}
}



void planTest(Task &t,const solvers::PipelinePlannerPtr& pipeline, const solvers::CartesianPathPtr& cartesian,const moveit::core::RobotModelPtr& robotModel)
 {
	t.setRobotModel(robotModel);

	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	t.add(std::move(initial));

	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{"left_gripper", pipeline}, {"left_arm", cartesian}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));
	}

	{
		// grasp generator
		auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose left");
		grasp_generator->setPreGraspPose("left_open");
		grasp_generator->setGraspPose("left_close");
		grasp_generator->setProperty("object", "obj_0");
		grasp_generator->setMonitoredStage(current_state);
		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
		Eigen::Isometry3d tr = Eigen::Isometry3d::Identity();
		//tr.translation() = Eigen::Vector3d(0.05,0.0,0.0);
		grasp->setIKFrame(tr, "l_gripper_tool_frame");
		grasp->setMaxIKSolutions(32);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp),"pick left");
		pick->setProperty("eef", "left_gripper");
		pick->setProperty("group","left_arm");
		pick->setProperty("object", "obj_0");
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "l_gripper_tool_frame";
		approach.twist.linear.x = 1.0;
		pick->setApproachMotion(approach, 0.01, 0.10);

		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
		stage->allowCollisions({ "obj_1" }, "tableLaas", true);
		pick->add(std::move(stage));

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "base_footprint";
		lift.twist.linear.x =  0.0;
		lift.twist.linear.y =  0.0;
		lift.twist.linear.z =  1.0;
		pick->setLiftMotion(lift, 0.05, 0.10);
		current_state = pick.get();
		t.add(std::move(pick));
	}

	/******************************************************
---- *          Retreat Motion                            *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("hand to hooman", cartesian);
		stage->setProperty("group", "left_arm");
		stage->setProperty("eef", "left_gripper");
		stage->setMinMaxDistance(.01, .10);
		stage->setIKFrame(Eigen::Isometry3d::Identity(),"l_gripper_tool_frame");
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = "l_gripper_tool_frame";
		vec.vector.x = 1.0;
		stage->setDirection(vec);
	  t.add(std::move(stage));
	}
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

//Open the gripper
bool gripper_open(GripperClient* gripper)
{
	pr2_controllers_msgs::Pr2GripperCommandGoal open;
  bool success;
  open.command.position = 0.08;
  open.command.max_effort = -1.0;  // Do not limit effort (negative)

  gripper->sendGoal(open);
  gripper->waitForResult();

  if(gripper->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    //ROS_INFO("gripper opened!");
    success = true;
  }
  else
  {
    //ROS_ERROR("gripper failed to open.");
    success = false;
  }

  return success;
}

//Close the gripper
bool gripper_close(GripperClient* gripper, float effort)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;

  bool success;
  squeeze.command.position = 0.0;
  squeeze.command.max_effort = effort;  // Close gently

  gripper->sendGoal(squeeze);
  gripper->waitForResult();
  if(gripper->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Right gripper closed!");
     success = true;
  }
  else
  {
    ROS_ERROR("Right gripper failed to close.");
    success = false;
  }

  return success;
}

void markerCallback(const visualization_msgs::MarkerConstPtr& marker, moveit::planning_interface::PlanningSceneInterface& plan_scene,std::vector<moveit_msgs::CollisionObject>& collision_object_vector, geometry_msgs::TransformStamped& transform)
{
  // Add table as a fixed collision object.
  moveit_msgs::CollisionObject collisionObj;
  geometry_msgs::Pose collisionObj_pose;

  geometry_msgs::PoseStamped colliObjPosetransformed;
  geometry_msgs::PoseStamped colliObjPoseUntransformed;

  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::Mesh* m;

  collisionObj.header.frame_id = "base_footprint";
  shape_msgs::SolidPrimitive collisionObj_primitive;

	// From ar_track_alvar
	if(marker->ns == "main_shapes")
	{

		if(marker->id == 54)
		{
			collisionObj.id = "tableLaas";

			std::string mesh_uri("package://mtc_demos/meshes/ikea_table.dae");
			m = shapes::createMeshFromResource(mesh_uri);
			shapes::constructMsgFromShape(m, mesh_msg);
			mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
			// Add the mesh to the Collision object message
			collisionObj.meshes.push_back(mesh);

			// Define a pose for the object (specified relative to frame_id)
			colliObjPoseUntransformed.pose = marker->pose;

			tf2::doTransform(colliObjPoseUntransformed,colliObjPosetransformed,transform);

			collisionObj_pose = colliObjPosetransformed.pose;
			collisionObj_pose.position.z = collisionObj_pose.position.z - 0.75;

			collisionObj.mesh_poses.push_back(collisionObj_pose);
		}
		else
		{

			// Define a pose for the object (specified relative to frame_id)
			colliObjPoseUntransformed.pose = marker->pose;

			tf2::doTransform(colliObjPoseUntransformed,colliObjPosetransformed,transform);

			collisionObj_pose = colliObjPosetransformed.pose;

			if(marker->id == 33)
			{
				collisionObj.id = "boite";

				std::string mesh_uri("package://mtc_demos/meshes/director_box.dae");
				m = shapes::createMeshFromResource(mesh_uri);
				shapes::constructMsgFromShape(m, mesh_msg);
				mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
				// Add the mesh to the Collision object message
				collisionObj.meshes.push_back(mesh);

				collisionObj_pose.orientation.z = 0.707;
				collisionObj_pose.orientation.w = 0.707;

				collisionObj.mesh_poses.push_back(collisionObj_pose);
			}
			else if(marker->id == 206)
			{
				collisionObj.id = "obj";
				collisionObj_primitive.type = collisionObj_primitive.BOX;
		    collisionObj_primitive.dimensions.resize(3);
		    collisionObj_primitive.dimensions[0] = 0.055;
				collisionObj_primitive.dimensions[1] = 0.055*2;
				collisionObj_primitive.dimensions[2] = 0.055;

				collisionObj.primitives.push_back(collisionObj_primitive);
				collisionObj.primitive_poses.push_back(collisionObj_pose);
			}
		}
	}
	// OBJ is a primitive shape (box or cylinder)
	else
	{
		collisionObj.id = marker->text;
	  // Check the type of the marker. We search for CUBE and CYLINDER only
	  if(marker->type == visualization_msgs::Marker::CUBE)
	  {
	    collisionObj_primitive.type = collisionObj_primitive.BOX;
	    collisionObj_primitive.dimensions.resize(3);
	    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = marker->scale.z;
	  }
	  else if(marker->type == visualization_msgs::Marker::CYLINDER)
	  {
	    collisionObj_primitive.type = collisionObj_primitive.CYLINDER;
	    collisionObj_primitive.dimensions.resize(2);
	    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] =marker->scale.y /2.0;
	    collisionObj_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = marker->scale.z;

			/*
			std::cout << "Object name is :" << collisionObj.id << std::endl;
	    std::cout << "Object Size is :" << std::endl;
	    std::cout << "Radius :" << marker->scale.x/2.0 << std::endl;
	    std::cout << "Height :" <<  marker->scale.z << std::endl;
			*/
	  }
	  else
	  {
	    return;
	  }

	  // Define a pose for the object (specified relative to frame_id)

	  collisionObj_pose.position.x = marker->pose.position.x;
	  collisionObj_pose.position.y = marker->pose.position.y;
	  collisionObj_pose.position.z = marker->pose.position.z+0.02;

	  collisionObj_pose.orientation.x = marker->pose.orientation.x;
	  collisionObj_pose.orientation.y = marker->pose.orientation.y;
	  collisionObj_pose.orientation.z = marker->pose.orientation.z;
	  collisionObj_pose.orientation.w = marker->pose.orientation.w;

		collisionObj.primitives.push_back(collisionObj_primitive);
	  collisionObj.primitive_poses.push_back(collisionObj_pose);
	}

  collisionObj.operation = collisionObj.ADD;

  collision_object_vector.push_back(collisionObj);

  // Now, let's add the collision object into the world
  // Little sleep necessary before adding it
  ros::Duration(0.2).sleep();
  // Add the remaining collision object
  plan_scene.addCollisionObjects(collision_object_vector);
}


void onLeftGripperEvent(const std_msgs::StringConstPtr& msg, std_msgs::StringPtr& left_event)
{
	left_event->data = msg->data;
}


void onLeftArmEffortEvent(const std_msgs::Float32ConstPtr& msg, std_msgs::Float32* left_error)
{
	left_error->data = msg->data;
}

void onLeftArmJointEvent(const std_msgs::Float32ConstPtr& msg, std_msgs::Float32* left_error)
{
	left_error->data = msg->data;
}



int main(int argc, char** argv){
	ros::init(argc, argv, "pr2_task");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	std::string left_obj;
	std::string right_obj;

	std::string choosedPlanner;

	nh.getParam("left", left_obj);
  nh.getParam("right", right_obj);
	nh.getParam("planner", choosedPlanner);

	char ch;

	const std::string RIGHT_GRIPPER_PLANNING_GROUP = "right_gripper";
	const std::string LEFT_GRIPPER_PLANNING_GROUP = "left_gripper";

	const std::string RIGHT_ARM_PLANNING_GROUP = "right_arm";
	const std::string LEFT_ARM_PLANNING_GROUP = "left_arm";

	const robot_state::JointModelGroup* right_gripper_joint_model_group;
	const robot_state::JointModelGroup* left_gripper_joint_model_group;

	const robot_state::JointModelGroup* right_arm_joint_model_group;
	const robot_state::JointModelGroup* left_arm_joint_model_group;

	moveit::planning_interface::MoveGroupInterface right_gripper_move_group(RIGHT_GRIPPER_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface left_gripper_move_group(LEFT_GRIPPER_PLANNING_GROUP);

	moveit::planning_interface::MoveGroupInterface right_arm_move_group(RIGHT_ARM_PLANNING_GROUP);
	moveit::planning_interface::MoveGroupInterface left_arm_move_group(LEFT_ARM_PLANNING_GROUP);

	GripperClient* left_gripper_client_;
	GripperClient* right_gripper_client_;

	right_gripper_joint_model_group =
  right_gripper_move_group.getCurrentState()->getJointModelGroup(RIGHT_GRIPPER_PLANNING_GROUP);
  left_gripper_joint_model_group =
  left_gripper_move_group.getCurrentState()->getJointModelGroup(LEFT_GRIPPER_PLANNING_GROUP);

	//***** Grippers controllers *****//
	//Initialize the client for the Action interface to the gripper controller
	//and tell the action client that we want to spin a thread by default
	left_gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);
	right_gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);

	//wait for the left gripper action server to come up
	while(!left_gripper_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");
	}

	//wait for the right gripper action server to come up
	while(!right_gripper_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
	}



	// Store the planning scene where the collision object will be add
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// Store the collision objects added to the scene
	std::vector<moveit_msgs::CollisionObject> collision_objects_vector;

  // Table transform
	tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tableListenner(tfBuffer);
	geometry_msgs::TransformStamped tableTransform;
	ros::Duration(5).sleep();

	try{
	tableTransform = tfBuffer.lookupTransform("base_footprint","head_mount_kinect2_rgb_optical_frame",ros::Time(0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
	}

	// Subscribe to topic giving the marker (RoboSherlock) to show the object as boxes in Rviz

	ros::Subscriber marker_sub = nh.subscribe<visualization_msgs::Marker>("/visualization_marker", 1000, boost::bind(&markerCallback,_1,boost::ref(planning_scene_interface),boost::ref(collision_objects_vector),boost::ref(tableTransform)));
	ros::Duration(5).sleep();
	marker_sub.shutdown();


	std_msgs::StringPtr left_event(new std_msgs::String);

	std_msgs::Float32 left_arm_effort_error;
	std_msgs::Float32 left_arm_joint_error;


	ros::Subscriber left_gripper_sub = nh.subscribe<std_msgs::String>("/pr2_gripper_event/left_gripper_event", 1000, boost::bind(&onLeftGripperEvent,_1,boost::ref(left_event)));
	ros::Subscriber left_arm_sub_effort = nh.subscribe<std_msgs::Float32>("/pr2_stall_event/l_arm_effort", 1000, boost::bind(&onLeftArmEffortEvent,_1,&left_arm_effort_error));
	ros::Subscriber left_arm_sub_joint = nh.subscribe<std_msgs::Float32>("/pr2_stall_event/l_arm_joint", 1000, boost::bind(&onLeftArmJointEvent,_1,&left_arm_joint_error));


	ros::Rate r(10); // 10 hz


	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();

	auto cartesian = std::make_shared<solvers::CartesianPath>();
	cartesian->setProperty("jump_threshold", 0.0);

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId(choosedPlanner);

	geometry_msgs::PoseStamped graspPose_left;
	graspPose_left.header.frame_id = left_obj;
	graspPose_left.pose.position.x = 0.00;
	graspPose_left.pose.position.y = 0.0;
	graspPose_left.pose.position.z = 0.02;
	graspPose_left.pose.orientation.x = 0.500;
	graspPose_left.pose.orientation.y = 0.500;
	graspPose_left.pose.orientation.z = -0.500;
	graspPose_left.pose.orientation.w = 0.500;

	geometry_msgs::PoseStamped movePose_left;
	movePose_left.header.frame_id = left_obj;
	movePose_left.pose.position.x = 0.00;
	movePose_left.pose.position.y = 0.0;
	movePose_left.pose.position.z = 0.2;
	movePose_left.pose.orientation.x = 0.500;
	movePose_left.pose.orientation.y = 0.500;
	movePose_left.pose.orientation.z = -0.500;
	movePose_left.pose.orientation.w = 0.500;

	geometry_msgs::PoseStamped graspPose_right;
	graspPose_right.header.frame_id = right_obj;
	graspPose_right.pose.position.x = -0.01;
	graspPose_right.pose.position.y = 0.0;
	graspPose_right.pose.position.z = 0.02;
	graspPose_right.pose.orientation.x = 0.0;
	graspPose_right.pose.orientation.y = 0.0;
	graspPose_right.pose.orientation.z = 0.0;
	graspPose_right.pose.orientation.w = 1.0;


	geometry_msgs::PoseStamped placePose_left;
	placePose_left.header.frame_id = "base_footprint";
	placePose_left.pose.position.x = 0.7;
	placePose_left.pose.position.y = 0.0;
	placePose_left.pose.position.z = 0.75+0.05;
	placePose_left.pose.orientation.x = -1.0;
	placePose_left.pose.orientation.y = 0.0;
	placePose_left.pose.orientation.z = 0.0;
	placePose_left.pose.orientation.w = 0.0;

	geometry_msgs::PoseStamped placePose_right;
	placePose_right.header.frame_id = "base_footprint";
	placePose_right.pose.position.x = 0.70;
	placePose_right.pose.position.y = 0.0;
	placePose_right.pose.position.z = 0.75+0.15+0.04;
	placePose_right.pose.orientation.x = -1.0;
	placePose_right.pose.orientation.y = 0.0;
	placePose_right.pose.orientation.z = 0.0;
	placePose_right.pose.orientation.w = 0.0;


	//createPickTaskCustom(t,"left_arm","obj_2",graspPose);
  //createPickTask(t,pipeline,cartesian,kinematic_model,"left_arm","obj_0");
	Task moveLeft("moveLeft");
	Task pick_left("pick_left");
	Task pick_right("pick_right");
	Task place_left("place_left");
	Task place_right("place_right");



	try {
		//planTest(pick,pipeline,cartesian,kinematic_model);
		//createPickTask(pick_left,pipeline,cartesian,kinematic_model,"left_arm",left_obj);
		//createPickTask(pick_right,pipeline,cartesian,kinematic_model,"right_arm",right_obj);

		//createPickTaskCustom(pick_left,pipeline, cartesian,kinematic_model,"left_arm",left_obj, graspPose_left);
		//createPickTaskCustom(pick_right,pipeline, cartesian,kinematic_model,"right_arm",right_obj, graspPose_right);

		//createPlaceTask(place_left,pipeline,cartesian,kinematic_model,"left_arm",left_obj,placePose_left);
		//createPlaceTask(place_right,pipeline,cartesian,kinematic_model,"right_arm",right_obj,placePose_right);
		createMoveTask(moveLeft,pipeline, cartesian,kinematic_model,"left_arm", movePose_left);
		createPickTaskCustom(pick_left,pipeline, cartesian,kinematic_model,"left_arm",left_obj, graspPose_left);

		moveLeft.plan(5);

		std::cout << "waiting for any key + <enter>\n";
		std::cin >> ch;

		if(ch != 'q')
		{
			execute(moveLeft);
		}

		pick_left.plan(5);

		if(ch != 'q')
		{
			execute(pick_left);
		}
		//
		// pick_right.plan(10);
		// execute(pick_right);
		//
		//

		//
		// place_left.plan(10);
		// execute(place_left);
		//
		// place_right.plan(10);
		// execute(place_right);
	}
	catch (const InitStageException& e)
	{
		ROS_ERROR_STREAM(e);
	}

	if(ch != 'q')
	{
		//execute(place);

		// while(ros::ok())
		// {
		// 	if(left_arm_effort_error.data > 7.0 && left_arm_joint_error.data > 10)
		// 	{
		// 		gripper_open(left_gripper_client_);
		// 		return 0;
		// 	}
		// 	r.sleep();
		// }
	}

	while(ros::ok());


	return 0;
}
