/***********************************************************************/
/**                                                                    */
/** wsbs_controller.cpp                                                */
/**                                                                    */
/** Copyright (c) 2016, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Ignacio Perez-Hurtado (maintainer)                                 */
/** Jesus Capitan                                                      */
/** Fernando Caballero                                                 */
/** Luis Merino                                                        */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/


#include <ros/ros.h>
#include <teresa_wsbs/start.h>
#include <teresa_wsbs/stop.h>
#include <teresa_wsbs/select_mode.h>
#include <teresa_wsbs/common.hpp>
#include <teresa_wsbs/select_target_id.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt8.h>
#include <teresa_wsbs/Info.h>
#include <animated_marker_msgs/AnimatedMarker.h>
#include <animated_marker_msgs/AnimatedMarkerArray.h>	
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <lightsfm/sfm.hpp>
#include <lightsfm/cmd_vel.hpp>



namespace wsbs
{

#define ABORT_CODE 100

const double PERSON_MESH_SCALE = (2.0 / 8.5 * 1.8)*0.9;


class Timeout
{
public:
	Timeout() :id(""), time(ros::Time::now()), timeout(0) {}
	void setId(const std::string& id) {Timeout::id = id;}
	void setTimeout(double timeout) {Timeout::timeout = timeout;}
	void setTime(const ros::Time& time) {Timeout::time = time;}
	double getTimeout() const {return timeout;}
	const ros::Time& getTime() const {return time;}
	double getTimeElapsed() const {return (ros::Time::now() - time).toSec();}
	bool check(const ros::Time& current, bool isError = true) const
	{
		if ((current - time).toSec() >= timeout) {
			if (isError) {
				ROS_ERROR("%s timeout", id.c_str());
			}
			return true;
		}
		return false;
	}
private:
	std::string id;
	ros::Time time;
	double timeout;
};


class Controller
{
public:
	// Controller state
	enum State {
		WAITING_FOR_START    = 0, 
		WAITING_FOR_ODOM     = 1, 
		WAITING_FOR_LASER    = 2, 
		WAITING_FOR_XTION    = 3,
		WAITING_FOR_PEOPLE   = 4,
		RUNNING              = 5, 
		TARGET_LOST          = 6,
		FINISHED             = 7,
		ABORTED		     = 8
	};

	// Service error codes
	enum ErrorCode {
		SUCCESS                     = 0,
		STATE_IS_WAITING_FOR_START  = 1, 
		STATE_IS_WAITING_FOR_ODOM   = 2, 
		STATE_IS_WAITING_FOR_LASER  = 3, 
		STATE_IS_WAITING_FOR_XTION  = 4,
		STATE_IS_WAITING_FOR_PEOPLE = 5, 
		STATE_IS_RUNNING            = 6, 
		STATE_IS_TARGET_LOST        = 7,
		STATE_IS_FINISHED           = 8
	};


	
	Controller(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Controller() {}
private:
	void pointReceived(const geometry_msgs::PointStamped::ConstPtr& point); 
	bool start(teresa_wsbs::start::Request &req, teresa_wsbs::start::Response &res);
	bool stop(teresa_wsbs::stop::Request &req, teresa_wsbs::stop::Response &res);
	bool selectTargetId(teresa_wsbs::select_target_id::Request &req, teresa_wsbs::select_target_id::Response &res);
	bool selectMode(teresa_wsbs::select_mode::Request &req, teresa_wsbs::select_mode::Response &res);

	void odomReceived(const nav_msgs::Odometry::ConstPtr& odom);
	void laserReceived(const sensor_msgs::LaserScan::ConstPtr& laser);
	void xtionReceived(const sensor_msgs::LaserScan::ConstPtr& xtion);
	void peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people);
	
	void publishForceMarker(unsigned index, const std_msgs::ColorRGBA& color, 
					const utils::Vector2d& force, visualization_msgs::MarkerArray& markers);

	
	void publishTrajectories(sfm::CmdVelProvider& cmdVelProvider);
	void stopRobot();
	void setState(const State& state);
	static const char *getStateId(const State& state);

	
	void reset();
	void publishTarget();
	void publishScan();

	static std_msgs::ColorRGBA getColor(double r, double g, double b, double a);

	void publishForces();
	void publishPath(const GoalProvider& goalProvider);
	void publishGoal();
	void publishStatus();
	void checkEndingCondition(bool finishing);
	void checkTimeouts(const ros::Time& current_time);
	

	State state;

	std::string cmd_vel_id;
	std::string laser_id;
	std::string xtion_id;
	std::string odom_id; 
	std::string people_id; 

	Timeout odom_timeout;
	Timeout laser_timeout;
	Timeout xtion_timeout;
	Timeout people_timeout;
	Timeout finish_timeout;
	Timeout target_lost_timeout;
	Timeout goal_timeout;
	
	ros::Publisher cmd_vel_pub;
	
	unsigned targetId;
	ControllerMode controller_mode;
	utils::Vector2d controller_mode_goal;
	
	ros::Publisher robot_markers_pub;
	ros::Publisher target_pub;
	ros::Publisher robot_pub;
	ros::Publisher path_pub;
	ros::Publisher goal_pub;
	ros::Publisher trajectories_pub;
	ros::Publisher status_pub;
	ros::Publisher forces_pub;
	ros::Publisher scan_pub;
	


	geometry_msgs::Twist zeroTwist;

	bool is_finishing;
	bool break_if_aborted;
	bool break_if_finished;

	std::vector<sfm::Agent> agents; // 0: robot, 1..: Others 

	utils::Vector2d currentGoal;
	utils::Vector2d targetPos;
	utils::Vector2d targetVel;
	sfm::Goal robot_local_goal;
	
	
	bool use_estimated_target;
	bool clicked_goals;
	
	bool publish_target;

	double robot_radius;
	double person_radius;
	bool heuristic_controller;
	bool targetFound;
	unsigned target_index;
	

	double obstacle_distance_threshold;
	double naive_goal_time;

	double goal_radius;
	double target_velocity;
	double people_velocity;
	double robot_velocity;

	double robot_max_lin_acc;
	double robot_max_ang_acc;
	double beta_v;
	double beta_y;
	double beta_d;
	
	sfm::CmdVelProvider *cmdVelProviderPtr;
	GoalProvider *goalProviderPtr;

	ros::Time modelTime;
	double modelTimeStep;
	utils::Vector2d modelPos;
	std::vector<double> controllerError;
	
};


Controller::Controller(ros::NodeHandle& n, ros::NodeHandle& pn)
:  state(WAITING_FOR_START),
   controller_mode(HEURISTIC),
   is_finishing(false),
   targetFound(false),
   target_index(0),
   modelTimeStep(1.0)
{
	double odom_timeout_threshold;
	double laser_timeout_threshold;
	double xtion_timeout_threshold;
	double people_timeout_threshold;
	double finish_timeout_threshold;
	double target_lost_timeout_threshold;
	double goal_timeout_threshold;
	double freq;
	double lookahead;
	
	std::string path_file;	
	zeroTwist.linear.x = 0;
	zeroTwist.linear.y = 0;
	zeroTwist.linear.z = 0;
	zeroTwist.angular.x = 0;
	zeroTwist.angular.y = 0;
	zeroTwist.angular.z = 0;

	pn.param<std::string>("odom_id",odom_id,"/odom");
	pn.param<std::string>("laser_id",laser_id,"/scan360");
	pn.param<std::string>("xtion_id",xtion_id,"/depthcamscan_node/scanXtion");
	pn.param<std::string>("people_id",people_id,"/people/navigation");
	pn.param<std::string>("cmd_vel_id",cmd_vel_id,"/cmd_vel");
	
	pn.param<double>("robot_radius",robot_radius,0.35);
	pn.param<double>("person_radius",person_radius,0.35);
	pn.param<double>("target_velocity",target_velocity,0.6);
	pn.param<double>("people_velocity",people_velocity,0.6);
	pn.param<double>("robot_velocity",robot_velocity,0.6);
	pn.param<double>("robot_max_lin_acc",robot_max_lin_acc,1.0);
	pn.param<double>("robot_max_ang_acc",robot_max_ang_acc,2.0);
	pn.param<double>("beta_v",beta_v,0.4);
	pn.param<double>("beta_y",beta_y,0.3);
	pn.param<double>("beta_d",beta_d,0.3);

	pn.param<double>("odom_timeout",odom_timeout_threshold,0.5);
	pn.param<double>("laser_timeout",laser_timeout_threshold,0.5);
	pn.param<double>("xtion_timeout",xtion_timeout_threshold,0.5);
	pn.param<double>("people_timeout",people_timeout_threshold,1200.0);
	pn.param<double>("finish_timeout",finish_timeout_threshold,5); // 10
	pn.param<double>("target_lost_timeout",target_lost_timeout_threshold,5); // 30
	pn.param<double>("goal_timeout_threshold",goal_timeout_threshold,40);
	pn.param<bool>("use_estimated_target",use_estimated_target,false);
	pn.param<bool>("publish_target",publish_target,true);
	pn.param<std::string>("path_file",path_file,"");
	pn.param<bool>("break_if_aborted",break_if_aborted,false);
	pn.param<bool>("break_if_finished",break_if_finished,false);
	pn.param<double>("obstacle_distance_threshold",obstacle_distance_threshold,2.0);	
	pn.param<double>("naive_goal_time",naive_goal_time,2.0);
	pn.param<double>("goal_radius",goal_radius,0.25);
	pn.param<double>("lookahead",lookahead,2.0);	
	
	AStarPathProvider pathProvider(path_file);
	
	pn.param<double>("freq",freq,15);
	pn.param<bool>("heuristic_controller",heuristic_controller, true);
	pn.param<bool>("clicked_goals",clicked_goals,false); // Controller no heuristic, no data association, going to goal clicked in Rviz
	
	if (clicked_goals) {
		controller_mode = SET_FINAL_GOAL;
	}
	ros::ServiceServer select_mode_srv;
	std::string start_service_name, stop_service_name;
	if (heuristic_controller || clicked_goals) {
		start_service_name = "/wsbs/start";
		stop_service_name = "/wsbs/stop";
	} else {
		start_service_name = "/wsbs/controller/start";
		stop_service_name = "/wsbs/controller/stop";
		select_mode_srv = n.advertiseService("/wsbs/select_mode", &Controller::selectMode,this);	
	}

	ros::ServiceServer start_srv = n.advertiseService(start_service_name, &Controller::start,this);
	ros::ServiceServer stop_srv  = n.advertiseService(stop_service_name, &Controller::stop,this);
	ros::ServiceServer select_id_srv  = n.advertiseService("/wsbs/select_target_id", &Controller::selectTargetId,this);	
	
	ros::Subscriber point_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point",1,&Controller::pointReceived,this);
	ros::Subscriber people_sub = n.subscribe<upo_msgs::PersonPoseArrayUPO>(people_id, 1, &Controller::peopleReceived,this);
	
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(odom_id, 1, &Controller::odomReceived,this);
	ros::Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_id, 1, &Controller::laserReceived,this);
	scan_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/scan",1);
	ros::Subscriber xtion_sub;
	if (!xtion_id.empty()) {
		xtion_sub = n.subscribe<sensor_msgs::LaserScan>(xtion_id, 1, &Controller::xtionReceived,this);
	}
	agents.resize(1);
	agents[0].desiredVelocity = robot_velocity;
	agents[0].radius = robot_radius;
	agents[0].cyclicGoals = false;
	agents[0].teleoperated = true;

	status_pub = pn.advertise<std_msgs::UInt8>("/wsbs/status", 1);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_id, 1);
	robot_markers_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/robot_forces", 1);
	target_pub = pn.advertise<animated_marker_msgs::AnimatedMarkerArray>("/wsbs/markers/target", 1);
	robot_pub = pn.advertise<animated_marker_msgs::AnimatedMarkerArray>("/wsbs/markers/robot", 1);
	forces_pub = pn.advertise<teresa_wsbs::Info>("/wsbs/controller/info", 1);
	path_pub = pn.advertise<visualization_msgs::Marker>("/wsbs/markers/target_path", 1);
	goal_pub = pn.advertise<visualization_msgs::Marker>("/wsbs/markers/local_goal", 1);
	trajectories_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/trajectories", 1);
	
	odom_timeout.setId(odom_id);
	odom_timeout.setTimeout(odom_timeout_threshold);
	odom_timeout.setTime(ros::Time::now());

	laser_timeout.setId(laser_id);
	laser_timeout.setTimeout(laser_timeout_threshold);
	laser_timeout.setTime(ros::Time::now());

	xtion_timeout.setId(xtion_id);
	xtion_timeout.setTimeout(xtion_timeout_threshold);
	xtion_timeout.setTime(ros::Time::now());

	people_timeout.setId(people_id);
	people_timeout.setTimeout(people_timeout_threshold);
	people_timeout.setTime(ros::Time::now());

	target_lost_timeout.setId("Target lost");
	target_lost_timeout.setTimeout(target_lost_timeout_threshold);
	target_lost_timeout.setTime(ros::Time::now());

	finish_timeout.setId("Finish");
	finish_timeout.setTimeout(finish_timeout_threshold);
	finish_timeout.setTime(ros::Time::now());

	goal_timeout.setId("Goal");
	goal_timeout.setTimeout(goal_timeout_threshold);
	goal_timeout.setTime(ros::Time::now());
	upo_msgs::PersonPoseArrayUPO arrayAux;
	arrayAux.header.frame_id="/odom";
	upo_msgs::PersonPoseArrayUPO::ConstPtr ptr(&arrayAux);
	ros::Rate r(freq);
	//double dt = 1/freq;
	bool finishing;
	controller_mode_goal.set(24.4989,26.5523);

	GoalProvider goalProvider(0.5,100,lookahead,naive_goal_time,1.0,"odom",pathProvider);
	goalProviderPtr = &goalProvider;

	sfm::CmdVelProvider cmdVelProvider(obstacle_distance_threshold,robot_velocity ,robot_max_lin_acc,robot_max_ang_acc, beta_v,  beta_y,  beta_d);
	cmdVelProviderPtr = &cmdVelProvider;
	while(n.ok()) {
		checkTimeouts(ros::Time::now());
		if (state == RUNNING || state == TARGET_LOST) {
			if (people_timeout.getTimeElapsed()>=1.0) {
				peopleReceived(ptr);
			}
			
			goalProvider.update(agents[0].position,targetPos,targetVel,controller_mode_goal);
			agents[0].goals.clear();
			robot_local_goal.center = goalProvider.getRobotLocalGoal(controller_mode);
			robot_local_goal.radius = goal_radius;
			agents[0].goals.push_back(robot_local_goal);
			sfm::SFM.computeForces(agents);
			if ((ros::Time::now() - modelTime).toSec()>modelTimeStep) {
				double error = (agents[0].position - modelPos).norm();
				controllerError.push_back(error);
				//std::cout<<error<<std::endl;
				modelTime = ros::Time::now();
				utils::Vector2d vel = agents[0].velocity + agents[0].forces.globalForce * modelTimeStep;
				if (vel.norm() > agents[0].desiredVelocity) {
					vel.normalize();
					vel *= agents[0].desiredVelocity;
				}
				modelPos = agents[0].position + vel * modelTimeStep;
				
			}
			finishing=cmdVelProvider.compute(agents[0],agents[target_index],targetFound,!agents[0].antimove,agents[0].params.relaxationTime);
			cmd_vel_pub.publish(cmdVelProvider.getCommand());


			checkEndingCondition(finishing);
			publishForces();
			publishPath(goalProvider);
			publishGoal();
			publishTrajectories(cmdVelProvider);
			publishScan();
			publishTarget();			
			
		}
		
		if(!r.sleep())
			ROS_WARN("WSBS controller desired rate not met");
	
		ros::spinOnce();
		publishStatus();
		if (break_if_aborted && state == ABORTED) {
			break;
		}
		if (break_if_finished && state == FINISHED) {
			break;
		}	
	}
	double averageControllerError=0;
	for (unsigned i=1;i<controllerError.size();i++) {
		averageControllerError+=controllerError[i];
	}
	averageControllerError/=(double)controllerError.size()-1;
	double sd=0;
	for (unsigned i=1;i<controllerError.size();i++) {
		sd+=(controllerError[i] - averageControllerError) * (controllerError[i] - averageControllerError);
	}
	sd/=(double)controllerError.size()-1;
	sd = sqrt(sd);
	ROS_WARN("AVERAGE CONTROLLER ERROR: %f. SD: %f. N: %d",averageControllerError,sd,(int)controllerError.size()-1);
}	



void Controller::pointReceived(const geometry_msgs::PointStamped::ConstPtr& point)
{
	if (clicked_goals) {
		ROS_INFO("CLICKED GOAL received");
		controller_mode_goal.set(point->point.x, point->point.y);
	}
}


void Controller::checkEndingCondition(bool finishing)
{
	if (state == TARGET_LOST &&
		target_lost_timeout.check(ros::Time::now())) {
		setState(ABORTED);
		return;
	}
	
	if (!is_finishing && finishing) {
		finish_timeout.setTime(ros::Time::now());
	}
	
	is_finishing = finishing;

	if (state == RUNNING && is_finishing && finish_timeout.check(ros::Time::now(),false)) {
		is_finishing=false;
		setState(FINISHED);
		return;
	}
	
	if (state == RUNNING && (currentGoal - agents[0].goals.front().center).norm()<0.01 &&
		goal_timeout.check(ros::Time::now())) {
			setState(ABORTED);
	} else {
		currentGoal = agents[0].goals.front().center;
		goal_timeout.setTime(ros::Time::now());
	}
	
}

void Controller::publishStatus()
{
	std_msgs::UInt8 status_msg;
	status_msg.data = (uint8_t)state;
	status_pub.publish(status_msg);

}

void Controller::checkTimeouts(const ros::Time& current_time)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}

	if (state!= WAITING_FOR_ODOM && 
		odom_timeout.check(current_time)) {
		setState(WAITING_FOR_ODOM);
		return;
	}	
	
	if (state!= WAITING_FOR_ODOM && 
		state!= WAITING_FOR_LASER && 
		laser_timeout.check(current_time)) {
		setState(WAITING_FOR_LASER);
		return;
	}	
	
	if (!xtion_id.empty() && 
		state!= WAITING_FOR_ODOM && 
		state!= WAITING_FOR_LASER && 
		state!= WAITING_FOR_XTION && 
		xtion_timeout.check(current_time)) {
		setState(WAITING_FOR_XTION);
		return;
	}	

	if (state!= WAITING_FOR_ODOM && 
		state!= WAITING_FOR_LASER && 
		state!= WAITING_FOR_XTION && 
		state!= WAITING_FOR_PEOPLE && 
		people_timeout.check(current_time)) {
		setState(WAITING_FOR_PEOPLE);
		return;
	}	

	

}



void Controller::publishTrajectories(sfm::CmdVelProvider& cmdVelProvider)
{
	for (unsigned i=0; i< cmdVelProvider.getMarkers().markers.size(); i++) {
		cmdVelProvider.getMarkers().markers[i].action = (state==RUNNING || state==TARGET_LOST)?0:2;
	}
	trajectories_pub.publish(cmdVelProvider.getMarkers());

}


void Controller::stopRobot()
{
	cmd_vel_pub.publish(zeroTwist);
}



bool Controller::start(teresa_wsbs::start::Request &req, teresa_wsbs::start::Response &res) 
{
	ROS_INFO("START received");
	if (state != WAITING_FOR_START && state != FINISHED && state != ABORTED) {
		ROS_ERROR("Error: State is %s",getStateId(state));
		res.error_code = state+1;
	} else {
		ROS_INFO("Target ID is %d",req.target_id);
		targetId = req.target_id;
		setState(WAITING_FOR_ODOM);
		res.error_code = 0;
	}
	return true;
}


bool Controller::stop(teresa_wsbs::stop::Request &req, teresa_wsbs::stop::Response &res)
{
	ROS_INFO("STOP received");
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		ROS_ERROR("Error: State is %s",getStateId(state));
		res.error_code = state+1;
	} else {
		setState(FINISHED);
		res.error_code = 0;
	}
	return true;
}

bool Controller::selectTargetId(teresa_wsbs::select_target_id::Request &req, teresa_wsbs::select_target_id::Response &res)
{
	ROS_INFO("SELECT_TARGET_ID received");
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		ROS_ERROR("Error: State is %s",getStateId(state));
		res.error_code = state+1;
	} else {
		ROS_INFO("Target ID is %d",req.target_id);
		targetId = req.target_id;
		res.error_code = 0;
	}
	return true;
}

bool Controller::selectMode(teresa_wsbs::select_mode::Request &req, teresa_wsbs::select_mode::Response &res)
{
	ROS_INFO("SELECT_MODE received");
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		ROS_ERROR("Error: State is %s",getStateId(state));
		res.error_code = state+1;
	} else if (req.controller_mode == ABORT_CODE) {
		ROS_INFO("Abort code received");
		setState(ABORTED);
		res.error_code = 0;
	} else {
		ROS_INFO("Controller mode is %d",req.controller_mode);
		controller_mode = (ControllerMode)req.controller_mode;
		if (use_estimated_target) {
			ROS_INFO("Likely target ID is %d",req.target_id);
			if (req.target_id >= 0) {
				targetId = req.target_id;
			}
			ROS_INFO("Likely target position is (%f, %f)@odom",req.target_pos_x,req.target_pos_y);
			targetPos.set(req.target_pos_x,req.target_pos_y);
			ROS_INFO("Likely target yaw is (%f)@odom radians",req.target_yaw);
			ROS_INFO("Likely target velocity is %f m/s",req.target_vel);
			utils::Angle yaw = utils::Angle::fromRadian(req.target_yaw);
			targetVel.set(req.target_vel * yaw.cos(),req.target_vel * yaw.sin());
		}
		ROS_INFO("Likely target goal is (%f, %f)@map", req.goal_x, req.goal_y);
		controller_mode_goal.set(req.goal_x,req.goal_y);
		res.error_code = 0;
	}
	return true;
}

std_msgs::ColorRGBA Controller::getColor(double r, double g, double b, double a)
{
	std_msgs::ColorRGBA color;
	color.r = r;
	color.g = g;
	color.b = b;
	color.a = a;
	return color;
}


void Controller::publishTarget()
{
	
	if (publish_target && targetFound && (state == RUNNING || state == TARGET_LOST)) {
		animated_marker_msgs::AnimatedMarkerArray marker_array;
		animated_marker_msgs::AnimatedMarker marker;
       		marker.mesh_use_embedded_materials = true;
		marker.lifetime = ros::Duration(1.0);	
		marker.header.frame_id = odom_id;
		marker.header.stamp = ros::Time::now();
		marker.action = targetFound?0:2;
		marker.id = 0;
		marker.type = animated_marker_msgs::AnimatedMarker::MESH_RESOURCE;
		marker.mesh_resource = "package://teresa_wsbs/images/animated_walking_man.mesh";
		marker.pose.position.x = agents[target_index].position.getX();
		marker.pose.position.y = agents[target_index].position.getY();
		marker.action = 0; 
		marker.scale.x = PERSON_MESH_SCALE;
		marker.scale.y = PERSON_MESH_SCALE;
		marker.scale.z = PERSON_MESH_SCALE;
		marker.color.a = 1.0;
		marker.color.r = 0.255;
		marker.color.g = 0.412;
		marker.color.b = 0.882;
		marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI*0.5, 0.0, agents[target_index].yaw.toRadian()+M_PI*0.5);
  		marker.animation_speed =agents[target_index].linearVelocity * 0.7;
		marker_array.markers.push_back(marker);	
		target_pub.publish(marker_array);
	}
	// Put the robot
	animated_marker_msgs::AnimatedMarkerArray marker_array1;
	animated_marker_msgs::AnimatedMarker marker1;
       	marker1.mesh_use_embedded_materials = true;
	marker1.lifetime = ros::Duration(1.0);	
	marker1.header.frame_id = odom_id;
	marker1.header.stamp = ros::Time::now();
	marker1.action = 0;
	marker1.id = 1;	
	marker1.type = animated_marker_msgs::AnimatedMarker::MESH_RESOURCE;
	marker1.mesh_resource = "package://teresa_wsbs/images/full_TERESA.dae";
	marker1.pose.position.x = agents[0].position.getX();
	marker1.pose.position.y = agents[0].position.getY();
	marker1.scale.x = PERSON_MESH_SCALE*0.25;
	marker1.scale.y = PERSON_MESH_SCALE*0.25;
	marker1.scale.z = PERSON_MESH_SCALE*0.25;
	marker1.color.a = 1.0;
	marker1.color.r = 1.0;//0.882;
	marker1.color.g = 1.0;//0.412;
	marker1.color.b = 1.0;//0.255;
	marker1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
						0.0, 0.0, agents[0].yaw.toRadian()-M_PI*0.5);
  	marker1.animation_speed = agents[0].velocity.norm() * 0.7;
	

	
	marker_array1.markers.push_back(marker1);
	robot_pub.publish(marker_array1);
	
	
}





const char *Controller::getStateId(const State& state)
{
	static std::string ids[] = {"WAITING_FOR_START",
				    "WAITING_FOR_ODOM",
				    "WAITING_FOR_LASER",
				    "WAITING_FOR_XTION",
				    "WAITING_FOR_PEOPLE",
				    "RUNNING",
				    "TARGET_LOST",
				    "FINISHED",
				    "ABORTED"};

	return ids[state].c_str();
}

void Controller::reset()
{
	agents.clear();
	agents.resize(1);

}


void Controller::setState(const State& state)
{
	if (state == Controller::state) {
		return;
	}
	
		
	if ( (Controller::state == RUNNING && state != TARGET_LOST) ||
		(Controller::state == TARGET_LOST && state != RUNNING)) {
		stopRobot();
		reset();
		publishForces();
		publishPath(*goalProviderPtr);
		publishGoal();
		publishTrajectories(*cmdVelProviderPtr);
		publishTarget();
	}
	ROS_INFO("State is %s",getStateId(state));
	Controller::state = state;
	if (state == TARGET_LOST) {
		target_lost_timeout.setTime(ros::Time::now());
	}
	if (state == RUNNING) {
		goal_timeout.setTime(ros::Time::now());
	}
}




void Controller::publishForceMarker(unsigned index, const std_msgs::ColorRGBA& color, 
					const utils::Vector2d& force, 
					visualization_msgs::MarkerArray& markers) 
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "robot_forces";
	marker.id = index;
	marker.action = force.norm()>1e-4 && (state==RUNNING || state==TARGET_LOST)?0:2;
	marker.color = color;
	marker.lifetime = ros::Duration(1.0);
	marker.scale.x = std::max(1e-4,force.norm());
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.pose.position.x = agents[0].position.getX();
	marker.pose.position.y = agents[0].position.getY();
	marker.pose.position.z = 0;
	marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,force.angle().toRadian());
	markers.markers.push_back(marker);
}


void Controller::publishScan()
{
	visualization_msgs::MarkerArray markers;
	unsigned counter=0;
	for (unsigned k= 0; k<2; k++) {
		const std::vector<utils::Vector2d>& obstacles = k==0 ? agents[0].obstacles1 : agents[0].obstacles2; 
		for (unsigned i = 0; i< obstacles.size(); i++) {
			visualization_msgs::Marker marker;
			marker.header.frame_id = "base_link";
          		marker.header.stamp = ros::Time::now();
			marker.ns = "scan_markers";
			marker.type = visualization_msgs::Marker::CUBE;
			marker.id = counter++;
			marker.action = visualization_msgs::Marker::ADD;
			marker.color.a = 1.0;			
			marker.color.r = 0.0;
            		marker.color.g = 1.0;
            		marker.color.b = 0.0;
			marker.lifetime = ros::Duration(0.1);
			marker.scale.x = 0.1;
            		marker.scale.y = 0.1;
            		marker.scale.z = 0.1;
			marker.pose.position.x = obstacles[i].getX();
			marker.pose.position.y = obstacles[i].getY();
			marker.pose.position.z = 0;
			markers.markers.push_back(marker);
		}
	}	
	scan_pub.publish(markers);

}



void Controller::publishForces()
{
	visualization_msgs::MarkerArray markers;
	publishForceMarker(0,getColor(0,0,1,1),agents[0].forces.obstacleForce,markers);
	publishForceMarker(1,getColor(0,1,1,1),agents[0].forces.socialForce,markers);	
	publishForceMarker(2,getColor(0,1,0,1),agents[0].forces.groupForce,markers);	
	publishForceMarker(3,getColor(1,0,0,1),agents[0].forces.desiredForce,markers);	
	//publishForceMarker(4,getColor(1,1,0,1),agents[0].forces.globalForce,markers);	
	publishForceMarker(4,getColor(1,1,0,1),agents[0].velocity,markers);	
	robot_markers_pub.publish(markers);


	teresa_wsbs::Info forces;
	forces.header.frame_id = "/odom";
	forces.header.stamp = ros::Time::now();

	forces.status = (uint8_t)state;
	forces.mode = (uint8_t)controller_mode;
	forces.target_detected = targetFound;
	forces.target_id = targetId;
	if (targetFound) {
		forces.target_pose.x = agents[target_index].position.getX();
		forces.target_pose.y = agents[target_index].position.getY();
		forces.target_pose.theta = agents[target_index].yaw.toRadian();
		forces.target_lin_vel = agents[target_index].linearVelocity;
		forces.target_group_force.x =  agents[target_index].forces.groupForce.getX();
		forces.target_group_force.y =  agents[target_index].forces.groupForce.getY();	
		forces.target_group_vis_force.x = agents[target_index].forces.groupGazeForce.getX();
		forces.target_group_vis_force.y = agents[target_index].forces.groupGazeForce.getY();
		forces.target_group_att_force.x = agents[target_index].forces.groupCoherenceForce.getX();
		forces.target_group_att_force.y = agents[target_index].forces.groupCoherenceForce.getY();
		forces.target_group_rep_force.x = agents[target_index].forces.groupRepulsionForce.getX();
		forces.target_group_rep_force.y = agents[target_index].forces.groupRepulsionForce.getY();
	}
	forces.robot_pose.x = agents[0].position.getX();
	forces.robot_pose.y = agents[0].position.getY();
	forces.robot_pose.theta = agents[0].yaw.toRadian();

	forces.robot_lin_vel = agents[0].linearVelocity;
	forces.robot_ang_vel = agents[0].angularVelocity;

	forces.robot_antimove =  agents[0].antimove;
	forces.robot_local_goal.x = agents[0].goals.front().center.getX();
	forces.robot_local_goal.y = agents[0].goals.front().center.getY();
	

	forces.robot_global_force.x = agents[0].forces.globalForce.getX();
	forces.robot_global_force.y = agents[0].forces.globalForce.getY();
	forces.robot_desired_force.x = agents[0].forces.desiredForce.getX();
	forces.robot_desired_force.y = agents[0].forces.desiredForce.getY();
	forces.robot_obstacle_force.x = agents[0].forces.obstacleForce.getX();
	forces.robot_obstacle_force.y = agents[0].forces.obstacleForce.getY();
	forces.robot_social_force.x = agents[0].forces.socialForce.getX();
	forces.robot_social_force.y = agents[0].forces.socialForce.getY();
	forces.robot_group_force.x =  agents[0].forces.groupForce.getX();
	forces.robot_group_force.y =  agents[0].forces.groupForce.getY();	
	forces.robot_group_vis_force.x = agents[0].forces.groupGazeForce.getX();
	forces.robot_group_vis_force.y = agents[0].forces.groupGazeForce.getY();
	forces.robot_group_att_force.x = agents[0].forces.groupCoherenceForce.getX();
	forces.robot_group_att_force.y = agents[0].forces.groupCoherenceForce.getY();
	forces.robot_group_rep_force.x = agents[0].forces.groupRepulsionForce.getX();
	forces.robot_group_rep_force.y = agents[0].forces.groupRepulsionForce.getY();
	forces.robot_vref.x = agents[0].velocity.getX();
	forces.robot_vref.y = agents[0].velocity.getY();

	forces_pub.publish(forces);
}


void Controller::publishPath(const GoalProvider& goalProvider)
{
	
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "target_path";
	marker.type = 4;
	marker.id = 0;
	marker.lifetime = ros::Duration(1.0);
	marker.action = goalProvider.getHistory().size()>0  && (state==RUNNING || state==TARGET_LOST)?0:2;
	marker.color = getColor(0,1,0,1);
	marker.scale.x = 0.05;
	
	for (auto it = 	goalProvider.getHistory().begin(); it != goalProvider.getHistory().end(); ++it) {
		geometry_msgs::Point p;
		p.x = it->getX();
		p.y = it->getY();
		p.z = 0;
		marker.points.push_back(p);
	}
	path_pub.publish(marker);	
	

}


void Controller::publishGoal()
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "goal";
	marker.type = 2;
	marker.id = 0;
	marker.lifetime = ros::Duration(1.0);
	marker.action = !agents[0].antimove && (state==RUNNING || state==TARGET_LOST)?0:2;
	marker.color = getColor(1,0,0,1);
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.pose.position.x = agents[0].goals.front().center.getX();
	marker.pose.position.y = agents[0].goals.front().center.getY();
	marker.pose.position.z = 0;
	

	goal_pub.publish(marker);	


}



void Controller::odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}
	if (odom->header.frame_id != "/odom" && odom->header.frame_id !="odom") {
		ROS_ERROR("Odometry frame is %s, it should be odom",odom->header.frame_id.c_str()); 
		return;
	}	
	agents[0].position.set(odom->pose.pose.position.x,odom->pose.pose.position.y); 
	agents[0].yaw = utils::Angle::fromRadian(tf::getYaw(odom->pose.pose.orientation));
	agents[0].linearVelocity = odom->twist.twist.linear.x;
	agents[0].angularVelocity = odom->twist.twist.angular.z;	
	agents[0].velocity.set(odom->twist.twist.linear.x * agents[0].yaw.cos(), odom->twist.twist.linear.x * agents[0].yaw.sin());
	odom_timeout.setTime(ros::Time::now());
	if (state == WAITING_FOR_ODOM) {
		setState(WAITING_FOR_LASER);
	}

}


void Controller::laserReceived(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}
	if (scan->header.frame_id != "/base_link" && scan->header.frame_id !="base_link") {
		ROS_ERROR("Laser frame is %s, it should be base_link",scan->header.frame_id.c_str()); 
		return;
	}
	utils::Angle alpha = agents[0].yaw + utils::Angle::fromRadian(scan->angle_min);
	utils::Angle angle_inc = utils::Angle::fromRadian(scan->angle_increment);
	agents[0].obstacles1.clear();
	for (unsigned i=0; i<scan->ranges.size();i++) {
		if (!std::isnan(scan->ranges[i]) && scan->ranges[i]<obstacle_distance_threshold) {
			agents[0].obstacles1.emplace_back(scan->ranges[i]*alpha.cos(),scan->ranges[i]*alpha.sin());
		}
		alpha+=angle_inc;
	}	
	laser_timeout.setTime(ros::Time::now());
	if (state == WAITING_FOR_LASER) {
		setState(xtion_id.empty() ? WAITING_FOR_PEOPLE : WAITING_FOR_XTION);
	}

}

void Controller::xtionReceived(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}
	if (scan->header.frame_id != "/base_link" && scan->header.frame_id !="base_link") {
		ROS_ERROR("Xtion frame is %s, it should be base_link",scan->header.frame_id.c_str()); 
		return;
	}
	utils::Angle alpha = agents[0].yaw + utils::Angle::fromRadian(scan->angle_min);
	utils::Angle angle_inc = utils::Angle::fromRadian(scan->angle_increment);
	agents[0].obstacles2.clear();
	for (unsigned i=0; i<scan->ranges.size();i++) {
		if (!std::isnan(scan->ranges[i]) && scan->ranges[i]<obstacle_distance_threshold) {
			agents[0].obstacles2.emplace_back(scan->ranges[i]*alpha.cos(),scan->ranges[i]*alpha.sin());
		}
		alpha+=angle_inc;
	}	
	xtion_timeout.setTime(ros::Time::now());
	if (state == WAITING_FOR_XTION) {
		setState(WAITING_FOR_PEOPLE);
	}


}

void Controller::peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}
	if (people->header.frame_id != "/odom" && people->header.frame_id !="odom") {
		ROS_ERROR("People frame is %s, it should be odom",people->header.frame_id.c_str()); 
		return;
	}
	target_index=0;
	targetFound=false;
	agents[0].groupId = -1;
	agents.resize(people->personPoses.size()+1);

	for (unsigned i=0; i< people->personPoses.size(); i++) {
		agents[i+1].position.set(people->personPoses[i].position.x,people->personPoses[i].position.y);
		agents[i+1].yaw = utils::Angle::fromRadian(tf::getYaw(people->personPoses[i].orientation));
		agents[i+1].velocity.set(people->personPoses[i].vel * agents[i+1].yaw.cos(), people->personPoses[i].vel * agents[i+1].yaw.sin());
		agents[i+1].linearVelocity = agents[i+1].velocity.norm();
		agents[i+1].radius = person_radius;
		agents[i+1].teleoperated=false;
		if (fabs(people->personPoses[i].vel) < 0.05) {
			agents[i+1].velocity.set(0,0);
		} 
		agents[i+1].goals.clear();
		sfm::Goal naiveGoal;
		naiveGoal.center =agents[i+1].position + naive_goal_time * agents[i+1].velocity;
		naiveGoal.radius = goal_radius;
		agents[i+1].goals.push_back(naiveGoal);
		if (people->personPoses[i].id == targetId) {
			targetFound = true;
			target_index=i+1;
			agents[i+1].desiredVelocity = target_velocity;
			agents[i+1].groupId = 0;
			agents[0].groupId = 0;
			targetPos = agents[i+1].position;
			targetVel = agents[i+1].velocity;
		} else {
			agents[i+1].desiredVelocity = people_velocity;
			agents[i+1].groupId = -1;
		}
	}
	people_timeout.setTime(ros::Time::now());
	if (state == RUNNING && agents[0].groupId == -1) {
		setState(TARGET_LOST);
	} else if (state == TARGET_LOST && agents[0].groupId != -1) {
		setState(RUNNING);
	} else	if (state == WAITING_FOR_PEOPLE) {
		setState(agents[0].groupId != -1 ? RUNNING : TARGET_LOST);
	}
}


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wsbs_controller");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	wsbs::Controller node(n,pn);
	return 0;
}
