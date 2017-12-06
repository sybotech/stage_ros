/*
 * robotwrappwer.h
 *
 *  Created on: Dec 6, 2017
 *      Author: vrobot
 */

#ifndef STAGE_ROS_SRC_ROBOT_H_
#define STAGE_ROS_SRC_ROBOT_H_

// libstage
#include <stage.hh>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_broadcaster.h"

#include <vector>
#include <memory>

struct PublishContext
{
	bool use_common_root;
	std::string root_frame_id;
	tf::TransformBroadcaster & tf;
	bool isDepthCanonical;
};

// A class that wraps robot model from the simulator
class StageRobot
{
public:
	StageRobot(const ros::NodeHandle & nh, Stg::ModelPosition * positionmodel);
	~StageRobot();

	// Init ROS-related stuff here
	void initROS();
	// Get robot name
	const char * getName() const;

	// Return mapped name for an object
	const char * mapName(const char * name) const;

	const char * mapName(const char * name, int index) const;

	// Reset robot to initial state
	void resetPose();

	void addLaser(Stg::ModelRanger * model);
	void addCamera(Stg::ModelCamera * model);

	ros::Time getTime() const;
	// Publish model-related data to ROS
	void publishData(PublishContext & context);
protected:
	// Subscriber function for velocity commands
	void onCmdVel(const geometry_msgs::Twist& msg);
	// Callback for stage model. We do apply acceleration control here
	static int cb_model(Stg::ModelPosition * mod, StageRobot * sr);

	void publishLaser(PublishContext & context);
	void publishCamera(PublishContext & context);
	void publishOdom(PublishContext & context);
	// Node handle points to proper namespace. All robot's publishers are created relative to this NodeHandle
	ros::NodeHandle nh;
	//stage related models
	Stg::ModelPosition* positionmodel; //one position
	std::vector<Stg::ModelCamera *> cameramodels; 	//< multiple cameras per position
	std::vector<Stg::ModelRanger *> lasermodels; 		//< multiple rangers per position

	//ros publishers
	ros::Publisher odom_pub; //one odom
	ros::Publisher ground_truth_pub; //one ground truth

	std::vector<ros::Publisher> image_pubs; 	//multiple images
	std::vector<ros::Publisher> depth_pubs; 	//multiple depths
	std::vector<ros::Publisher> camera_pubs; 	//multiple cameras
	std::vector<ros::Publisher> laser_pubs; 	//multiple lasers

	Stg::Velocity target_vel; /// target velocity for acceleration control
	ros::Subscriber cmdvel_sub; //one cmd_vel subscriber
	ros::Time base_last_cmd;

	// Last time we saved global position (for velocity calculation).
	ros::Time base_last_globalpos_time;
	// Last published global pose of each robot
	Stg::Pose base_last_globalpos;

	Stg::Pose initial_pose;
};

#endif /* STAGE_ROS_SRC_ROBOT_H_ */
