/*
 * stageros.h
 *
 *  Created on: Dec 7, 2017
 *      Author: vrobot
 */

#ifndef STAGE_ROS_SRC_STAGEROS_H_
#define STAGE_ROS_SRC_STAGEROS_H_

// libstage
#include <stage.hh>

// roscpp
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetMap.h>

#include "tf/transform_broadcaster.h"

#include <map>
#include "robot.h"

typedef boost::shared_ptr<StageRobot> StageRobotPtr;

// Contains a survey for scene models
struct SceneModels
{
	std::vector<Stg::ModelPosition *> positionmodels;
	std::vector<Stg::ModelCamera *> cameramodels;
	std::vector<Stg::ModelRanger *> lasermodels;
};

// Export mode
// Export means exposing robot from stage to ROS
enum StageExportMode
{
	// Manual export. Should call a service to export another robot
	ExportManual = 0,
	// Export the first position model found in a world. All topics are created relative to 'global' namespace
	// Raises an error if no models found
	ExportSimple,
	// Export only the robots from specified tag list
	// Should set rosparam export to array of strings
	ExportSelected,
	// Export all position models from the world.
	// It will create ROS namespace for each robot. Model token is used for a name
	ExportAll,

};

// Our node
class StageNode
{
private:
	// roscpp-related bookkeeping
	ros::NodeHandle n_;

	// A mutex to lock access to fields that are used in message callbacks
	boost::mutex msg_lock;

	typedef std::map<std::string, StageRobotPtr > RobotContainer;
	RobotContainer robotmodels_;

	// Used to remember initial poses for soft reset
	std::vector<Stg::Pose> initial_poses;
	ros::ServiceServer reset_srv_;

	ros::Publisher clock_pub_;
	ros::Publisher map_pub_;
	ros::Publisher map_info_pub_;
	ros::ServiceServer map_srv_;

	ros::Timer map_publish_timer_;

	bool isDepthCanonical;
	bool use_model_names;
	bool use_acceleration_control;	//< Use acceleration control for all position models
	bool use_common_root;						//< Use common TF root for all the robots
	StageExportMode exportMode;			//< Current export mode

	double map_resolution;					//< resolution for published map
	std::string map_model_name;				//< model name to be used as raster map source
	std::string root_frame_id;

	static bool s_update(Stg::World* world, StageNode* node);

	tf::TransformBroadcaster tf;

	// Last time that we received a velocity command
	//ros::Time base_last_cmd;
	ros::Duration base_watchdog_timeout;

	// Current simulation time
	ros::Time sim_time;
	// Period for map publishing
	double map_publish_period;
	bool map_generated;
	// Contains cached map info
	nav_msgs::OccupancyGrid cached_map_;
	// Callback for map requests
	bool cb_getmap_srv(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);
	// Timer event for map updates
	void onMapPublish(const ros::TimerEvent& event);

	// Path for persistent world file. It will be saved when node exits
	std::string persistent_file;
	bool persistent_world;
public:
	// Constructor
	StageNode(Stg::World * world, bool use_model_names);
	~StageNode();

	// Load the world and parse scene models
	// It first tries to load world from persistent storage, and then only tries to load the world specified
	//@param *fname - world name to be loaded
	//@returns 0 on success (both models subscribed), -1 otherwise.
	int Load(const char* fname);
	// Our callback
	void WorldCallback();

	// Do one update of the world.  May pause if the next update time
	// has not yet arrived.
	bool UpdateWorld();

	// Fill in occupancy grid from floor model
	bool generateMap(nav_msgs::OccupancyGrid & map);

	// The main simulator object
	Stg::World* world;

protected:
	// Subscribe to models of interest.  Currently, we find and subscribe
	// to the first 'laser' model and the first 'position' model.  Returns
	// 0 on success (both models subscribed), -1 otherwise.
	int ExportModels(SceneModels & models);
	// Service callback for soft reset
	bool cb_reset_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
};

#endif /* STAGE_ROS_SRC_STAGEROS_H_ */
