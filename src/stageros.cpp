/*
 *  stageros
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**

@mainpage

@htmlinclude manifest.html
**/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>


// libstage
#include <stage.hh>

// roscpp
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetMap.h>

#include "tf/transform_broadcaster.h"

#include <map>
#include <memory>

#define USAGE "stageros <worldfile>"

#include "robot.h"

typedef boost::shared_ptr<StageRobot> StageRobotPtr;

// Contains a survey for scene models
struct SceneModels
{
	std::vector<Stg::ModelPosition *> positionmodels;
	std::vector<Stg::ModelCamera *> cameramodels;
	std::vector<Stg::ModelRanger *> lasermodels;
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

    double map_resolution;					//< resolution for published map
    std::string map_model_name;				//< model name to be used as raster map source
    std::string root_frame_id;

    static bool s_update(Stg::World* world, StageNode* node)
    {
        node->WorldCallback();
        // We return false to indicate that we want to be called again (an
        // odd convention, but that's the way that Stage works).
        return false;
    }

    // Appends the given robot ID to the given message name.  If omitRobotID
    // is true, an unaltered copy of the name is returned.
    //const char *mapName(const char *name, size_t robotID, Stg::Model* mod) const;
    //const char *mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const;

    tf::TransformBroadcaster tf;

    // Last time that we received a velocity command
    //ros::Time base_last_cmd;
    ros::Duration base_watchdog_timeout;

    // Current simulation time
    ros::Time sim_time;
    
    // Last time we saved global position (for velocity calculation).
    //ros::Time base_last_globalpos_time;
    // Last published global pose of each robot
    //std::vector<Stg::Pose> base_last_globalpos;

    // Period for map publishing
    double map_publish_period;

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
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    StageNode(int argc, char** argv, bool gui, bool use_model_names);
    ~StageNode();

    // Load the world and parse scene models
    // 0 on success (both models subscribed), -1 otherwise.
    int Load(const char* fname);
    // Our callback
    void WorldCallback();
    
    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    bool generateMap(nav_msgs::OccupancyGrid & map);

    // The main simulator object
    Stg::World* world;

protected:
	// Subscribe to models of interest.  Currently, we find and subscribe
	// to the first 'laser' model and the first 'position' model.  Returns
	// 0 on success (both models subscribed), -1 otherwise.
	int SubscribeModels(SceneModels & models);
	// Service callback for soft reset
	bool cb_reset_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
};

/*
// since stageros is single-threaded, this is OK. revisit if that changes!
const char * StageNode::mapName(const char *name, size_t robotID, Stg::Model* mod) const
{
    //ROS_INFO("Robot %lu: Device %s", robotID, name);
    bool umn = this->use_model_names;

    if ((positionmodels.size() > 1 ) || umn)
    {
        static char buf[100];
        std::size_t found = std::string(((Stg::Ancestor *) mod)->Token()).find(":");

        if ((found==std::string::npos) && umn)
        {
            snprintf(buf, sizeof(buf), "/%s/%s", ((Stg::Ancestor *) mod)->Token(), name);
        }
        else
        {
            snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);
        }

        return buf;
    }
    else
        return name;
}

const char * StageNode::mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const
{
    //ROS_INFO("Robot %lu: Device %s:%lu", robotID, name, deviceID);
    bool umn = this->use_model_names;

    if ((positionmodels.size() > 1 ) || umn)
    {
        static char buf[100];
        std::size_t found = std::string(((Stg::Ancestor *) mod)->Token()).find(":");

        if ((found==std::string::npos) && umn)
        {
            snprintf(buf, sizeof(buf), "/%s/%s_%u", ((Stg::Ancestor *) mod)->Token(), name, (unsigned int)deviceID);
        }
        else
        {
            snprintf(buf, sizeof(buf), "/robot_%u/%s_%u", (unsigned int)robotID, name, (unsigned int)deviceID);
        }

        return buf;
    }
    else
    {
        static char buf[100];
        snprintf(buf, sizeof(buf), "/%s_%u", name, (unsigned int)deviceID);
        return buf;
    }
}
*/

// A helper function that is executed for each stage model.  We use it
// to search for models of interest.
void ghfunc(Stg::Model* mod, SceneModels * models)
{
    if (dynamic_cast<Stg::ModelRanger *>(mod))
    {
        models->lasermodels.push_back(dynamic_cast<Stg::ModelRanger *>(mod));
    }
    if (dynamic_cast<Stg::ModelPosition *>(mod))
    {
      Stg::ModelPosition * p = dynamic_cast<Stg::ModelPosition *>(mod);
      // remember initial poses
      models->positionmodels.push_back(p);
      //node->initial_poses.push_back(p->GetGlobalPose());
    }
    if (dynamic_cast<Stg::ModelCamera *>(mod))
    {
      models->cameramodels.push_back(dynamic_cast<Stg::ModelCamera *>(mod));
    }
}


bool StageNode::cb_reset_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Resetting stage!");

  for (RobotContainer::iterator it = robotmodels_.begin(); it != robotmodels_.end(); ++it)
  {
  	it->second->resetPose();
  }
  return true;
}

bool
StageNode::generateMap(nav_msgs::OccupancyGrid & map)
{
	map.header.frame_id = this->root_frame_id;
	map.header.stamp = this->sim_time;
	map.info.resolution = map_resolution;
	map.info.origin.orientation.w = 1.0;

	Stg::Model * floor = world->GetModel(this->map_model_name);

	if(!floor)
	{
		ROS_ERROR("No floor model is found");
		return false;
	}

	Stg::Geom geom = floor->GetGeom();

	int width = ceilf(geom.size.x / map_resolution);
	int height = ceilf(geom.size.y / map_resolution);

	if(width * height == 0)
	{
		ROS_ERROR("Will not publish map with zero size");
		return false;
	}

	map.info.width = width;
	map.info.height = height;
	map.data.resize(width * height);
	unsigned char * data = reinterpret_cast<unsigned char*>(&map.data.front());

	/// Get floor data
	floor->Rasterize(data, width, height, map_resolution, map_resolution);

	Stg::Pose pose = floor->GetGlobalPose();
	//Stg::bounds3d_t bounds = floor->blockgroup.BoundingBox();

	map.info.origin.position.x = pose.x - geom.size.x*0.5;
	map.info.origin.position.y = pose.y - geom.size.y*0.5;
	/// Fixing 'colors' to match ROS nav_msgs::OccupancyGrid notation
	for(int i = 0; i < width * height; i++)
		if(data[i] > 0)
			data[i] = 100;

	return true;
}

StageNode::StageNode(int argc, char** argv, bool gui, bool use_model_names)
{
	this->use_acceleration_control = false;
	this->persistent_world = false;
	this->use_model_names = use_model_names;
	this->sim_time.fromSec(0.0);
	this->map_publish_period = -1.0;
	this->map_resolution = 0.05;

	double t;
	ros::NodeHandle localn("~");
	if(!localn.getParam("base_watchdog_timeout", t))
		t = 0.2;
	this->base_watchdog_timeout.fromSec(t);

	if(!localn.getParam("is_depth_canonical", isDepthCanonical))
		isDepthCanonical = true;

	localn.param<std::string>("persistent_file", this->persistent_file, "");
	localn.param<bool>("common_root", use_common_root, false);
	localn.param<bool>("control_acceleration", this->use_acceleration_control, false);
	localn.param<std::string>("root_frame_id", root_frame_id, "map");

	localn.param<double>("map_publish_period", map_publish_period, -1.0);
	localn.param<double>("map_resolution", map_resolution, 0.05);
	localn.param<std::string>("map_model_name", map_model_name, "ground");

	if(!persistent_file.empty())
		persistent_world = true;
#ifdef HAS_STAGE_GUI
	if(gui)
			this->world = new Stg::WorldGui(600, 400, "Stage (ROS)");
	else
#endif
	{
			this->world = new Stg::World();
	}
}

//@returns 0 on success (both models subscribed), -1 otherwise.
int StageNode::Load(const char* fname)
{
	assert(fname != NULL);
	// We'll check the existence of the world file, because libstage doesn't
	// expose its failure to open it.  Could go further with checks (e.g., is
	// it readable by this user).
	struct stat s;
	if(stat(fname, &s) != 0)
	{
			ROS_FATAL("The world file %s does not exist.", fname);
			return -1;
	}

	// Apparently an Update is needed before the Load to avoid crashes on
	// startup on some systems.
	// As of Stage 4.1.1, this update call causes a hang on start.
	//this->UpdateWorld();

	/// Try to load a scene from persistent file
	bool loaded = false;
	if(persistent_world)
	{
		if(this->world->Load(this->persistent_file))
		{
			ROS_INFO("Loaded world from persistent file %s", persistent_file.c_str());
			loaded = true;
		}
		else
		{
			ROS_ERROR("Failed to load world from persistent file %s. Loading from source file", persistent_file.c_str());
		}
	}

	if(!loaded)
	{
		if(this->world->Load(fname))
			loaded = true;
		else
		{
			ROS_FATAL("Failed to load world from %s", fname);
			return -1;
		}
	}

	if(loaded)
	{
		SceneModels models;
		this->world->ForEachDescendant((Stg::model_callback_t)ghfunc, &models);
		this->SubscribeModels(models);
		// We add our callback here, after the Update, so avoid our callback
		// being invoked before we're ready.
		this->world->AddUpdateCallback((Stg::world_callback_t)s_update, this);
	}

	return 0;
}

// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int StageNode::SubscribeModels(SceneModels & models)
{
	n_.setParam("/use_sim_time", true);

	for (size_t r = 0; r < models.positionmodels.size(); r++)
	{
		Stg::ModelPosition * positionmodel = models.positionmodels[r];
		std::string name;
		ROS_INFO("Found robot token=%s", positionmodel->Token());
		StageRobotPtr robot(new StageRobot(ros::NodeHandle(n_, name), positionmodel));
		robot->initROS();
		robotmodels_[name] = robot;

		/// Gather laser models for the robot
		for (size_t s = 0; s < models.lasermodels.size(); s++)
		{
			Stg::ModelRanger * model = models.lasermodels[s];
			if (model && model->Parent() == positionmodel)
				robot->addLaser(model);
		}

		/// Gather camera models for the robot
		for (size_t s = 0; s < models.cameramodels.size(); s++)
		{
			Stg::ModelCamera * model = models.cameramodels[s];
			if (model && model->Parent() == positionmodel)
				robot->addCamera(model);
		}
		robot->initROS();
	}

	clock_pub_ = n_.advertise<rosgraph_msgs::Clock>("/clock", 10);
	// advertising reset service
	reset_srv_ = n_.advertiseService("reset_positions", &StageNode::cb_reset_srv, this);

	// Advertising map topics only if map_publish_period is specified
	if(map_publish_period > 0)
	{
		if(this->map_model_name.empty())
		{
			ROS_ERROR("No 'map_model_name' is specified, no map will be published");
		}
		else
		{
		this->map_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 10);
		this->map_info_pub_ = n_.advertise<nav_msgs::MapMetaData>("map_info", 10);
		this->map_srv_ = n_.advertiseService("dynamic_map", &StageNode::cb_getmap_srv, this);
		this->map_publish_timer_ = n_.createTimer(ros::Duration(this->map_publish_period), &StageNode::onMapPublish, this);
		}
	}

	return(0);
}

StageNode::~StageNode()
{
	robotmodels_.clear();
	/// Save world before exit
	if(persistent_world)
	{
		ROS_INFO("Saving world state to %s", persistent_file.c_str());
		world->Save(this->persistent_file.c_str());
	}
}

bool
StageNode::UpdateWorld()
{
    return this->world->UpdateAll();
}


bool
StageNode::cb_getmap_srv(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
	res.map = this->cached_map_;
	return true;
}

void
StageNode::onMapPublish(const ros::TimerEvent & event)
{
	if(this->generateMap(cached_map_))
	{
		map_pub_.publish(this->cached_map_);
		map_info_pub_.publish(this->cached_map_.info);
	}
}

void
StageNode::WorldCallback()
{
    boost::mutex::scoped_lock lock(msg_lock);

    ros::Time last_time = sim_time;

    this->sim_time.fromSec(world->SimTimeNow() / 1e6);
    // We're not allowed to publish clock==0, because it used as a special
    // value in parts of ROS, #4027.
    if(this->sim_time.sec == 0 && this->sim_time.nsec == 0)
    {
        ROS_DEBUG("Skipping initial simulation step, to avoid publishing clock==0");
        return;
    }

    // TODO make this only affect one robot if necessary
    /*
    if((this->base_watchdog_timeout.toSec() > 0.0) &&
            ((this->sim_time - this->base_last_cmd) >= this->base_watchdog_timeout))
    {
        for (size_t r = 0; r < this->positionmodels.size(); r++)
            this->positionmodels[r]->SetSpeed(0.0, 0.0, 0.0);
    }*/

    //loop on the robot models
    for (RobotContainer::iterator it = robotmodels_.begin(); it != robotmodels_.end(); ++it)
    {
        StageRobotPtr robotmodel = it->second;

        PublishContext context = {use_common_root, root_frame_id, tf, isDepthCanonical};
        robotmodel->publishData(context);
    }

    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = sim_time;
    this->clock_pub_.publish(clock_msg);
}

int main(int argc, char** argv)
{ 
    if( argc < 2 )
    {
        puts(USAGE);
        exit(-1);
    }

    ros::init(argc, argv, "stageros");
    // initialize libstage
		Stg::Init( &argc, &argv );

    bool gui = true;
    bool use_model_names = false;
    for(int i=0;i<(argc-1);i++)
    {
        if(!strcmp(argv[i], "-g"))
            gui = false;
        if(!strcmp(argv[i], "-u"))
            use_model_names = true;
    }

    const char * fname = argv[argc-1];

    StageNode sn(argc-1,argv,gui,use_model_names);

    if(sn.Load(fname) != 0)
        exit(-1);

    boost::thread t = boost::thread(boost::bind(&ros::spin));

    ros::NodeHandle private_nh("~");

    double rate = 10;
    private_nh.param<double>("rate", rate, 10.0);

    ROS_INFO("Specified update rate: %f", rate);

    // New in Stage 4.1.1: must Start() the world.
    sn.world->Start();

    // TODO: get rid of this fixed-duration sleep, using some Stage builtin
    // PauseUntilNextUpdate() functionality.
    ros::WallRate r(rate);
    while(ros::ok() && !sn.world->TestQuit())
    {
#ifdef HAS_STAGE_GUI
        if(gui)
            Fl::wait(r.expectedCycleTime().toSec());
        else
#endif
        {
            sn.UpdateWorld();
            r.sleep();
        }
    }

    t.join();
}
