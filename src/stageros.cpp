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
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetMap.h>

#include "tf/transform_broadcaster.h"

#define USAGE "stageros <worldfile>"
#define IMAGE "image"
#define DEPTH "depth"
#define CAMERA_INFO "camera_info"
#define ODOM "odom"
#define BASE_SCAN "base_scan"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"

// Our node
class StageNode
{
private:

    // roscpp-related bookkeeping
    ros::NodeHandle n_;

    // A mutex to lock access to fields that are used in message callbacks
    boost::mutex msg_lock;

    // The models that we're interested in
    std::vector<Stg::ModelCamera *> cameramodels;
    std::vector<Stg::ModelRanger *> lasermodels;
    std::vector<Stg::ModelPosition *> positionmodels;

    //a structure representing a robot inthe simulator
    struct StageRobot
    {
        //stage related models
        Stg::ModelPosition* positionmodel; //one position
        std::vector<Stg::ModelCamera *> cameramodels; //multiple cameras per position
        std::vector<Stg::ModelRanger *> lasermodels; //multiple rangers per position

        //ros publishers
        ros::Publisher odom_pub; //one odom
        ros::Publisher ground_truth_pub; //one ground truth

        std::vector<ros::Publisher> image_pubs; //multiple images
        std::vector<ros::Publisher> depth_pubs; //multiple depths
        std::vector<ros::Publisher> camera_pubs; //multiple cameras
        std::vector<ros::Publisher> laser_pubs; //multiple lasers

        Stg::Velocity target_vel; /// target velocity for acceleration control
        ros::Subscriber cmdvel_sub; //one cmd_vel subscriber
        ros::Time base_last_cmd;

        static int cb_model(Stg::ModelPosition * mod, StageRobot * sr);
    };

    std::vector<StageRobot*> robotmodels_;

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
    bool use_acceleration_control;
    bool use_common_root;

    double map_resolution;					//< resolution for published map
    std::string map_model_name;				//< model name to be used as raster map source

    std::string root_frame_id;

    // A helper function that is executed for each stage model.  We use it
    // to search for models of interest.
    static void ghfunc(Stg::Model* mod, StageNode* node);

    static bool s_update(Stg::World* world, StageNode* node)
    {
        node->WorldCallback();
        // We return false to indicate that we want to be called again (an
        // odd convention, but that's the way that Stage works).
        return false;
    }

    // Appends the given robot ID to the given message name.  If omitRobotID
    // is true, an unaltered copy of the name is returned.
    const char *mapName(const char *name, size_t robotID, Stg::Model* mod) const;
    const char *mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const;

    tf::TransformBroadcaster tf;

    // Last time that we received a velocity command
    //ros::Time base_last_cmd;
    ros::Duration base_watchdog_timeout;

    // Current simulation time
    ros::Time sim_time;
    
    // Last time we saved global position (for velocity calculation).
    ros::Time base_last_globalpos_time;
    // Last published global pose of each robot
    std::vector<Stg::Pose> base_last_globalpos;

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
    StageNode(int argc, char** argv, bool gui, const char* fname, bool use_model_names);
    ~StageNode();

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Our callback
    void WorldCallback();
    
    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    static void CalculateRobotControl(Stg::ModelPosition* mp, Stg::Velocity newVel, bool acc);

    // Message callback for a MsgBaseVel message, which set velocities.
    void cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg);

    bool generateMap(nav_msgs::OccupancyGrid & map);

    // Service callback for soft reset
    bool cb_reset_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    // The main simulator object
    Stg::World* world;
};

// since stageros is single-threaded, this is OK. revisit if that changes!
const char *
StageNode::mapName(const char *name, size_t robotID, Stg::Model* mod) const
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

const char *
StageNode::mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const
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

void
StageNode::ghfunc(Stg::Model* mod, StageNode* node)
{
    if (dynamic_cast<Stg::ModelRanger *>(mod))
        node->lasermodels.push_back(dynamic_cast<Stg::ModelRanger *>(mod));
    if (dynamic_cast<Stg::ModelPosition *>(mod)) {
      Stg::ModelPosition * p = dynamic_cast<Stg::ModelPosition *>(mod);
      // remember initial poses
      node->positionmodels.push_back(p);
      node->initial_poses.push_back(p->GetGlobalPose());
    }
    if (dynamic_cast<Stg::ModelCamera *>(mod))
        node->cameramodels.push_back(dynamic_cast<Stg::ModelCamera *>(mod));
}




bool
StageNode::cb_reset_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Resetting stage!");
  for (size_t r = 0; r < this->positionmodels.size(); r++) {
    this->positionmodels[r]->SetPose(this->initial_poses[r]);
    this->positionmodels[r]->SetStall(false);
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

	/*
	if(!floor)
		floor=world->GetGround();*/

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

void
StageNode::cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg)
{
    boost::mutex::scoped_lock lock(msg_lock);
    Stg::ModelPosition * mp = this->positionmodels[idx];
    assert(mp != NULL);

    Stg::Velocity newVel(msg->linear.x, msg->linear.y, 0.0, msg->angular.z);

    robotmodels_[idx]->target_vel = newVel;
    robotmodels_[idx]->base_last_cmd = this->sim_time;
}

StageNode::StageNode(int argc, char** argv, bool gui, const char* fname, bool use_model_names)
{
	this->use_acceleration_control = false;
	this->persistent_world = false;
	this->persistent_file = fname;
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

    // We'll check the existence of the world file, because libstage doesn't
    // expose its failure to open it.  Could go further with checks (e.g., is
    // it readable by this user).
    struct stat s;
    if(stat(fname, &s) != 0)
    {
        ROS_FATAL("The world file %s does not exist.", fname);
        ROS_BREAK();
    }

    // initialize libstage
    Stg::Init( &argc, &argv );

    if(gui)
        this->world = new Stg::WorldGui(600, 400, "Stage (ROS)");
    else
        this->world = new Stg::World();

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
    		ROS_FATAL("Failed to load world from persistent file %s. Loading from source file", persistent_file.c_str());
    }

    if(!loaded)
    {
    	if(this->world->Load(fname))
    		loaded = true;
    	else
    	{
    		ROS_FATAL("Failed to laod world from %s", fname);
    	}
    }

    if(loaded)
    {
		// We add our callback here, after the Update, so avoid our callback
		// being invoked before we're ready.
		this->world->AddUpdateCallback((Stg::world_callback_t)s_update, this);

		this->world->ForEachDescendant((Stg::model_callback_t)ghfunc, this);
    }
}

// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int
StageNode::SubscribeModels()
{
    n_.setParam("/use_sim_time", true);

    for (size_t r = 0; r < this->positionmodels.size(); r++)
    {
        StageRobot* new_robot = new StageRobot;
        new_robot->positionmodel = this->positionmodels[r];
        new_robot->positionmodel->Subscribe();
        new_robot->positionmodel->AddCallback(Stg::Model::CB_UPDATE, (Stg::model_callback_t)&StageRobot::cb_model, new_robot);

        for (size_t s = 0; s < this->lasermodels.size(); s++)
        {
            if (this->lasermodels[s] and this->lasermodels[s]->Parent() == new_robot->positionmodel)
            {
                new_robot->lasermodels.push_back(this->lasermodels[s]);
                this->lasermodels[s]->Subscribe();
            }
        }

        for (size_t s = 0; s < this->cameramodels.size(); s++)
        {
            if (this->cameramodels[s] and this->cameramodels[s]->Parent() == new_robot->positionmodel)
            {
                new_robot->cameramodels.push_back(this->cameramodels[s]);
                this->cameramodels[s]->Subscribe();
            }
        }

        ROS_INFO("Found %lu laser devices and %lu cameras in robot %lu", new_robot->lasermodels.size(), new_robot->cameramodels.size(), r);

        new_robot->odom_pub = n_.advertise<nav_msgs::Odometry>(mapName(ODOM, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10);
        new_robot->ground_truth_pub = n_.advertise<nav_msgs::Odometry>(mapName(BASE_POSE_GROUND_TRUTH, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10);
        new_robot->cmdvel_sub = n_.subscribe<geometry_msgs::Twist>(mapName(CMD_VEL, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10, boost::bind(&StageNode::cmdvelReceived, this, r, _1));

        for (size_t s = 0;  s < new_robot->lasermodels.size(); ++s)
        {
            if (new_robot->lasermodels.size() == 1)
                new_robot->laser_pubs.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(BASE_SCAN, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
            else
                new_robot->laser_pubs.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(BASE_SCAN, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));

        }

        for (size_t s = 0;  s < new_robot->cameramodels.size(); ++s)
        {
            if (new_robot->cameramodels.size() == 1)
            {
                new_robot->image_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(IMAGE, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->depth_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(DEPTH, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->camera_pubs.push_back(n_.advertise<sensor_msgs::CameraInfo>(mapName(CAMERA_INFO, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
            }
            else
            {
                new_robot->image_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(IMAGE, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->depth_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(DEPTH, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->camera_pubs.push_back(n_.advertise<sensor_msgs::CameraInfo>(mapName(CAMERA_INFO, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
            }
        }

        this->robotmodels_.push_back(new_robot);
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
    for (std::vector<StageRobot*>::iterator r = this->robotmodels_.begin(); r != this->robotmodels_.end(); ++r)
        delete *r;

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

/// Calculates new acceleration
template<typename Scalar> Scalar calculateControl(Scalar target, Scalar &current, Scalar dt, Scalar accDec, Scalar accInc)
{
	if (current >= target)	// deceleration profule
	{
		if (current - target < accDec*dt)
		{
			current = target;
			return Scalar(0);//(current-target)/dt;
		}
		else
		{
			return -accDec;
		}
			//result = current - delta;
	}
	else	// acceleration profile
	{
		if (target - current < accInc*dt)
		{
			current = target;
			return Scalar(0);//(target-current)/dt;//result = target;
		}
		else
		{
			return accInc;//result = current + delta;
		}
	}
	//return result;
	return Scalar(0);
}

int StageNode::StageRobot::cb_model(Stg::ModelPosition * mod, StageRobot * sr)
{
	StageNode::CalculateRobotControl(sr->positionmodel, sr->target_vel, true);
	return 0;
}

void StageNode::CalculateRobotControl(Stg::ModelPosition* mp, Stg::Velocity newVel, bool acceleration)
{
	//double dt = ((double)mp->GetInterval() / 1e6);
	double dt = (double)mp->GetWorld()->sim_interval / 1e6;

    if(acceleration)
    {
		Stg::Velocity vel = mp->GetVelocity();
		/// This velocity is used to fix 'integration' errors and make
		/// robot follow target velocity instead of oscilation near it
		Stg::Velocity fixedVel = vel;
		double acceleration[3] = {0,0,0};

		acceleration[0] = calculateControl<double>(newVel.x, fixedVel.x, dt, fabs(mp->acceleration_bounds[0].min), fabs(mp->acceleration_bounds[0].max));
		acceleration[1] = calculateControl<double>(newVel.y, fixedVel.y, dt, fabs(mp->acceleration_bounds[1].min), fabs(mp->acceleration_bounds[1].max));
		acceleration[2] = calculateControl<double>(newVel.a, fixedVel.a, dt, fabs(mp->acceleration_bounds[3].min), fabs(mp->acceleration_bounds[3].max));
		mp->SetVelocity(fixedVel);
		mp->SetAcceleration(acceleration[0], acceleration[1], acceleration[2]);
		ROS_DEBUG_NAMED("Acc", "Model %s: vt=%f v0=%f v1=%f acc=%f dt=%f", mp->Token(), newVel.x, vel.x, fixedVel.x, acceleration[0], dt);
    }
    else
    {
    	mp->SetSpeed(newVel);
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
    for (size_t r = 0; r < this->robotmodels_.size(); ++r)
    {
        StageRobot * robotmodel = this->robotmodels_[r];

        //loop on the laser devices for the current robot
        for (size_t s = 0; s < robotmodel->lasermodels.size(); ++s)
        {
            Stg::ModelRanger const* lasermodel = robotmodel->lasermodels[s];
            const std::vector<Stg::ModelRanger::Sensor>& sensors = lasermodel->GetSensors();

            if( sensors.size() > 1 )
                ROS_WARN( "ROS Stage currently supports rangers with 1 sensor only." );

            // for now we access only the zeroth sensor of the ranger - good
            // enough for most laser models that have a single beam origin
            const Stg::ModelRanger::Sensor& sensor = sensors[0];

            if( sensor.ranges.size() )
            {
                // Translate into ROS message format and publish
                sensor_msgs::LaserScan msg;
                msg.angle_min = -sensor.fov/2.0;
                msg.angle_max = +sensor.fov/2.0;
                msg.angle_increment = sensor.fov/(double)(sensor.sample_count-1);
                msg.range_min = sensor.range.min;
                msg.range_max = sensor.range.max;
                msg.ranges.resize(sensor.ranges.size());
                msg.intensities.resize(sensor.intensities.size());

                for(unsigned int i = 0; i < sensor.ranges.size(); i++)
                {
                    msg.ranges[i] = sensor.ranges[i];
                    msg.intensities[i] = sensor.intensities[i];
                }

                if (robotmodel->lasermodels.size() > 1)
                    msg.header.frame_id = mapName("base_laser_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    msg.header.frame_id = mapName("base_laser_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel));

                msg.header.stamp = sim_time;
                robotmodel->laser_pubs[s].publish(msg);
            }

            // Also publish the base->base_laser_link Tx.  This could eventually move
            // into being retrieved from the param server as a static Tx.
            Stg::Pose lp = lasermodel->GetPose();
            tf::Quaternion laserQ;
            laserQ.setRPY(0.0, 0.0, lp.a);
            tf::Transform txLaser =  tf::Transform(laserQ, tf::Point(lp.x, lp.y, robotmodel->positionmodel->GetGeom().size.z + lp.z));

            if (robotmodel->lasermodels.size() > 1)
                tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
                                                      mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                      mapName("base_laser_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel))));
            else
                tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
                                                      mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                      mapName("base_laser_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));
        }

        //the position of the robot
        tf.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(),
                                              sim_time,
                                              mapName("base_footprint", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                              mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

        // Get latest odometry data
        // Translate into ROS message format and publish
        nav_msgs::Odometry odom_msg;
        odom_msg.pose.pose.position.x = robotmodel->positionmodel->est_pose.x;
        odom_msg.pose.pose.position.y = robotmodel->positionmodel->est_pose.y;
        odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotmodel->positionmodel->est_pose.a);
        Stg::Velocity v = robotmodel->positionmodel->GetVelocity();
        odom_msg.twist.twist.linear.x = v.x;
        odom_msg.twist.twist.linear.y = v.y;
        odom_msg.twist.twist.angular.z = v.a;

        //@todo Publish stall on a separate topic when one becomes available
        //this->odomMsgs[r].stall = this->positionmodels[r]->Stall();
        //
        odom_msg.header.frame_id = use_common_root ?
        		this->root_frame_id : mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
        odom_msg.header.stamp = sim_time;
        odom_msg.child_frame_id = mapName("base_footprint", r, static_cast<Stg::Model*>(robotmodel->positionmodel));

        robotmodel->odom_pub.publish(odom_msg);

        // broadcast odometry transform
        tf::Quaternion odomQ;
        tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, odomQ);
        tf::Transform txOdom(odomQ, tf::Point(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0.0));
        tf.sendTransform(tf::StampedTransform(txOdom, sim_time,
                                              mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                              mapName("base_footprint", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));
        if(use_common_root)
        {
        	tf::Quaternion odomQ;
			tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, odomQ);
			tf::Transform txOdom = tf::Transform::getIdentity();
			tf.sendTransform(tf::StampedTransform(txOdom, sim_time,
												  root_frame_id,
												  mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));
        }

        // Also publish the ground truth pose and velocity
        Stg::Pose gpose = robotmodel->positionmodel->GetGlobalPose();
        tf::Quaternion q_gpose;
        q_gpose.setRPY(0.0, 0.0, gpose.a);
        tf::Transform gt(q_gpose, tf::Point(gpose.x, gpose.y, 0.0));
        /// Does anybody use it?
        // Velocity is 0 by default and will be set only if there is previous pose and time delta>0
        Stg::Velocity gvel(0,0,0,0);
        if (this->base_last_globalpos.size()>r){
            Stg::Pose prevpose = this->base_last_globalpos.at(r);
            double dT = (this->sim_time-this->base_last_globalpos_time).toSec();
            if (dT>0)
                gvel = Stg::Velocity(
                            (gpose.x - prevpose.x)/dT,
                            (gpose.y - prevpose.y)/dT,
                            (gpose.z - prevpose.z)/dT,
                            Stg::normalize(gpose.a - prevpose.a)/dT
                            );
            this->base_last_globalpos.at(r) = gpose;
        }else //There are no previous readings, adding current pose...
            this->base_last_globalpos.push_back(gpose);

        nav_msgs::Odometry ground_truth_msg;
        ground_truth_msg.pose.pose.position.x     = gt.getOrigin().x();
        ground_truth_msg.pose.pose.position.y     = gt.getOrigin().y();
        ground_truth_msg.pose.pose.position.z     = gt.getOrigin().z();
        ground_truth_msg.pose.pose.orientation.x  = gt.getRotation().x();
        ground_truth_msg.pose.pose.orientation.y  = gt.getRotation().y();
        ground_truth_msg.pose.pose.orientation.z  = gt.getRotation().z();
        ground_truth_msg.pose.pose.orientation.w  = gt.getRotation().w();
        ground_truth_msg.twist.twist.linear.x = gvel.x;
        ground_truth_msg.twist.twist.linear.y = gvel.y;
        ground_truth_msg.twist.twist.linear.z = gvel.z;
        ground_truth_msg.twist.twist.angular.z = gvel.a;

        ground_truth_msg.header.frame_id = mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
        ground_truth_msg.header.stamp = sim_time;

        ground_truth_msg.child_frame_id = mapName("base_footprint", r, static_cast<Stg::Model*>(robotmodel->positionmodel));

        robotmodel->ground_truth_pub.publish(ground_truth_msg);

        //cameras
        for (size_t s = 0; s < robotmodel->cameramodels.size(); ++s)
        {
            Stg::ModelCamera* cameramodel = robotmodel->cameramodels[s];
            // Get latest image data
            // Translate into ROS message format and publish
            if (robotmodel->image_pubs[s].getNumSubscribers() > 0 && cameramodel->FrameColor())
            {
                sensor_msgs::Image image_msg;

                image_msg.height = cameramodel->getHeight();
                image_msg.width = cameramodel->getWidth();
                image_msg.encoding = "rgba8";
                //this->imageMsgs[r].is_bigendian="";
                image_msg.step = image_msg.width*4;
                image_msg.data.resize(image_msg.width * image_msg.height * 4);

                memcpy(&(image_msg.data[0]), cameramodel->FrameColor(), image_msg.width * image_msg.height * 4);

                //invert the opengl weirdness
                int height = image_msg.height - 1;
                int linewidth = image_msg.width*4;

                char* temp = new char[linewidth];
                for (int y = 0; y < (height+1)/2; y++)
                {
                    memcpy(temp,&image_msg.data[y*linewidth],linewidth);
                    memcpy(&(image_msg.data[y*linewidth]),&(image_msg.data[(height-y)*linewidth]),linewidth);
                    memcpy(&(image_msg.data[(height-y)*linewidth]),temp,linewidth);
                }

                if (robotmodel->cameramodels.size() > 1)
                    image_msg.header.frame_id = mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    image_msg.header.frame_id = mapName("camera", r,static_cast<Stg::Model*>(robotmodel->positionmodel));
                image_msg.header.stamp = sim_time;

                robotmodel->image_pubs[s].publish(image_msg);
            }

            //Get latest depth data
            //Translate into ROS message format and publish
            //Skip if there are no subscribers
            if (robotmodel->depth_pubs[s].getNumSubscribers()>0 && cameramodel->FrameDepth())
            {
                sensor_msgs::Image depth_msg;
                depth_msg.height = cameramodel->getHeight();
                depth_msg.width = cameramodel->getWidth();
                depth_msg.encoding = this->isDepthCanonical?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
                //this->depthMsgs[r].is_bigendian="";
                int sz = this->isDepthCanonical?sizeof(float):sizeof(uint16_t);
                size_t len = depth_msg.width * depth_msg.height;
                depth_msg.step = depth_msg.width * sz;
                depth_msg.data.resize(len*sz);

                //processing data according to REP118
                if (this->isDepthCanonical){
                    double nearClip = cameramodel->getCamera().nearClip();
                    double farClip = cameramodel->getCamera().farClip();
                    memcpy(&(depth_msg.data[0]),cameramodel->FrameDepth(),len*sz);
                    float * data = (float*)&(depth_msg.data[0]);
                    for (size_t i=0;i<len;++i)
                        if(data[i]<=nearClip)
                            data[i] = -INFINITY;
                        else if(data[i]>=farClip)
                            data[i] = INFINITY;
                }
                else{
                    int nearClip = (int)(cameramodel->getCamera().nearClip() * 1000);
                    int farClip = (int)(cameramodel->getCamera().farClip() * 1000);
                    for (size_t i=0;i<len;++i){
                        int v = (int)(cameramodel->FrameDepth()[i]*1000);
                        if (v<=nearClip || v>=farClip) v = 0;
                        ((uint16_t*)&(depth_msg.data[0]))[i] = (uint16_t) ((v<=nearClip || v>=farClip) ? 0 : v );
                    }
                }

                //invert the opengl weirdness
                int height = depth_msg.height - 1;
                int linewidth = depth_msg.width*sz;

                char* temp = new char[linewidth];
                for (int y = 0; y < (height+1)/2; y++)
                {
                    memcpy(temp,&depth_msg.data[y*linewidth],linewidth);
                    memcpy(&(depth_msg.data[y*linewidth]),&(depth_msg.data[(height-y)*linewidth]),linewidth);
                    memcpy(&(depth_msg.data[(height-y)*linewidth]),temp,linewidth);
                }

                if (robotmodel->cameramodels.size() > 1)
                    depth_msg.header.frame_id = mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    depth_msg.header.frame_id = mapName("camera", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
                depth_msg.header.stamp = sim_time;
                robotmodel->depth_pubs[s].publish(depth_msg);
            }

            //sending camera's tf and info only if image or depth topics are subscribed to
            if ((robotmodel->image_pubs[s].getNumSubscribers()>0 && cameramodel->FrameColor())
                    || (robotmodel->depth_pubs[s].getNumSubscribers()>0 && cameramodel->FrameDepth()))
            {

                Stg::Pose lp = cameramodel->GetPose();
                tf::Quaternion Q; Q.setRPY(
                            (cameramodel->getCamera().pitch()*M_PI/180.0)-M_PI,
                            0.0,
                            lp.a+(cameramodel->getCamera().yaw()*M_PI/180.0) - robotmodel->positionmodel->GetPose().a
                            );

                tf::Transform tr =  tf::Transform(Q, tf::Point(lp.x, lp.y, robotmodel->positionmodel->GetGeom().size.z+lp.z));

                if (robotmodel->cameramodels.size() > 1)
                    tf.sendTransform(tf::StampedTransform(tr, sim_time,
                                                          mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                          mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel))));
                else
                    tf.sendTransform(tf::StampedTransform(tr, sim_time,
                                                          mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                          mapName("camera", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

                sensor_msgs::CameraInfo camera_msg;
                if (robotmodel->cameramodels.size() > 1)
                    camera_msg.header.frame_id = mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    camera_msg.header.frame_id = mapName("camera", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
                camera_msg.header.stamp = sim_time;
                camera_msg.height = cameramodel->getHeight();
                camera_msg.width = cameramodel->getWidth();

                double fx,fy,cx,cy;
                cx = camera_msg.width / 2.0;
                cy = camera_msg.height / 2.0;
                double fovh = cameramodel->getCamera().horizFov()*M_PI/180.0;
                double fovv = cameramodel->getCamera().vertFov()*M_PI/180.0;
                //double fx_ = 1.43266615300557*this->cameramodels[r]->getWidth()/tan(fovh);
                //double fy_ = 1.43266615300557*this->cameramodels[r]->getHeight()/tan(fovv);
                fx = cameramodel->getWidth()/(2*tan(fovh/2));
                fy = cameramodel->getHeight()/(2*tan(fovv/2));

                //ROS_INFO("fx=%.4f,%.4f; fy=%.4f,%.4f", fx, fx_, fy, fy_);


                camera_msg.D.resize(4, 0.0);

                camera_msg.K[0] = fx;
                camera_msg.K[2] = cx;
                camera_msg.K[4] = fy;
                camera_msg.K[5] = cy;
                camera_msg.K[8] = 1.0;

                camera_msg.R[0] = 1.0;
                camera_msg.R[4] = 1.0;
                camera_msg.R[8] = 1.0;

                camera_msg.P[0] = fx;
                camera_msg.P[2] = cx;
                camera_msg.P[5] = fy;
                camera_msg.P[6] = cy;
                camera_msg.P[10] = 1.0;

                robotmodel->camera_pubs[s].publish(camera_msg);

            }

        }
    }

    this->base_last_globalpos_time = this->sim_time;
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = sim_time;
    this->clock_pub_.publish(clock_msg);
}

int 
main(int argc, char** argv)
{ 
    if( argc < 2 )
    {
        puts(USAGE);
        exit(-1);
    }

    ros::init(argc, argv, "stageros");

    bool gui = true;
    bool use_model_names = false;
    for(int i=0;i<(argc-1);i++)
    {
        if(!strcmp(argv[i], "-g"))
            gui = false;
        if(!strcmp(argv[i], "-u"))
            use_model_names = true;
    }

    StageNode sn(argc-1,argv,gui,argv[argc-1], use_model_names);

    if(sn.SubscribeModels() != 0)
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
        if(gui)
            Fl::wait(r.expectedCycleTime().toSec());
        else
        {
            sn.UpdateWorld();
            r.sleep();
        }
    }

    t.join();

    //exit(0);
}
