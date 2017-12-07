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

#include "stageros.h"

StageNode::StageNode(Stg::World * world, bool use_model_names)
{
	this->use_acceleration_control = false;
	this->persistent_world = false;
	this->use_model_names = use_model_names;
	this->sim_time.fromSec(0.0);
	this->map_publish_period = -1.0;
	this->map_resolution = 0.05;
	this->map_generated = false;

	exportMode = ExportSimple;

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

	if(localn.hasParam("export"))
	{
		XmlRpc::XmlRpcValue val;
		localn.getParam("export", val);
		if(val.getType() == XmlRpc::XmlRpcValue::TypeInt)
		{
			this->exportMode = (enum StageExportMode)(int)val;
		}
	}

	localn.param<std::string>("map_model_name", map_model_name, "ground");

	if(!persistent_file.empty())
		persistent_world = true;

	this->world = world;
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
		this->ExportModels(models);
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
int StageNode::ExportModels(SceneModels & models)
{
	n_.setParam("/use_sim_time", true);

	for (size_t r = 0; r < models.positionmodels.size(); r++)
	{
		Stg::ModelPosition * positionmodel = models.positionmodels[r];
		std::string name = getModelName(positionmodel, false);

		ROS_INFO("Found robot name=%s", name.c_str());
		StageRobotPtr robot;

		if(robotmodels_.find(name) == robotmodels_.end())
		{
			// Node handle for the robot
			ros::NodeHandle nh = exportMode == ExportSimple ? n_ : ros::NodeHandle(n_, name);
			// Create the robot for export
			robot.reset(new StageRobot(nh, name.c_str()));
			robotmodels_[name] = robot;
		}
		else
		{
			ROS_ERROR("Entity with name=%s already exists. Cannot attach another position model", name.c_str());
			continue;
		}

		robot->setPositionModel(positionmodel);

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

		if(exportMode == ExportSimple)
			break;
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

bool StageNode::UpdateWorld()
{
    return world->Update();
}

bool StageNode::cb_getmap_srv(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
	if(!map_generated)
		generateMap(cached_map_);
	res.map = this->cached_map_;
	return true;
}

bool StageNode::generateMap(nav_msgs::OccupancyGrid & map)
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

	map_generated = true;

	return true;
}

void StageNode::onMapPublish(const ros::TimerEvent & event)
{
	if(map_generated || generateMap(cached_map_))
	{
		map_pub_.publish(this->cached_map_);
		map_info_pub_.publish(this->cached_map_.info);
	}
}

bool StageNode::s_update(Stg::World* world, StageNode* node)
{
	node->WorldCallback();
	// We return false to indicate that we want to be called again (an
	// odd convention, but that's the way that Stage works).
	return false;
}

void StageNode::WorldCallback()
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

