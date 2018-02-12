/*
 * main.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: vrobot
 */

#include "stageros.h"

#ifdef HAS_STAGE_GUI
#include "world_gui.hh"
#endif

#define USAGE "stageros <worldfile>"

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

	Stg::World * world = NULL;

#ifdef HAS_STAGE_GUI
	if(gui)
		world = new Stg::WorldGui(600, 400, "Stage (ROS)");
	else
#endif
	{
		world = new Stg::World();
	}

	const char * fname = argv[argc-1];

	StageNode sn(world, use_model_names);

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
		{
			Fl::wait(r.expectedCycleTime().toSec());
		}
		else
#endif
		{
			sn.UpdateWorld();
			r.sleep();
		}
	}

	ros::shutdown();
	t.join();
}
