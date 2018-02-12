/*
 * robotwrappwer.cpp
 *
 *  Created on: Dec 6, 2017
 *      Author: vrobot
 */

#include "robot.h"

const char* IMAGE = "image";
const char* DEPTH = "depth";
const char* CAMERA_INFO = "camera_info";
const char* ODOM = "odom";
const char* BASE_SCAN = "scan";
const char* CMD_VEL = "cmd_vel";
const char* BASE_POSE_GROUND_TRUTH = "ground_truth";

std::string getModelName(Stg::Model * mod, bool trim_parent)
{
	std::string result = mod->Token();
	Stg::Model * parent = mod->Parent();
	std::string parent_token = "";
	if(trim_parent && parent != NULL)
		parent_token = parent->Token();

	// Remove part from parent
	size_t pos = result.find(parent_token);
	if(pos != std::string::npos)
	{
		result.erase(0, pos);
	}
	// Erase extra symbols
	for(std::string::iterator it = result.begin(); it != result.end(); ++it)
	{
		if(*it == ':' || *it == '.')
		{
			*it = '_';
		}
	}
	return result;
}

StageRobot::StageRobot(const char* name)
{
	this->name = name;
	positionmodel = nullptr;
}

StageRobot::~StageRobot()
{
	if(positionmodel)
		positionmodel->Unsubscribe();
	for(int i = 0; i < this->lasermodels.size(); i++)
		lasermodels[i]->Unsubscribe();
	for(int i = 0; i < this->cameramodels.size(); i++)
		cameramodels[i]->Unsubscribe();
}

const char * StageRobot::getName() const
{
	return name.c_str();
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

void CalculateRobotControl(Stg::ModelPosition* mp, Stg::Velocity newVel, bool acceleration)
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

ros::Time StageRobot::getTime() const
{
	Stg::World * world = positionmodel->GetWorld();
	ros::Time sim_time;
	sim_time.fromSec(world->SimTimeNow() / 1e6);
	return sim_time;
}
/*
std::string StageRobot::mapName(const char * name) const
{
	std::string result = name;

	for(char& ch: result)
	{
		if(ch == ':' || ch == '.')
		{
			ch = '_';
		}
	}
	return name;
}
*/

std::string StageRobot::getTfFrame(const char* frame, int index) const
{
	std::stringstream ss;
	if(!tf_prefix.empty())
	{
		ss << tf_prefix.c_str() << "/";
	}
	ss << frame;
	if(index >= 0)
		ss << index;
	return ss.str();
}

std::string StageRobot::getTopicName(const char* topic, int index) const
{
	std::stringstream ss;
	if(!topic_ns.empty())
	{
		ss << topic_ns.c_str() << "/";
	}
	ss << topic;
	if(index >= 0)
		ss << index;
	return ss.str();
}

void StageRobot::initROS(ros::NodeHandle& nh, bool separate)
{
	if(separate)
	{
		tf_prefix = getName();
		topic_ns = getName();
	}
	odom_frame_id = getTfFrame("odom");
	base_link = getTfFrame("base_link");
	base_footprint = getTfFrame("base_footprint");

	odom_pub = nh.advertise<nav_msgs::Odometry>(getTopicName(ODOM), 10);
	ground_truth_pub = nh.advertise<nav_msgs::Odometry>(getTopicName(BASE_POSE_GROUND_TRUTH), 10);
	cmdvel_sub = nh.subscribe(getTopicName(CMD_VEL), 10, &StageRobot::onCmdVel, this);

	ROS_INFO("Found %lu laser devices and %lu cameras in robot %s", lasermodels.size(), cameramodels.size(), this->getName());

	bool collapseNames = true;
	/// Create publishers for laser models
	for (size_t s = 0;  s < lasermodels.size(); ++s)
	{
		const Stg::ModelRanger * lm = this->lasermodels[s];
		int index = (collapseNames && lasermodels.size() == 1) ? -1 : s;
		std::string topic_name = getTopicName(BASE_SCAN, index);
		ROS_INFO("Will publish laser sensor %s to topic name=%s", lm->Token(), topic_name.c_str());
		ros::Publisher pub;
		laser_pubs.push_back(nh.advertise<sensor_msgs::LaserScan>(topic_name, 10));
	}

	for (size_t s = 0;  s < cameramodels.size(); ++s)
	{
		Stg::Model * root = static_cast<Stg::Model*>(positionmodel);
		if (cameramodels.size() == 1)
		{
			image_pubs.push_back(nh.advertise<sensor_msgs::Image>(getTopicName(IMAGE), 10));
			depth_pubs.push_back(nh.advertise<sensor_msgs::Image>(getTopicName(DEPTH), 10));
			camera_pubs.push_back(nh.advertise<sensor_msgs::CameraInfo>(getTopicName(CAMERA_INFO), 10));
		}
		else
		{
			image_pubs.push_back(nh.advertise<sensor_msgs::Image>(getTopicName(IMAGE, s), 10));
			depth_pubs.push_back(nh.advertise<sensor_msgs::Image>(getTopicName(DEPTH, s), 10));
			camera_pubs.push_back(nh.advertise<sensor_msgs::CameraInfo>(getTopicName(CAMERA_INFO, s), 10));
		}
	}
}

int StageRobot::cb_model(Stg::ModelPosition * mod, StageRobot * sr)
{
	/// Apply acceleration control
	CalculateRobotControl(sr->positionmodel, sr->target_vel, true);
	return 0;
}

void StageRobot::onCmdVel(const geometry_msgs::Twist& msg)
{
//    boost::mutex::scoped_lock lock(msg_lock);
    Stg::ModelPosition * mp = positionmodel;
    assert(mp != NULL);

    Stg::Velocity newVel(msg.linear.x, msg.linear.y, 0.0, msg.angular.z);

    target_vel = newVel;
    base_last_cmd = getTime();
}

void StageRobot::setControlTimeout(float timeout)
{
	control_timeout = timeout;
}

void StageRobot::setPositionModel(Stg::ModelPosition * model)
{
	assert(model != NULL);
	assert(this->positionmodel == NULL);

	this->positionmodel = model;
	model->Subscribe();
	model->AddCallback(Stg::Model::CB_UPDATE, (Stg::model_callback_t)&StageRobot::cb_model, this);
}

void StageRobot::addLaser(Stg::ModelRanger * model)
{
	assert(model != NULL);
	lasermodels.push_back(model);
	model->Subscribe();
}

void StageRobot::addCamera(Stg::ModelCamera * model)
{
	assert(model != NULL);
	cameramodels.push_back(model);
	model->Subscribe();
}

void StageRobot::resetPose()
{
	positionmodel->SetPose(initial_pose);
	positionmodel->SetStall(false);

}

void StageRobot::publishData(PublishContext & context)
{
	publishOdom(context);
	publishLaser(context);
	publishCamera(context);
}

void StageRobot::publishLaser(PublishContext & context)
{
	ros::Time sim_time = context.time;

	bool singleSensor = lasermodels.size() <= 1;

	//loop on the laser devices for the current robot
	for (size_t s = 0; s < lasermodels.size(); ++s)
	{
		Stg::ModelRanger const* lasermodel = lasermodels[s];
		const std::vector<Stg::ModelRanger::Sensor>& sensors = lasermodel->GetSensors();

		if( sensors.size() > 1 )
			ROS_WARN( "ROS Stage currently supports rangers with 1 sensor only." );

		// for now we access only the zeroth sensor of the ranger - good
		// enough for most laser models that have a single beam origin
		const Stg::ModelRanger::Sensor& sensor = sensors[0];

		std::string frame_id = getTfFrame("laser_link", singleSensor? -1: s);

		if( sensor.ranges.size() )
		{
			// Translate into ROS message format and publish
			sensor_msgs::LaserScan msg;
			msg.header.frame_id = frame_id;
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

			msg.header.stamp = sim_time;
			laser_pubs[s].publish(msg);
		}

		// Also publish the base->base_laser_link Tx.  This could eventually move
		// into being retrieved from the param server as a static Tx.
		Stg::Pose lp = lasermodel->GetPose();
		tf::Quaternion laserQ;
		laserQ.setRPY(0.0, 0.0, lp.a);
		tf::Point center(lp.x, lp.y, lp.z + positionmodel->GetGeom().size.z);
		tf::Transform txLaser(laserQ, center);
		tf::StampedTransform transform(txLaser, sim_time, base_link, frame_id);
		context.tf.sendTransform(transform);
	}
}

void StageRobot::publishCamera(PublishContext & context)
{
	ros::Time sim_time = context.time;
	bool singleSensor = cameramodels.size() <= 1;
	//cameras
	for (size_t s = 0; s < cameramodels.size(); ++s)
	{
		Stg::ModelCamera* cameramodel = cameramodels[s];

		std::string frame_id = getTfFrame("camera_link", singleSensor? -1: s);
		// Get latest image data
		// Translate into ROS message format and publish
		if (image_pubs[s].getNumSubscribers() > 0 && cameramodel->FrameColor())
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

			image_msg.header.frame_id = frame_id;
			image_msg.header.stamp = sim_time;

			image_pubs[s].publish(image_msg);
		}

		//Get latest depth data
		//Translate into ROS message format and publish
		//Skip if there are no subscribers
		if (depth_pubs[s].getNumSubscribers()>0 && cameramodel->FrameDepth())
		{
			sensor_msgs::Image depth_msg;
			depth_msg.height = cameramodel->getHeight();
			depth_msg.width = cameramodel->getWidth();
			depth_msg.encoding = context.isDepthCanonical?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
			//this->depthMsgs[r].is_bigendian="";
			int sz = context.isDepthCanonical?sizeof(float):sizeof(uint16_t);
			size_t len = depth_msg.width * depth_msg.height;
			depth_msg.step = depth_msg.width * sz;
			depth_msg.data.resize(len*sz);

			//processing data according to REP118
			if (context.isDepthCanonical)
			{
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

			depth_msg.header.frame_id = frame_id;
			depth_msg.header.stamp = sim_time;
			depth_pubs[s].publish(depth_msg);
		}

		//sending camera's tf and info only if image or depth topics are subscribed to
		if ((image_pubs[s].getNumSubscribers()>0 && cameramodel->FrameColor())
						|| (depth_pubs[s].getNumSubscribers()>0 && cameramodel->FrameDepth()))
		{

			Stg::Pose lp = cameramodel->GetPose();
			tf::Quaternion Q; Q.setRPY((cameramodel->getCamera().pitch()*M_PI/180.0)-M_PI,
									0.0,
									lp.a+(cameramodel->getCamera().yaw()*M_PI/180.0) - positionmodel->GetPose().a
									);

			tf::Transform tr(Q, tf::Point(lp.x, lp.y, positionmodel->GetGeom().size.z+lp.z));
			tf::StampedTransform stamped_tr(tr, sim_time, base_link, frame_id);
			context.tf.sendTransform(stamped_tr);

			sensor_msgs::CameraInfo camera_msg;
			camera_msg.header.frame_id = frame_id;
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

			camera_pubs[s].publish(camera_msg);
		}
	}
}

void StageRobot::updateControl(const ros::Time& time)
{
	if(control_timeout > 0 && (time - base_last_cmd).toSec() >= control_timeout)
	{
		positionmodel->SetSpeed(0, 0, 0);
	}
}

void StageRobot::publishOdom(PublishContext & context)
{
	ros::Time sim_time = context.time;
  //the position of the robot
	tf::Transform transform = tf::Transform::getIdentity();
	tf::StampedTransform stamped(transform, sim_time, base_footprint, base_link);
  context.tf.sendTransform(stamped);

  // Get latest odometry data
  // Translate into ROS message format and publish
  nav_msgs::Odometry odom_msg;
  odom_msg.pose.pose.position.x = positionmodel->est_pose.x;
  odom_msg.pose.pose.position.y = positionmodel->est_pose.y;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(positionmodel->est_pose.a);

  // This velocity is not a real velocity. It is 'intended' velocity
  Stg::Velocity v = positionmodel->GetVelocity();
  odom_msg.twist.twist.linear.x = v.x;
  odom_msg.twist.twist.linear.y = v.y;
  odom_msg.twist.twist.angular.z = v.a;

  //@todo Publish stall on a separate topic when one becomes available
  //this->odomMsgs[r].stall = this->positionmodels[r]->Stall();
  //
  odom_msg.header.frame_id = context.root_frame_id.empty() ?  odom_frame_id: context.root_frame_id;
  odom_msg.header.stamp = sim_time;
  //static_cast<Stg::Model*>(positionmodel)
  odom_msg.child_frame_id = base_footprint;
  odom_pub.publish(odom_msg);

  // broadcast odometry transform
  tf::Quaternion odomQ;
  tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, odomQ);
  tf::Transform txOdom(odomQ, tf::Point(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0.0));
  tf::StampedTransform stamped_odom(txOdom, sim_time, odom_frame_id, base_footprint);
  context.tf.sendTransform(stamped_odom);

  if(!context.root_frame_id.empty())
  {
		tf::Transform zero_tf = tf::Transform::getIdentity();
		context.tf.sendTransform(
				tf::StampedTransform(zero_tf, sim_time, context.root_frame_id, odom_frame_id));
  }

  // Also publish the ground truth pose and velocity
  Stg::Pose gpose = positionmodel->GetGlobalPose();
  tf::Quaternion q_gpose;
  q_gpose.setRPY(0.0, 0.0, gpose.a);
  tf::Transform gt(q_gpose, tf::Point(gpose.x, gpose.y, 0.0));

  /// Does anybody use it?
  // Velocity is 0 by default and will be set only if there is previous pose and time delta>0
  Stg::Velocity gvel(0,0,0,0);

	Stg::Pose prevpose = this->base_last_globalpos;
	double dT = (sim_time-base_last_globalpos_time).toSec();
	if (dT>0)
	{
		gvel = Stg::Velocity(
								(gpose.x - prevpose.x)/dT,
								(gpose.y - prevpose.y)/dT,
								(gpose.z - prevpose.z)/dT,
								Stg::normalize(gpose.a - prevpose.a)/dT
								);
	}

	this->base_last_globalpos = gpose;
	this->base_last_globalpos_time = sim_time;
  //  this->base_last_globalpos.push_back(gpose);

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

  ground_truth_msg.header.frame_id = odom_frame_id;
  ground_truth_msg.header.stamp = sim_time;
  ground_truth_msg.child_frame_id = base_footprint;

  ground_truth_pub.publish(ground_truth_msg);
}
