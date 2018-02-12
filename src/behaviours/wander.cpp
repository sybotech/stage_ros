/*
 * wander.cpp
 *
 *  Created on: Feb 9, 2018
 *      Author: vrobot
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <algorithm>

/*
 * Wanderer behaviour
 * Subscribes to scan and odom and controls the robot through cmd_vel
 *
 * 1. Move forward until obstacle is near (scanHandler found some obstacle)
 * 2. Stop completely
 * 3. Turn to random direction for random time
 * 4. Stop completely
 * 5. If obstacle - goto 3 else goto 1
 */

const double minLinearVel = 0.01;
const double minAngularVel = 0.01;

// Size of the robot
double robotDiameter = 0.3;
// Multiplier for turning speed
double turnDirection = 0;
// Limit for turning time
double turningTime = 4.0;
// Limit for turning speed
double turnSpeed = 0.3;

double cruiseSpeed = 0.4;

// Distance to the nearest obstacle
double obstacleDistance = 10.0;
// Distance when we begin to stop and find another direction
double stopDistance = 1.0;
ros::Time obstacleTime;

// Time for the next state change
ros::Time stateEnd;

ros::Publisher pub_vel;

// Publisher for the obstacle
ros::Publisher pub_obstacle;

// State of controller
enum class State
{
	Initial,
	Stopping,				//< Robot is reducing its velocity to zero
	StandStill,			//< Robot is standing still
	MoveForward, 		//< Moving forward until we approach some obstacle
	RandomTurn,			//< Turning in random direction
};

State state = State::Initial;

void changeState(State newState)
{
	state = newState;
}

void changeStateTimed(State newState, ros::Time limit)
{
	state = newState;
	stateEnd = limit;
	ros::Duration delta = limit - ros::Time::now();
	ROS_INFO("State will last for %fsec", delta.toSec());
}

nav_msgs::Odometry last_odom;

void odomHandler(const nav_msgs::Odometry& odom)
{
	last_odom = odom;

	switch(state)
	{
	case State::Stopping:
		if (abs(odom.twist.twist.linear.x) < minLinearVel &&
				abs(odom.twist.twist.angular.z) < minAngularVel)
		{
			ROS_INFO("Reduced velocity to zero");
			changeState(State::StandStill);
		}
		break;
	case State::StandStill:
		break;
	case State::MoveForward: 	//< Moving forward until we approach some obstacle
		break;
	case State::RandomTurn:		//< Turning in random direction
		break;
	}
}

void scanHandler(const sensor_msgs::LaserScan& scan)
{
	float angle = scan.angle_min;

	geometry_msgs::PoseStamped msg;
	msg.header = scan.header;
	bool hasObstacle = false;
	float obstacleAngle = 0;

	// Minimal distance to the obstacle
	float minDistance = 100000;
	for (uint32_t i = 0; i < scan.ranges.size(); i++, angle+=scan.angle_increment)
	{
		float range = scan.ranges[i];

		float x = range * cos(angle);
		float y = range * sin(angle);

		if(fabs(y) < robotDiameter*0.5)
		{
			if (x < minDistance && x > 0.0)
			{
				minDistance = x;
				hasObstacle = true;
				msg.pose.position.x = x;
				msg.pose.position.y = y;
				obstacleAngle = angle;
			}
		}
	}

	obstacleDistance = minDistance;
	obstacleTime = scan.header.stamp;

	if(hasObstacle)
	{
		msg.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
		pub_obstacle.publish(msg);
	}
};

double random(double min, double max)
{
	double delta = fabs(max - min);
	return min + delta * (rand()%10000) * 0.0001;
}

ros::Duration randomDuration(float min, float max)
{
	return ros::Duration(random(min, max));
}

double randomDirection()
{
	double dir = random(-1.0, 1.0);
	if(dir < 0)
		return -1.0;
	return 1.0;
}

void onTimer(const ros::TimerEvent& event)
{
	geometry_msgs::Twist cmd_vel;
	ros::Time now = event.current_expected;

	switch(state)
	{
	case State::Initial:
		ROS_INFO("Initial->Stopping");
		changeState(state = State::Stopping);
		break;
	case State::Stopping:
		cmd_vel.angular.z = 0;
		cmd_vel.linear.x = 0;
		break;
	case State::StandStill:
		if(obstacleDistance < stopDistance)
		{
			ROS_INFO("Can not move forward. Turning");
			// Should turn randomly
			turnDirection = randomDirection();
			changeStateTimed(State::RandomTurn, now + randomDuration(turningTime*0.5, turningTime));
		}
		else
		{
			ROS_INFO("Can move forward");
			changeState(State::MoveForward);
		}
		break;
	case State::MoveForward: 		//< Moving forward until we approach some obstacle
		if(obstacleDistance < stopDistance)
		{
			ROS_INFO("Stopping before obstacle at range %f", obstacleDistance);
			changeState(State::Stopping);
		}
		else
		{
			cmd_vel.linear.x = cruiseSpeed;
		}
		break;
	case State::RandomTurn:			//< Turning in random direction
		if(obstacleDistance > stopDistance)
		{
			ROS_INFO("Found gap to move");
			changeState(State::Stopping);
		}
		else if(now < stateEnd)
		{
			cmd_vel.angular.z = turnDirection*turnSpeed;
			cmd_vel.linear.x = 0;
		}
		else
		{
			ROS_INFO("Done turning");
			changeState(State::Stopping);
		}
		break;
	}

	pub_vel.publish(cmd_vel);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "wanderer");

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	private_nh.param<double>("diameter", robotDiameter, 0.3);
	private_nh.param<double>("stop_distance", stopDistance, 1.0);
	private_nh.param<double>("turn_time", turningTime, 2.0);
	private_nh.param<double>("turn_speed", turnSpeed, 0.4);
	private_nh.param<double>("cruise_speed", cruiseSpeed, 0.4);

	pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
	pub_obstacle = nh.advertise<geometry_msgs::PoseStamped>("obstacle", 5);

	ros::Subscriber scan_sub = nh.subscribe("scan", 10, &scanHandler);
	ros::Subscriber odom_sub = nh.subscribe("odom", 10, &odomHandler);
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), &onTimer);

	ROS_INFO("Started wanderer controller for robot=%s", nh.getNamespace().c_str());
	ros::spin();

	return 0;
}
