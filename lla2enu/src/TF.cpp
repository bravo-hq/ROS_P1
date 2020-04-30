#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Vector3Stamped.h"
#include <math.h>
#include "nav_msgs/Odometry.h"

class tf_sub_pub
{
public:
	tf_sub_pub()
	{
		sub1 = n.subscribe("/front_car/front_data", 1000, &tf_sub_pub::callback1, this);
		sub2 = n.subscribe("/obs_car/obs_data", 1000, &tf_sub_pub::callback2, this);
		odom_pub1 = n.advertise<nav_msgs::Odometry>("odom_front", 1000);
		odom_pub2 = n.advertise<nav_msgs::Odometry>("odom_obs", 1000);
	}

	void callback1(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
	{
		transform.setOrigin(tf::Vector3(msg->vector.x, msg->vector.y, msg->vector.z));
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "car_transform"));

		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "world";
		odom.pose.pose.position.x = msg->vector.x;
		odom.pose.pose.position.y = msg->vector.y;
		odom.pose.pose.position.z = msg->vector.z;
		odom.child_frame_id = "car_transform";
		odom_pub1.publish(odom);
	}

	void callback2(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
	{

		transform.setOrigin(tf::Vector3(msg->vector.x, msg->vector.y, msg->vector.z));
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "obstacle_transform"));

		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "world";
		odom.pose.pose.position.x = msg->vector.x;
		odom.pose.pose.position.y = msg->vector.y;
		odom.pose.pose.position.z = msg->vector.z;
		odom.child_frame_id = "obstacle_transform";
		odom_pub2.publish(odom);
	}

private:
	ros::NodeHandle n;
	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	nav_msgs::Odometry odom;
	ros::Subscriber sub1;
	ros::Subscriber sub2;
	ros::Publisher odom_pub1;
	ros::Publisher odom_pub2;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tf_publisher");

	tf_sub_pub my_tf_sub_bub;

	ros::spin();

	return 0;
}
