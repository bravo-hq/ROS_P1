#include "ros/ros.h"
#include "lla2enu/calculation.h"
#include "geometry_msgs/Vector3Stamped.h"

bool calculation(lla2enu::calculation::Request &req,
         lla2enu::calculation::Response &res)
{
    ROS_INFO("in server function");
    double deltax = req.Car.vector.x - req.Obs.vector.x;
    double deltay = req.Car.vector.y - req.Obs.vector.y;
    double deltaz = req.Car.vector.z - req.Obs.vector.z;
    res.dist = sqrt(pow(deltax, 2) + pow(deltay, 2) + pow(deltaz, 2));
    ROS_INFO("distance = %f",res.dist);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cal_server");
    ros::NodeHandle n("~");
    ROS_INFO("server started");
    ros::ServiceServer service = n.advertiseService("Calculate", calculation);
    //   ROS_INFO("Ready to add two ints.");
    ros::spin();

    return 0;
}
