#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include "lla2enu/dist.h"
#include <dynamic_reconfigure/server.h>
#include <lla2enu/parametersConfig.h>
#include <sstream>
#include <string>
#include "lla2enu/calculation.h"

class Client
{
    lla2enu::dist messagio;
    int higher_limit;
    int lower_limit;

private:
    ros::NodeHandle n;
    ros::Publisher pub;
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub1;
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub2;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    ros::ServiceClient client;
    lla2enu::calculation srv;
    dynamic_reconfigure::Server<lla2enu::parametersConfig> param_server;
    dynamic_reconfigure::Server<lla2enu::parametersConfig>::CallbackType f;

public:
    Client()
    {
        sub1.subscribe(n, "/front_car/front_data", 10);
        sub2.subscribe(n, "/obs_car/obs_data", 10);
        pub = n.advertise<lla2enu::dist>("dist_data", 1000);
        client = n.serviceClient<lla2enu::calculation>("calculation_Server");
        f = boost::bind(&Client::callback1, this, _1, _2);
        param_server.setCallback(f);
        lla2enu::parametersConfig config;
        param_server.getConfigDefault(config);
        param_server.updateConfig(config);
        sync.reset(new Sync(MySyncPolicy(10), sub1, sub2));
        sync->registerCallback(boost::bind(&Client::callback, this, _1, _2));
    }

    void callback(const geometry_msgs::Vector3StampedConstPtr &msg1, const geometry_msgs::Vector3StampedConstPtr &msg2)
    {
        ROS_INFO("recieved a message");

        srv.request.Car = *msg1;
        srv.request.Obs = *msg2;

        ROS_INFO("msg has been set %f", srv.request.Obs.vector.x);

        if (client.call(srv))
        {
            double dist = srv.response.dist;
            ROS_INFO("inside the call");
            if (isnan(dist))
            {
                messagio.dist = NAN;
                messagio.flag = "GPS data missing";
            }
            else
            {
                messagio.dist = dist;
                if (dist >= higher_limit)
                {
                    messagio.flag = "safe";
                }
                else if (dist > lower_limit && dist < higher_limit)
                {
                    messagio.flag = "unsafe";
                }
                else if (dist <= lower_limit)
                {
                    messagio.flag = "crash";
                }
            }
            pub.publish(messagio);
        }
    }

    void callback1(lla2enu::parametersConfig &config, uint32_t level)
    {
        lower_limit = config.lower_limit;
        higher_limit = config.higher_limit;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber_sync");

    Client cal;

    ros::spin();

    return 0;
}
