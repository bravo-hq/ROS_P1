Group members:

Yousef Sadeghieh 10700751
Shayan Shokri 10708757
Kimia Mesghali  10721013

Summary of the project:

     In this project, The ENU position of both car and obstacle is calculated with the help of the bag file consist of GPS data. Then, the distance between them is published. In addition, the transfer function between frames and the odometry of the car and the obstacle are published.

Folders:

In SRC folder, 3 files are defined : sub_front, distance_cal ,TF.
    1. Sub_front: 
  	  • this node subscribes from bag data and publish ENU position.
  	  • In case the GPS data is zero, it publishes NAN
     2.Service_client: 
  	  • It subscribes to car_data and obs_data (which is ENU position of them) by filter message and then sends the data to the server to calculate the distance and after receiving the data from the server, it publishes data using the preset custom message we made as well as the flag related to the condition. Also, it uses dynamic reconfiguration to set dynamic thresholds.  
  	  • In case the GPS data is zero.NAN is published in the distance with the flag "GPS data missing."
     3.service_server:
          • It calculates the distance between the data received from the service client.
          
     4.TF:
  	  • relationships between coordinate frames obstacle and car and the world(in callback1 and callback2).
  	  • publishing both odometry of car and obstacle(in callback1 and callback2).

       Launch folder:
  	  • the run.launch is in this folder to run all the nodes at the same time. 
  	  • ENU origin parameter is initialized in the launch file.
           In case the dynamic reconfiguration is not used, the default values for thresholds are defined in the launch file( by default, the parameters are commented in the launch file).  

Name of parameters:

	latitude_init, longitude_init, and h0 are the parameters defining the ENU origin in the launch file. Also, dynamic reconfiguration has been implemented to change the threshold of the safe and unsafe conditions of the flag. (lower_threshold and higher_threshold in case of using default parameters for thresholds)

structure of service:
        Name: calculation.srv
        Type: geometry_msgs/vector3Stamped (request) and Float64 (responce)

structure of custom messages: 

	Name: dist.msg
	Type: Dist(float64 and string)

structure of tf tree:

	The fixed transform frame is called "world" and the other two transfer functions for the car and the
obstacle are called "car_transform" and "obstacle_transform", respectively. These two transfer functions are independent of each other.

How to Start Nodes:
	for starting all the nodes, it is just needed to start a run.launch file by this command:
	$ roslaunch lla2enu run.launch 
		it consists of 5 nodes:
		front_car: converts lla to enu for car.
		obs_car: converts lla to enu for obstcle.
		service_server: calculates distance between car and obstacle.
                service_client: publishes the custom message.
		TF_broadcaster: calculates the relationship between frames and odometry of frames. 

	*ROSBAG file is in the same directory as the package and it will start by launching the launch file.


