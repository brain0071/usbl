#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "usbl.h"



// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "location_node");
//     ros::NodeHandle nh;
//     ros::Publisher usbl_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/location/xy_depth", 1000);

//     // get beacon_ID
//     int beacon_ID = nh.getParam("/beacon_ID", beacon_ID);
//     // intial usbl
//     usbl u(beacon_ID);
//     // u.open_port();

//     ros::Rate usbl_rate(1);
//     ros::Rate pos_update_rate(1);
//     while (ros::ok())
//     {
//         ros::spinOnce();
//         // run usbl
//         u.run();
//         usbl_rate.sleep();
//     }

//     while (ros::ok())
//     {
//         ros::spinOnce();
//         // get pos
//         float *cur_pos;
//         cur_pos = u.get_pos();
//         geometry_msgs::Vector3Stamped v;
//         v.vector.x = cur_pos[0];
//         v.vector.y = cur_pos[1];
//         v.vector.z = cur_pos[2];
//         // send usbl position
//         usbl_pub.publish(v);
//         ROS_INFO("Current position is %f, %f, %f", cur_pos[0], cur_pos[1], cur_pos[2]);
//         pos_update_rate.sleep();
//     }
// }