#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Twist.h"
#include <map>

bool markerSeen = false;

visualization_msgs::Marker current_vis_msg;

void vis_cb(const visualization_msgs::Marker::ConstPtr& msg) {
    current_vis_msg = *msg;
    markerSeen = true;
}

int main(int argc, char **argv){

    //initialize the node
    ros::init(argc, argv, "ar_tag_tracker");

    //instantiate the node handle which is used for creating publishers and subscribers
    ros::NodeHandle n;

    //subscriber for the /visualization_marker
    ros::Subscriber vis_sub = n.subscribe("/visualization_marker", 1, vis_cb);

    std::map<int, geometry_msgs::Pose> pose_map;
    std::map<int, geometry_msgs::Pose>::iterator it;

    //your code for solving the traveling salesmane problem goes here (of course you can define helper functions above the main function)
    while(ros::ok()) {
    	ros::spinOnce();
        //CREATE DS that holds all the tags and their positions
    	if(markerSeen)
    	{
        it = pose_map.find(current_vis_msg.id);
        if(it == pose_map.end()){
          ROS_INFO("Current tag is at x : %f, y : %f, z : %f", current_vis_msg.pose.position.x, current_vis_msg.pose.position.y
          , current_vis_msg.pose.position.z);
          pose_map.insert(it, std::pair<int, geometry_msgs::Pose>(current_vis_msg.id , current_vis_msg.pose));
        }
        else {
          ROS_INFO("ALREADY EXISTS"); 
        }
    		markerSeen = false;
    	}
	}
}
