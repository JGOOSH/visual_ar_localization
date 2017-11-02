#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Twist.h"

bool markerSeen = false;

//HashSet K,V --> ID, Location

visualization_msgs::Marker current_vis_msg;
// geometry_msgs::Pose current_pose;
// geometry_msgs::Vector3 pose_direction;

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

    //your code for solving the traveling salesmane problem goes here (of course you can define helper functions above the main function)
    while(ros::ok()) {
    	ros::spinOnce();
        //CREATE DS that holds all the tags and their positions
    	if(markerSeen)
    	{
            ROS_INFO("Current tag is at x : %f, y : %f, z : %f", current_vis_msg.pose.position.x, current_vis_msg.pose.position.y
                , current_vis_msg.pose.position.z);
    		markerSeen = false;
    	}
	}
}