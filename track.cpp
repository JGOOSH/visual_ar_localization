#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Twist.h"

bool markerSeen = false;

//HashSet K,V --> ID, Location

visualization_msgs::Marker current_vis_msg;

void vis_cb(const visualization_msgs::Marker::ConstPtr& msg) {
    current_pose = *msg;
    markerSeen = true;
}

int main(int argc, char **argv){
    
    //initialize the node
    ros::init(argc, argv, "traveling_salesturtle");
    
    //instantiate the node handle which is used for creating publishers and subscribers
    ros::NodeHandle n;

    //subscriber for the /visualization_marker
    ros::Subscriber vis_sub = n.subscribe("/visualization_marker", 1, vis_cb );

    //your code for solving the traveling salesmane problem goes here (of course you can define helper functions above the main function)
    while(ros::ok()) {
    	ros::spinOnce();
    	if(markerSeen)
    	{
    		for(int count = 0; count < sizeof(x_coords)/sizeof(x_coords[0]); count++)
    		{
    			goToLoc(x_coords[count], y_coords[count]);
    		}
    		markerSeen = false;
    		break;
    	}
	}
}