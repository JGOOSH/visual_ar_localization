#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Twist.h"
#include <map>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include "Eigen/Dense"
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <iostream>
#include <fstream>

bool markerSeen = false;

visualization_msgs::Marker current_vis_msg;
visualization_msgs::MarkerArray marker_array_msg;

void vis_cb(const visualization_msgs::Marker::ConstPtr& msg) {
    current_vis_msg = *msg;
    markerSeen = true;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "ar_tag_tracker");
	ros::NodeHandle n;
    tf::TransformListener tf_l;

    //subscriber for the /visualization_marker
    ros::Subscriber vis_sub = n.subscribe("/visualization_marker", 1, vis_cb);
    //publisher for the markers on the map
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("map_markers", 10);

    //HOLDS ID->Pose pairs
    //Internal Data Structure
    std::map<int, geometry_msgs::PoseStamped> pose_map;
    std::map<int, geometry_msgs::PoseStamped>::iterator it;

   	geometry_msgs::PoseStamped stampedPose;

    //output stampedPose
    geometry_msgs::PoseStamped tag_wresp_map;

    while(ros::ok()) {
    	ros::spinOnce();
    	if(markerSeen) {
	      	  	//Transfer information into a StampedPose
	  			stampedPose.header = current_vis_msg.header;
	  			stampedPose.pose = current_vis_msg.pose;
	        	try {
	        		tf_l.waitForTransform("/map",
	                              stampedPose.header.frame_id, ros::Time(0), ros::Duration(1));
	        		//Performs transformation and stores it in the third parameter
	        		tf_l.transformPose("/map", stampedPose, tag_wresp_map);
	        		//JAMIN TAKES tag_wresp_map and annotates the map with its
		        }
			    catch (tf::TransformException ex) {}
	          ROS_INFO("%d, %f, %f, %f", current_vis_msg.id, tag_wresp_map.pose.position.x, tag_wresp_map.pose.position.y
	          , tag_wresp_map.pose.position.z);
	          markerSeen = false;
	    }
	}
}