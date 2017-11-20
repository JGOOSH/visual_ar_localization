#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Twist.h"
#include <map>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "Eigen/Dense"
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <iostream>
#include <fstream>

bool markerSeen = false;

visualization_msgs::Marker current_vis_msg;

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

    //Internal Data Structure
    std::map<int, geometry_msgs::PoseStamped> pose_map;
    std::map<int, geometry_msgs::PoseStamped>::iterator it;

   	geometry_msgs::PoseStamped stampedPose;
    geometry_msgs::PoseStamped tag_wresp_map;

    while(ros::ok()) {
    	ros::spinOnce();
    	if(markerSeen) {
	        it = pose_map.find(current_vis_msg.id);
	        if(it == pose_map.end()) {
				stampedPose.header = current_vis_msg.header;
				stampedPose.pose = current_vis_msg.pose;
	        	try {
	        		tf_l.waitForTransform("/map", stampedPose.header.frame_id, ros::Time(0), ros::Duration(1));
	        		tf_l.transformPose("/map", stampedPose, tag_wresp_map);
			    }
			    catch (tf::TransformException ex) {
			        ROS_ERROR("TransformException");
			    }
	          	//ROS_INFO("Current tag is at x : %f, y : %f, z : %f", tag_wresp_map.pose.position.x, tag_wresp_map.pose.position.y, tag_wresp_map.pose.position.z);
	          	pose_map.insert(it, std::pair<int, geometry_msgs::PoseStamped>(current_vis_msg.id , tag_wresp_map));
	        }		    	
	        else {

		    	//Check if the AR Tag wrt map is within a certain threshold of our first location for the Tag


		    	//Calculate AR Tag position q^-1 p q

		    	//Calculate Robot position

		    	//Check if norm of (AR Pos - Robot Pos) < Treshold

		    		//If "" then print out to file the location of the robot


		    	//Record the Robot's Location in our log file

	        	//Log file will be full of 

				ROS_INFO("The robot is at : %f, y : %f, z : %f", arPose_wrt_robot.x() - arPose_wrt_map.x(), arPose_wrt_robot.y() - arPose_wrt_map.y(), arPose_wrt_robot.z() - arPose_wrt_map.z());
		    	}
	    	markerSeen = false;
	    }
	}
}