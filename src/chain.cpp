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

bool marker_seen = false;
bool first_seen = false;
geometry_msgs::Pose first_pose = NULL;
visualization_msgs::Marker current_vis_msg;
Eigen::Matrix4f mats_arr[50];
int next_tag_id = 0;

void vis_cb(const visualization_msgs::Marker::ConstPtr& msg) {
    current_vis_msg = *msg;
    marker_seen = true;
}

Eigen::Matrix4f getMatFromPose() {
	Eigen::Matrix4f temp;
	temp.block(0, 0, 2, 2) = current_vis_msg.pose.orientation.toRotationMatrix();
	temp(0, 3) = current_vis_msg.pose.position.x;
	temp(1, 3) = current_vis_msg.pose.position.y;
	temp(2, 3) = current_vis_msg.pose.position.z;
	temp(3, 3) = 1;
	return temp;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "chain");
	ros::NodeHandle n;
    //tf::TransformListener tf_l;

    //subscriber for the /visualization_marker
    ros::Subscriber vis_sub = n.subscribe("/visualization_marker", 1, vis_cb);

    while(ros::ok()) {
    	ros::spinOnce();
    	if(marker_seen) {
    		if(current_vis_msg.id == next_tag_id) {
	    		if(!first_seen) {
	    			first_pose = current_vis_msg.pose;
	    			first_seen = true;
	    			mats_arr[0] = getMatFromPose();
	    		} else {
	    			Eigen::Matrix4f before = mats_arr[next_tag_id - 1];
	    			Eigen::Matrix4f Mn = getMatFromPose();
	    			mats_arr[next_tag_id] = before.inverse() * Mn;
	    		}
	    		next_tag_id++;
	    	}
		}
	}
}