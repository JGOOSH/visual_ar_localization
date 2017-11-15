#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Twist.h"
#include <map>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

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
	          	ROS_INFO("Current tag is at x : %f, y : %f, z : %f", tag_wresp_map.pose.position.x, tag_wresp_map.pose.position.y
	          			, tag_wresp_map.pose.position.z);
	          	pose_map.insert(it, std::pair<int, geometry_msgs::PoseStamped>(current_vis_msg.id , tag_wresp_map));
	        }
	        else {
	        	geometry_msgs::PoseStamped curPose = pose_map.at(current_vis_msg.id);
		    	ROS_INFO("This pre-existing tag is at x : %f, y : %f, z : %f", curPose.pose.position.x, curPose.pose.position.y, curPose.pose.position.z);
		    	//Build first 4x4 Matrix
		    	Eigen::Matrix4f tag_wrt_map;
		    	tag_wrt_map.block(0, 0, 3, 3) = curpose.pose.orientation.toRotationMatrix();
		    	tag_wrt_map.row(3) << 0, 0, 0, 1;
		    	tag_wrt_map(0, 3) = curpose.pose.position.x;
		    	tag_wrt_map(1, 3) = curpose.pose.position.y;
		    	tag_wrt_map(2, 3) = curpose.pose.position.z;

		    	//Build second 4x4 Matrix
		    	geometry_msgs::Pose ps = current_vis_msg.pose;
		    	Eigen::Matrix4f tag_wrt_seg;
		    	tag_wrt_seg.block(0, 0, 3, 3) = ps.orientation.toRotationMatrix();
		    	tag_wrt_seg.row(3) << 0, 0, 0, 1;
		    	tag_wrt_seg(0, 3) = ps.position.x;
		    	tag_wrt_seg(1, 3) = ps.position.y;
		    	tag_wrt_seg(2, 3) = ps.position.z;
		    	
		    	//Compute ending Matrix
		    	Eigen::Matrix4f robotPosMat = (tag_wrt_seg.inverse()) * tag_wrt_map;
		    	//Extract Pose object out of result

		    	//Public result
	    	}
	    	markerSeen = false;
	    }
	}
}
