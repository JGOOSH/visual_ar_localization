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
		    	//Last value of the quaternion is 0 because we converted the points into a quaternion
				Quaternion pQuat = new Quaternion(current_vis_msg.pose.position.x, current_vis_msg.pose.position.y, current_vis_msg.pose.position.z, 0);

				Quaternion arPose_wrt_robot = current_vis_msg.pose.orientation.inverse();
				arPose_wrt_robot*= pQuat;
				arPose_wrt_robot*= current_vis_msg.pose.orientation;

				geometry_msgs::Pose curPose = pose_map.at(current_vis_msg.id).pose;
				Quaternion pARTag = new Quaternion(curPose.position.x, curPose.position.y, curPose.pose.position.z, 0);
				
				Quaternion arPose_wrt_map = curPose.orientation.inverse();
				arPose_wrt_map*= pARTag;
				arPose_wrt_map*= curPose.orientation;

		    	Quaternion result = arPose_wrt_robot - arPose_wrt_map;
		    	//Grab xyz of result, just do result.x ....

		    	geometry_msgs::Pose outputPose;
		    	outputPose.position.x = result.x;
		    	outputPose.position.y = result.y;
		    	outputPose.position.z = result.z;

		    	outputPose.orientation.x = 0;
		    	outputPose.orientation.y = 0;
		    	outputPose.orientation.z = 0;
		    	outputPose.orientation.w = 1;

				ROS_INFO("The robot is at : %f, y : %f, z : %f", outputPose.position.x, outputPose.position.y, outputPose.position.z);
		    	}
	    	markerSeen = false;
	    }
	}
}
