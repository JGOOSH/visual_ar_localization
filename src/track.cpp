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
				Eigen::Quaternionf pQuat;
				pQuat.x() = current_vis_msg.pose.position.x;
				pQuat.y() = current_vis_msg.pose.position.y;
				pQuat.z() = current_vis_msg.pose.position.z;
				pQuat.w() = 0;


				Eigen::Quaternionf tempQuat;
				tempQuat.x() = current_vis_msg.pose.orientation.x;
				tempQuat.y() = current_vis_msg.pose.orientation.y;
				tempQuat.z() = current_vis_msg.pose.orientation.z;
				tempQuat.w() = current_vis_msg.pose.orientation.w;

				Eigen::Quaternionf arPose_wrt_robot = tempQuat.inverse();
				arPose_wrt_robot*= pQuat;
				arPose_wrt_robot*= tempQuat;

				geometry_msgs::Pose curPose = pose_map.at(current_vis_msg.id).pose;
				Eigen::Quaternionf pARTag;
				pARTag.x() = curPose.position.x;
				pARTag.y() = curPose.position.y;
				pARTag.z() = curPose.position.z;
				pARTag.w() = 0;

				Eigen::Quaternionf arPose_wrt_map = curPose.orientation.inverse();
				arPose_wrt_map*= pARTag;
				arPose_wrt_map*= curPose.orientation;

		    	Eigen::Quaternionf result = arPose_wrt_robot - arPose_wrt_map;
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
