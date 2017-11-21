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
float THRESHOLD = 1;

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
    		//Iterator stores the location of the id within the internal map
	        it = pose_map.find(current_vis_msg.id);
	        //If the tag is NEW
	        if(it == pose_map.end()) {
	          //first visualize it it in to the rviz
	          marker_array_msg.markers.push_back(current_vis_msg);
	      	  //Transfer information into a StampedPose
	  				stampedPose.header = current_vis_msg.header;
	  				stampedPose.pose = current_vis_msg.pose;
	        	try{
	        		tf_l.waitForTransform("/map",
	                              stampedPose.header.frame_id, ros::Time(0), ros::Duration(1));
	        		//Performs transformation and stores it in the third parameter
	        		tf_l.transformPose("/map", stampedPose, tag_wresp_map);
	        		//JAMIN TAKES tag_wresp_map and annotates the map with its
			         }
				    catch (tf::TransformException ex){
				        ROS_ERROR("TransformException");
				         }
	          ROS_INFO("Current tag is at x : %f, y : %f, z : %f", tag_wresp_map.pose.position.x, tag_wresp_map.pose.position.y
	          , tag_wresp_map.pose.position.z);
	          pose_map.insert(it, std::pair<int, geometry_msgs::PoseStamped>(current_vis_msg.id , tag_wresp_map));
		        }
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
		        	
		        	//Pose for Tag Truth
		        	geometry_msgs::PoseStamped tag_truth = pose_map.at(current_vis_msg.id);

		        	//Create Quaternion that goes with the Tag
		        	//This helps compute the x,y,z of the Tag
					Eigen::Quaternionf tag_position;
					tag_position.x() = tag_truth.pose.position.x;
					tag_position.y() = tag_truth.pose.position.y;
					tag_position.z() = tag_truth.pose.position.z;
					tag_position.w() = 0;


					Eigen::Quaternionf tag_quat;
					tag_quat.x() = tag_truth.pose.orientation.x;
					tag_quat.y() = tag_truth.pose.orientation.y;
					tag_quat.z() = tag_truth.pose.orientation.z;
					tag_quat.w() = tag_truth.pose.orientation.w;

					//Quat that stores the XYZ of the AR Tag Truth
					Eigen::Quaternionf tag_truth_location = tag_quat.inverse();
					tag_truth_location*= tag_position;
					tag_truth_location*= tag_quat;

					//Calculate relative position of AR Tag
					Eigen::Quaternionf tag_rel;
					tag_rel.x() = current_vis_msg.pose.position.x;
					tag_rel.y() = current_vis_msg.pose.position.y;
					tag_rel.z() = current_vis_msg.pose.position.z;
					tag_rel.w() = 0;


					Eigen::Quaternionf rel_tag_quat;
					rel_tag_quat.x() = current_vis_msg.pose.orientation.x;
					rel_tag_quat.y() = current_vis_msg.pose.orientation.y;
					rel_tag_quat.z() = current_vis_msg.pose.orientation.z;
					rel_tag_quat.w() = current_vis_msg.pose.orientation.w;


					//Quat that stores the XYZ of the relative AR Tag
					Eigen::Quaternionf tag_rel_location = rel_tag_quat.inverse();
					tag_rel_location*= tag_rel;
					tag_rel_location*= rel_tag_quat;


					//Quaternion that's going to store the difference value for the truth and relative
					Eigen::Quaternionf difference_quat;

					difference_quat.x() = tag_truth_location.x() - tag_rel_location.x();
					difference_quat.y() = tag_truth_location.y() - tag_rel_location.y();
					difference_quat.z() = tag_truth_location.z() - tag_rel_location.z();
					difference_quat.w() = 0;

					float critical_value = difference_quat.norm();

					if(critical_value < THRESHOLD) {
						//HOW DO I GET THE XYZ OF THE ROBOT???
						ROS_INFO("%d,%f,%f,%f", current_vis_msg.id, );
						//SLEEP for 4 SECONDS until the TAG IS OUT OF VIEW
				}
		    }
	    	markerSeen = false;
        	marker_pub.publish(marker_array_msg);
	    }
	}
}