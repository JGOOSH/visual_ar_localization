#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Twist.h"
#include <map>
#include <tf/tf.h>
#include <tf/transform_listener.h>

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

    //Listener for the transform
    tf::TransformListener tf_l;

    //subscriber for the /visualization_marker
    ros::Subscriber vis_sub = n.subscribe("/visualization_marker", 1, vis_cb);

    //publisher for the markers on the map
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Makrer>("map_markers", 1000);

    //HOLDS ID->Pose pairs
    std::map<int, geometry_msgs::PoseStamped> pose_map;
    //Iterator of the map to find if an ID already exists when detecting a new Tag
    std::map<int, geometry_msgs::PoseStamped>::iterator it;

    //input stampedPose
   	geometry_msgs::PoseStamped stampedPose;

    //output stampedPose
    geometry_msgs::PoseStamped tag_wresp_map;

    while(ros::ok()) {
    	ros::spinOnce();
      visualization_msgs::Marker map_msg;
    	if(markerSeen) {
    		//Iterator stores the location of the id within the internal map
        it = pose_map.find(current_vis_msg.id);
        //If the tag is NEW
        if(it == pose_map.end()) {
      	  //Transfer information into a StampedPose
          msgs = current_vis_msg;
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
          marker_pub.publish(map_msg);
	        }
	        else {
	        	//The issue here is that we have to use the Pose that the Iterator is pointing at!!
		    	ROS_INFO("This pre-existing tag is at x : %f, y : %f, z : %f", current_vis_msg.pose.position.x, current_vis_msg.pose.position.y
		          , current_vis_msg.pose.position.z);
	    	}
	    	markerSeen = false;
	    }
	}
}
