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

    tf::TransformListener tf_l(ros::Duration(10));

    //subscriber for the /visualization_marker
    ros::Subscriber vis_sub = n.subscribe("/visualization_marker", 1, vis_cb);

    //HOLDS ID->Pose pairs
    std::map<int, geometry_msgs::Pose> pose_map;
    //Iterator of the map to find if an ID already exists when detecting a new Tag
    std::map<int, geometry_msgs::Pose>::iterator it;

    //Pose of Tag with resp to map
    geometry_msgs::Pose tag_wresp_map;

    while(ros::ok()) {
    	ros::spinOnce();
    	if(markerSeen) {
	        it = pose_map.find(current_vis_msg.id);
	        if(it == pose_map.end()) {
	        	try{
	        		ROS_INFO_STREAM("wait");
	        		tf_l.waitForTransform(base_link,
	                              current_vis_msg.pose, ros::Time(0), ros::Duration(1));
	        		ROS_INFO_STREAM("before transform");
	        		tf_l.transformPose(base_link, current_vis_msg.pose, tag_wresp_map);
	        		ROS_INFO_STREAM("Transorm exected");
	        		//JAMIN TAKES tag_wresp_map and annotates the map with it

			    }
			    catch (tf::TransformException ex){
			        ROS_ERROR("%s",ex.what());
			    }
	          ROS_INFO("Current tag is at x : %f, y : %f, z : %f", current_vis_msg.pose.position.x, current_vis_msg.pose.position.y
	          , current_vis_msg.pose.position.z);
	          pose_map.insert(it, std::pair<int, geometry_msgs::Pose>(current_vis_msg.id , current_vis_msg.pose));

	        }
	        else {
	          ROS_INFO("ALREADY EXISTS"); 
	        }
	    		markerSeen = false;
	    }
	}
}
