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
#include <stdio.h>
#include <vector>

struct marker {
  int id;
  int counter;
  float x;
  float y;
  float z;
};

bool markerSeen = false;

visualization_msgs::Marker current_vis_msg;
visualization_msgs::MarkerArray marker_array_msg;
std::map<int, marker> data;
std::map<int, marker>::iterator it;

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
		       }
			    catch (tf::TransformException ex) {}
          it = data.find(current_vis_msg.id);
          if(it == data.end()){
            marker m;
            m.id = current_vis_msg.id;
            m.counter = 1;
            m.x = tag_wresp_map.pose.position.x;
            m.y = tag_wresp_map.pose.position.y;
            m.z = tag_wresp_map.pose.position.z;
            data.insert(it, std::pair<int, marker>(m.id , m));
          }
          else{
            marker m = data.at(current_vis_msg.id);
            m.x = (m.x * m.counter + tag_wresp_map.pose.position.x) / m.counter+1;
            m.y = (m.y * m.counter + tag_wresp_map.pose.position.y) / m.counter+1;
            m.z = (m.z * m.counter + tag_wresp_map.pose.position.z) / m.counter+1;
            m.counter++;
            data.insert(it, std::pair<int, marker>(m.id , m));
          }
          printf("%d,%f,%f,%f\n", current_vis_msg.id, tag_wresp_map.pose.position.x, tag_wresp_map.pose.position.y
          , tag_wresp_map.pose.position.z);
          markerSeen = false;
	    }
	}
}
