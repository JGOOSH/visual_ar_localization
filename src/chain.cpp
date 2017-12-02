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
bool look_prev = false;
geometry_msgs::Pose first_pose = NULL;
visualization_msgs::Marker current_vis_msg;
Eigen::Matrix4f mats_arr[50];
int cur_tag_id = 0;

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
    		if(current_vis_msg.id == cur_tag_id) {
	    		if(!first_seen)
          {
	    			first_pose = current_vis_msg.pose;
	    			first_seen = true;
	    			mats_arr[0] = getMatFromPose();
            cur_tag_id++;
	    		}
          else {
            /* non first method */
            if(look_prev && current_vis_msg.id == cur_tag_id-1)
            {
              Eigen::Matrix4f prev = getMatFromPose();
              mats_arr[cur_tag_id] = mats_arr[cur_tag_id-1] * prev.inverse() * mats_arr[cur_tag_id];
              look_prev = false;
              cur_tag_id++;
            }
            else
            {
              /* the case where we see the tag for the first time ever
              then, we have to get Mn matrix and tell the algo to look back at
              tag n-1  Tn = Tn-1 * a^-1 * b where a is new perspective of
              prev tag and b is new tag */
              Eigen::Matrix4f cur = getMatFromPose();
              /* temporaily store this matrix into array */
              mats_arr[cur_tag_id] = cur;
              look_prev = true;
            }
	    		}
          if(!look_prev) printf("Done calculating %dth tag\n", cur_tag_id-1);
	    	}
      marker_seen = false;
		}
	}
}
