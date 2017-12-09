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
#include "geometry_msgs/PoseWithCovarianceStamped.h"

bool marker_seen = false;
bool first_seen = false;
bool look_prev = false;
geometry_msgs::PoseStamped first_pose;
geometry_msgs::PoseStamped sec_pose;
visualization_msgs::Marker current_vis_msg;
Eigen::Matrix4f mats_arr[50];
int cur_tag_id = 0;
geometry_msgs::PoseWithCovarianceStamped robot_pose;

void vis_cb(const visualization_msgs::Marker::ConstPtr& msg) {
    current_vis_msg = *msg;
    marker_seen = true;
}

void amcl_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  robot_pose = *msg;
}

Eigen::Matrix4f getMatFromPose(geometry_msgs::Pose cur_pose) {
  geometry_msgs::Quaternion quat = cur_pose.pose.orientation;
  Eigen::Quaternionf q(quat.x,quat.y,quat.z,quat.w);
  Eigen::Matrix4f temp;
  temp.block(0, 0, 3, 3) = q.toRotationMatrix();
  temp(0, 3) = cur_pose.position.x;
  temp(1, 3) = cur_pose.position.y;
  temp(2, 3) = cur_pose.position.z;
  temp(3, 3) = 1;
  return temp;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "chain");
	  ros::NodeHandle n;
    //tf::TransformListener tf_l;

    //subscriber for the /visualization_marker
    ros::Subscriber vis_sub = n.subscribe("/visualization_marker", 1, vis_cb);

      //subscriber for the /amcl_pose
    ros::Subscriber amcl_sub = n.subscribe("/amcl_pose", 1, amcl_cb);

    ROS_INFO("MADE IT");
    while(ros::ok()) {
      ros::spinOnce();
      if(marker_seen) {
        if(current_vis_msg.id == cur_tag_id) {
          if(!first_seen)
          {
            first_pose.header = current_vis_msg.header;
            first_pose.pose = current_vis_msg.pose;
            try {
              tf_l.waitForTransform("/map", first_pose.header.frame_id, ros::Time(0), ros::Duration(1));
              tf_l.transformPose("/map", first_pose, sec_pose);
            }
          catch (tf::TransformException ex) {}
            first_seen = true;
            mats_arr[0] = getMatFromPose(sec_pose.pose);
            cur_tag_id++;
            ROS_INFO("FIRST SEEN");
          }
          else {
            /* non first method */
            if(look_prev && current_vis_msg.id == cur_tag_id-1)
            {
              Eigen::Matrix4f prev = getMatFromPose(current_vis_msg.pose);
              Eigen::Matrix4f temp = mats_arr[cur_tag_id-1] * prev.inverse();
              mats_arr[cur_tag_id] = temp * mats_arr[cur_tag_id];
              printf("%d, %d, %f, %f, %f\n", 1, cur_tag_id, temp(0, 3), temp(1, 3), temp(2, 3));
              printf("%d, %d, %f, %f, %f\n", 0, cur_tag_id, robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, robot_pose.pose.pose.position.z);
              look_prev = false;
              cur_tag_id++;
            }
            else
            {
              /* the case where we see the tag for the first time ever
              then, we have to get Mn matrix and tell the algo to look back at
              tag n-1  Tn = Tn-1 * a^-1 * b where a is new perspective of
              prev tag and b is new tag */
              Eigen::Matrix4f cur = getMatFromPose(current_vis_msg.pose);
              /* temporaily store this matrix into array */
              mats_arr[cur_tag_id] = cur;
              look_prev = true;
            }
          }
          if(!look_prev) printf("Done calculating %dth tag\n", cur_tag_id-1);
        }
      marker_seen = false;
      ros::Duration(5).sleep();
    }
  }
}

/*
      //TEST CODE FOR AMCL_POSE - accurate robot position from map
      printf("The robot is located at: %f, %f, %f\n", robot_pose.pose.pose.position.x, 
robot_pose.pose.pose.position.y, robot_pose.pose.pose.position.z);
*/
