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
#include <string>
#include <stdio.h>


#define NUM_TAGS 50

bool marker_seen = false;
bool first_seen = false;
bool look_prev = false;
bool demo = false;
geometry_msgs::PoseStamped camera_pose;
geometry_msgs::PoseStamped base_link_pose;
geometry_msgs::PoseStamped tag_map_pose;
visualization_msgs::Marker current_vis_msg;
Eigen::Matrix4f mats_arr[NUM_TAGS];
int cur_tag_id = 0;
geometry_msgs::PoseWithCovarianceStamped robot_pose;

/* Call back method to get the pose of AR marker */
void vis_cb(const visualization_msgs::Marker::ConstPtr& msg) {
    current_vis_msg = *msg;
    marker_seen = true;
}

/* Call back method to get the pose of Segbot */
void amcl_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  robot_pose = *msg;
}

/* Method to convert a vector into a 4x4 matrix */
Eigen::Matrix4f pose_to_mat(geometry_msgs::Pose cur_pose) {
    geometry_msgs::Quaternion quat = cur_pose.orientation;
    Eigen::Matrix3f quat_to_mat;
    Eigen::Matrix4f mat_output;
    quat_to_mat = Eigen::Quaternionf(quat.w, quat.x, quat.y, quat.z).toRotationMatrix();
    mat_output = Eigen::Matrix4f::Identity();
    mat_output.block(0,0,3,3) = quat_to_mat;
    mat_output(0, 3) = cur_pose.position.x;
    mat_output(1, 3) = cur_pose.position.y;
    mat_output(2, 3) = cur_pose.position.z; 
    return mat_output;
}

/* Method to print the output on ith marker computation */
void print_pose (Eigen::Matrix4f mat) {
  // std::string msg_ar_pos = "1, " + cur_tag_id + ", " + mat(0, 3) + ", " + mat(1, 3) + ", " + mat(2, 3);
  printf("%d, %d, %f, %f, %f\n", 1, cur_tag_id, mat(0, 3), mat(1, 3), mat(2, 3));
  printf("%d, %d, %f, %f, %f\n", 0, cur_tag_id, robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, robot_pose.pose.pose.position.z);
  fprintf(stderr, "%d, %d, %f, %f, %f\n", 1, cur_tag_id, mat(0, 3), mat(1, 3), mat(2, 3));
  fprintf(stderr, "%d, %d, %f, %f, %f\n", 0, cur_tag_id, robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, robot_pose.pose.pose.position.z);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "data_collection");
    ros::NodeHandle n;
    tf::TransformListener tf_l;

    //subscriber to get tag information
    ros::Subscriber vis_sub = n.subscribe("/visualization_marker", 1, vis_cb);

    //subscriber to get robot location
    ros::Subscriber amcl_sub = n.subscribe("/amcl_pose", 1, amcl_cb);

    // ros::Publisher data_pub = n.advertise<std_msgs::String>("ar_data", 100);

    while(ros::ok()) {
      ros::spinOnce();
      ros::Duration(2).sleep();
      // compute only if a tag is in the visual frame
      if(marker_seen) {
        ROS_INFO("Robot saw tag %d and the current tag should be %d", current_vis_msg.id , cur_tag_id);
        if(current_vis_msg.id == cur_tag_id || (look_prev && current_vis_msg.id == cur_tag_id-1) ) {
          if(!first_seen) {
            camera_pose.header = current_vis_msg.header;
            camera_pose.pose = current_vis_msg.pose;
            try {
              tf_l.waitForTransform("/map", camera_pose.header.frame_id, ros::Time(0), ros::Duration(4));
              tf_l.transformPose("/map", camera_pose, tag_map_pose);
              mats_arr[0] = pose_to_mat (tag_map_pose.pose);
              Eigen::Matrix4f first_robot_pose = mats_arr[0] * (pose_to_mat(base_link_pose.pose).inverse());
              print_pose(first_robot_pose);
            }
            catch (tf::TransformException ex) {ROS_INFO ("%s", ex.what());}

            first_seen = true;
            cur_tag_id++;
          }
          else {
            /* non first method */
            ROS_INFO("Look Previous is %d Current Tag ID is %d CUR TAG ID -1 is %d\n", look_prev, current_vis_msg.id, cur_tag_id-1);
            if(look_prev && current_vis_msg.id == cur_tag_id-1)
            {
              camera_pose.header = current_vis_msg.header;
              camera_pose.pose = current_vis_msg.pose;
              try {
                ros::Duration(2).sleep();
                tf_l.waitForTransform("/base_link", camera_pose.header.frame_id, ros::Time(0), ros::Duration(4));
                tf_l.transformPose("/base_link", camera_pose, base_link_pose);
                Eigen::Matrix4f prev = pose_to_mat(base_link_pose.pose);
                Eigen::Matrix4f cur_robot_pose = mats_arr[cur_tag_id-1] * prev.inverse();
                mats_arr[cur_tag_id] = cur_robot_pose * mats_arr[cur_tag_id];
                print_pose(cur_robot_pose);
                look_prev = false;
                if(cur_tag_id == 8) {
                  demo = true;
                }
                cur_tag_id++;
              }
              catch (tf::TransformException ex) {ROS_INFO ("%s", ex.what());} 
            }
            if(demo && current_vis_msg.id < 8) {
              camera_pose.header = current_vis_msg.header;
              camera_pose.pose = current_vis_msg.pose;
              try {
                ros::Duration(2).sleep();
                tf_l.waitForTransform("/base_link", camera_pose.header.frame_id, ros::Time(0), ros::Duration(4));
                tf_l.transformPose("/base_link", camera_pose, base_link_pose);
                Eigen::Matrix4f prev = pose_to_mat(base_link_pose.pose);
                Eigen::Matrix4f cur_robot_pose = mats_arr[cur_tag_id] * prev.inverse();
                printf("LOCATION OF ROBOT INCOMING");
                print_pose(cur_robot_pose);
              }
              catch (tf::TransformException ex) {ROS_INFO ("%s", ex.what());
            } 
            else
            {
              /* the case where we see the tag for the first time ever
              then, we have to get Mn matrix and tell the algo to look back at
              tag n-1  Tn = Tn-1 * a^-1 * b where a is new perspective of
              prev tag and b is new tag */
              /* temporaily store this matrix into array */
              camera_pose.header = current_vis_msg.header;
              camera_pose.pose = current_vis_msg.pose;
              try {
                ros::Duration(2).sleep();
                tf_l.waitForTransform("/base_link", camera_pose.header.frame_id, ros::Time(0), ros::Duration(4));
                tf_l.transformPose("/base_link", camera_pose, base_link_pose);
                mats_arr[cur_tag_id] = pose_to_mat(base_link_pose.pose);
                look_prev = true;
              }
              catch (tf::TransformException ex) {ROS_INFO ("%s", ex.what());} 
            }
          }
          if(!look_prev) printf("Done calculating tag ID: %d \n", cur_tag_id-1);
        }
      marker_seen = false;
    }
  }
}
