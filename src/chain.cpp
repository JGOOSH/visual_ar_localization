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
  geometry_msgs::Quaternion quat = cur_pose.orientation;
	Eigen::Matrix3f mat3 = Eigen::Quaternionf(quat.w, quat.x, quat.y, quat.z).toRotationMatrix();
	Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
	mat4.block(0,0,3,3) =mat3;
  // mat4(0, 3) = cur_pose.position.x;
  // mat4(1, 3) = cur_pose.position.y;
  // mat4(2, 3) = cur_pose.position.z;
	return mat4;

// geometry_msgs::Quaternion quat = cur_pose.orientation;
//   Eigen::Quaternionf q(quat.x,quat.y,quat.z,quat.w);
//   Eigen::Matrix4f temp;
//   temp.block(0, 0, 3, 3) = q.toRotationMatrix();
//   temp(0, 3) = cur_pose.position.x;
//   temp(1, 3) = cur_pose.position.y;
//   temp(2, 3) = cur_pose.position.z;
//   temp(3, 3) = 1;
  /*
Eigen::Matrix3f mat3 = Eigen::Quaternionf(W, X, Y, Z).toRotationMatrix(); Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity(); mat4.block(0,0,3,3) = mat3;
  */
  // return temp;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "chain");
	  ros::NodeHandle n;
    tf::TransformListener tf_l;

    //subscriber for the /visualization_marker
    ros::Subscriber vis_sub = n.subscribe("/visualization_marker", 1, vis_cb);

      //subscriber for the /amcl_pose
    ros::Subscriber amcl_sub = n.subscribe("/amcl_pose", 1, amcl_cb);

    ROS_INFO("MADE IT");
    while(ros::ok()) {
      ROS_INFO("Danny is lonely");
      ros::spinOnce();
      if(marker_seen) {
        ROS_INFO("Danny found the tag");
        ROS_INFO("Danny found this number of tag %d but the current number should be %d so he got sad.", current_vis_msg.id , cur_tag_id);
        if(current_vis_msg.id == cur_tag_id || (look_prev && current_vis_msg.id == cur_tag_id-1) ) {
          if(!first_seen)
          {
            first_pose.header = current_vis_msg.header;
            first_pose.pose = current_vis_msg.pose;
            Eigen::Matrix4f asd = getMatFromPose (first_pose.pose);
            Eigen::Matrix4f inv = asd.inverse();
          ROS_INFO("first m1 %f, %f, %f", current_vis_msg.pose.position.x, current_vis_msg.pose.position.y, current_vis_msg.pose.position.z);
          ROS_INFO("m1 inverse %f, %f, %f",inv(0, 3), inv(1, 3), inv(2, 3));
          ROS_INFO(" particle filter %f, %f, %f",robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, robot_pose.pose.pose.position.z);
            /*try {
              tf_l.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1));
              tf_l.transformPose("/map", first_pose, sec_pose);
            }
          catch (tf::TransformException ex) {ROS_INFO ("Hey handsome");}
            */
          first_seen = true;
            mats_arr[0] = getMatFromPose(first_pose.pose);
            cur_tag_id++;
            ROS_INFO("FIRST SEEN");
          }
          else {
            /* non first method */
            ROS_INFO("DANNY LOK PREV IS %d CUR VIS ID IS %d CUR TAG ID is %d\n", look_prev, current_vis_msg.id, cur_tag_id-1);
            if(look_prev && current_vis_msg.id == cur_tag_id-1)
            {
              ROS_INFO("DANNY I AM IN LOOK PREV FIRST NON METHOD");
              Eigen::Matrix4f prev = getMatFromPose(current_vis_msg.pose);
              Eigen::Matrix4f temp = mats_arr[cur_tag_id-1] * prev.inverse();
              // Eigen::Matrix4f abc = temp.inverse();
              mats_arr[cur_tag_id] = temp * mats_arr[cur_tag_id];
              printf("%d, %d, %f, %f, %f\n", 1, cur_tag_id, temp(0, 3), temp(1, 3), temp(2, 3));
              printf("%d, %d, %f, %f, %f\n", 0, cur_tag_id, robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, robot_pose.pose.pose.position.z);
              look_prev = false;
              cur_tag_id++;
            }
            else
            {
              ROS_INFO("I AM GONNA CALCULATE ");
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
      ros::Duration(2).sleep();
    }
  }
}

/*
      //TEST CODE FOR AMCL_POSE - accurate robot position from map
      printf("The robot is located at: %f, %f, %f\n", robot_pose.pose.pose.position.x,
robot_pose.pose.pose.position.y, robot_pose.pose.pose.position.z);
*/
