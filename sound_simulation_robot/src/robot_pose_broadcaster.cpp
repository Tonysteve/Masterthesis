#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Pose.h"


void poseCallback(const gazebo_msgs::LinkStatesConstPtr msg){

    std::string link_frame = "youbot::base_footprint";
    bool found = false;
    int index = -1;

    for(int i = 0; i< msg->name.size(); i++){

        if(msg->name[i]==link_frame){
            index = i;
            found = true;
            break;
        }
    }

    geometry_msgs::Pose robot_pos_gazebo= msg->pose[index];

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(robot_pos_gazebo.position.x, robot_pos_gazebo.position.y,robot_pos_gazebo.position.z) );
    tf::Quaternion q;
    q.setX(robot_pos_gazebo.orientation.x);
    q.setY(robot_pos_gazebo.orientation.y);
    q.setZ(robot_pos_gazebo.orientation.z);
    q.setW(robot_pos_gazebo.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_footprint_gazebo"));
}

int main(int argc, char** argv){

    ros::init(argc, argv, "robot_pose_broadcaster");


    ros::NodeHandle node;
    ros::Subscriber robot_pose_sub = node.subscribe("/gazebo/link_states", 10, &poseCallback);

    ros::spin();
    return 0;
};
