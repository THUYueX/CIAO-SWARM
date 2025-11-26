#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath> 
#include <string>
#include <vector>
#include <sstream>
#include <boost/bind.hpp>

class MultiDroneTFPublisher {
public:
    MultiDroneTFPublisher(const std::vector<std::string>& namespaces) {
        for (const auto& ns : namespaces) {
            std::string topic_name = ns + "/mavros/local_position/pose";
            ros::Subscriber sub = nh_.subscribe<geometry_msgs::PoseStamped>(
                topic_name, 10, 
                boost::bind(&MultiDroneTFPublisher::poseCallback, this, _1, ns)
            );
            pose_subs_.push_back(sub);
            ROS_INFO("TF Publisher started for namespace: %s", ns.c_str());
        }
    }
    
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& ns) {
        // 发布机体坐标系
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "map_ned";
        transform.child_frame_id = ns + "/base_link_frd";
        transform.transform.translation.x = msg->pose.position.x;
        transform.transform.translation.y = msg->pose.position.y;
        transform.transform.translation.z = msg->pose.position.z;
        transform.transform.rotation = msg->pose.orientation;
        br_.sendTransform(transform);

        // 发布相机坐标系
        geometry_msgs::TransformStamped cam_transform;
        cam_transform.header.stamp = ros::Time::now();
        cam_transform.header.frame_id = ns + "/base_link_frd";
        cam_transform.child_frame_id = ns + "/robot_camera_link";
        cam_transform.transform.translation.x = 0.1;
        cam_transform.transform.translation.y = 0;
        cam_transform.transform.translation.z = 0;

        tf2::Quaternion quat;
        quat.setRPY(-M_PI/2, 0, -M_PI/2);
        cam_transform.transform.rotation.x = quat.x();
        cam_transform.transform.rotation.y = quat.y();
        cam_transform.transform.rotation.z = quat.z();
        cam_transform.transform.rotation.w = quat.w();

        br_.sendTransform(cam_transform);
    }
    
private:
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> pose_subs_;
    tf2_ros::TransformBroadcaster br_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_drone_tf_publisher");
    
    if (argc < 2) {
        ROS_ERROR("Usage: multi_tf_publisher <namespace1> [namespace2] ...");
        ROS_ERROR("Example: multi_tf_publisher wmy_iris_camRay0 wmy_iris_camRay1");
        return 1;
    }
    
    std::vector<std::string> namespaces;
    for (int i = 1; i < argc; i++) {
        namespaces.push_back(argv[i]);
    }
    
    MultiDroneTFPublisher publisher(namespaces);
    
    ros::spin();  // 使用spin而不是spinOnce+循环
    
    return 0;
}