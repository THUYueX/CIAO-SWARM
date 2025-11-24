#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

class DroneTFPublisher {
public:
    DroneTFPublisher(const std::string& ns) : ns_(ns) {
        std::string topic_name = ns + "/mavros/local_position/pose";
        pose_sub_ = nh_.subscribe(topic_name, 10, 
                                 &DroneTFPublisher::poseCallback, this);
        ROS_INFO("TF Publisher started for namespace: %s", ns_.c_str());
    }
    
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 发布机体坐标系
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "map_ned";
        transform.child_frame_id = ns_ + "/base_link_frd";
        
        transform.transform.translation.x = msg->pose.position.x;
        transform.transform.translation.y = msg->pose.position.y;
        transform.transform.translation.z = msg->pose.position.z;
        transform.transform.rotation = msg->pose.orientation;
        
        br_.sendTransform(transform);
        
        // 发布相机坐标系
        geometry_msgs::TransformStamped cam_transform;
        cam_transform.header.stamp = ros::Time::now();
        cam_transform.header.frame_id = ns_ + "/base_link_frd";
        cam_transform.child_frame_id = ns_ + "/robot_camera_link";
        cam_transform.transform.translation.x = 0.1;
        cam_transform.transform.translation.y = 0;
        cam_transform.transform.translation.z = 0;
        cam_transform.transform.rotation.w = 1.0;
        
        br_.sendTransform(cam_transform);
    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    tf2_ros::TransformBroadcaster br_;
    std::string ns_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_tf_publisher");
    
    if (argc != 2) {
        ROS_ERROR("Usage: tf_publisher <namespace>");
        ROS_ERROR("Example: tf_publisher wmy_iris_camRay0");
        return 1;
    }
    
    std::string namespace_str = argv[1];
    DroneTFPublisher publisher(namespace_str);
    
    ros::Rate rate(100);  // 100Hz
    while (ros::ok()) {
        ros::spinOnce();  // 处理回调
        rate.sleep();     // 保持频率
    }
    
    return 0;
}