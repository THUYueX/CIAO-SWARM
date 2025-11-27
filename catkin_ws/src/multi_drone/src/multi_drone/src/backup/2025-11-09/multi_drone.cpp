#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <cmath>

class MultiDroneController
{
private:
  struct DroneUnit
  {
    std::string ns;
    mavros_msgs::State state;
    ros::Publisher  pub_sp;
    ros::Subscriber sub_state;
    ros::ServiceClient cli_arm;
    ros::ServiceClient cli_mode;
    geometry_msgs::PoseStamped sp;
    bool offboard_good{false};
    bool arm_good{false};
    ros::Time last_req{ros::Time::now()};

    DroneUnit(ros::NodeHandle& nh, const std::string& name)
      : ns(name)
    {
      sub_state = nh.subscribe<mavros_msgs::State>
                  ("/" + ns + "/mavros/state", 10, &DroneUnit::cb_state, this);
      pub_sp    = nh.advertise<geometry_msgs::PoseStamped>
                  ("/" + ns + "/mavros/setpoint_position/local", 10);
      cli_arm   = nh.serviceClient<mavros_msgs::CommandBool>
                  ("/" + ns + "/mavros/cmd/arming");
      cli_mode  = nh.serviceClient<mavros_msgs::SetMode>
                  ("/" + ns + "/mavros/set_mode");
      sp.pose.orientation.w = 1.0;
    }

    void cb_state(const mavros_msgs::State::ConstPtr& msg){ state = *msg; }

    bool try_set_mode(const std::string& mode)
    {
      mavros_msgs::SetMode srv;
      srv.request.custom_mode = mode;
      if (cli_mode.call(srv) && srv.response.mode_sent){ return true; }
      return false;
    }

    bool try_arm(bool arm)
    {
      mavros_msgs::CommandBool srv;
      srv.request.value = arm;
      if (cli_arm.call(srv) && srv.response.success){ return true; }
      return false;
    }

    void publish_sp()
    {
      sp.header.stamp = ros::Time::now();
      pub_sp.publish(sp);
    }
  };

  std::vector<DroneUnit> drones_;
  ros::NodeHandle        nh_;

public:
  MultiDroneController()
  {
    // 只留一架 drone3
    drones_.emplace_back(nh_, "drone3");
  }

  void run()
  {
    ros::Rate rate(20);
    auto& d = drones_[0];          // 只飞这一架

    /* 1. 先发 2 s 初始点 */
    for (int i = 0; i < 40 && ros::ok(); ++i){
      d.publish_sp();
      ros::spinOnce(); rate.sleep();
    }

    /* 2. 切 OFFBOARD + 解锁 */
    while (ros::ok()){
      d.publish_sp();
      if (!d.state.connected){ ros::spinOnce(); rate.sleep(); continue; }

      if (!d.offboard_good && d.state.mode != "OFFBOARD" &&
          (ros::Time::now() - d.last_req) > ros::Duration(2.0)){
        if (d.try_set_mode("OFFBOARD")) d.offboard_good = true;
        d.last_req = ros::Time::now();
      }
      if (!d.offboard_good){ ros::spinOnce(); rate.sleep(); continue; }

      if (!d.arm_good && !d.state.armed &&
          (ros::Time::now() - d.last_req) > ros::Duration(2.0)){
        if (d.try_arm(true)) d.arm_good = true;
        d.last_req = ros::Time::now();
      }
      if (d.offboard_good && d.arm_good) break;
      ros::spinOnce(); rate.sleep();
    }

    /* 3. 绕圈参数 */
    const double R   = 5.0;        // 半径 5 m
    const double H   = 2.0;        // 高度 2 m
    const double OMEGA = 0.3;      // 角速度 rad/s
    ros::Time t0 = ros::Time::now();

    /* 4. 绕圈飞 */
    ROS_INFO("drone3 begin circle flying...");
    while (ros::ok()){
      double t = (ros::Time::now() - t0).toSec();
      d.sp.pose.position.x = R * std::cos(OMEGA * t);
      d.sp.pose.position.y = R * std::sin(OMEGA * t);
      d.sp.pose.position.z = H;
      d.publish_sp();
      ros::spinOnce();
      rate.sleep();
    }
  }
};

/* ------------------------ main ------------------------ */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_drone_controller");
  MultiDroneController ctl;
  ctl.run();
  return 0;
}