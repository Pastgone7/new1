
#pragma once
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
namespace hero_chassis_controller
{

class HeroChassisController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:

  HeroChassisController();
  ~HeroChassisController();


  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);



  void starting(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);


  std::string getJointName();

 
  double command_[4];                              

private:
  void velCallBack(const geometry_msgs::TwistConstPtr msg);
  void solve();
  void calWheelSpeed(const ros::Duration& period);
  control_toolbox::Pid pid1_controller_;
  control_toolbox::Pid pid2_controller_;
  control_toolbox::Pid pid3_controller_;
  control_toolbox::Pid pid4_controller_;  
  
  hardware_interface::JointHandle right1_joint_;           
  hardware_interface::JointHandle right2_joint_;   
  hardware_interface::JointHandle left1_joint_;   
  hardware_interface::JointHandle left2_joint_;  
  
  double acc;
  //轴距
  double wheelBase;
  //轮距
  double trackWidth;
  //轮半径
  double radius;
  ros::Subscriber velSub;

  //获取的速度
  geometry_msgs::Twist g_vel;
  
  //历史时间
  ros::Time lastTime;
  
  //历史里程
  double x;
  double y;
  double th;
  
  double vax;
  double vay;
  double wa;
  
  double dt;
  
  tf2_ros::TransformBroadcaster *g_odomBroadcaster;
  ros::Publisher g_odomPub;
  
};

} 
