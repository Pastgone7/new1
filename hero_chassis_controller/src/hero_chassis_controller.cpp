#include <hero_chassis_controller/hero_chassis_controller.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace hero_chassis_controller {

HeroChassisController::HeroChassisController()
{}

HeroChassisController::~HeroChassisController()
{
 
}

bool HeroChassisController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  //从yaml中分别获取四个轴的joint名称
  std::string joint_name;
  if (!n.getParam("left1_joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }


  left1_joint_ = robot->getHandle(joint_name);
  
  if (!n.getParam("left2_joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }


  left2_joint_ = robot->getHandle(joint_name);
  
  
  if (!n.getParam("right1_joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }


  right1_joint_ = robot->getHandle(joint_name);
  
  
  if (!n.getParam("right2_joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }


  right2_joint_ = robot->getHandle(joint_name);
  

  //从yaml中获取四个轴的pid参数
  if (!pid1_controller_.init(ros::NodeHandle(n, "pid1")))
    return false;
  if (!pid2_controller_.init(ros::NodeHandle(n, "pid2")))
    return false;
  if (!pid3_controller_.init(ros::NodeHandle(n, "pid3")))
    return false;
  if (!pid4_controller_.init(ros::NodeHandle(n, "pid4")))
    return false; 
  
  
  
  n.getParam("wheelBase", wheelBase);
  n.getParam("trackWidth", trackWidth);
  n.getParam("radius", radius);
  n.getParam("acc", acc);
  
  g_odomPub = n.advertise<nav_msgs::Odometry>("/odom",10);
  velSub = n.subscribe("/cmd_vel",1,&HeroChassisController::velCallBack,this);
  
  g_odomBroadcaster = new tf2_ros::TransformBroadcaster();
  
  
  lastTime = ros::Time::now();
  
  ROS_INFO("HeroChassisController");
  return true;
}


void HeroChassisController::starting(const ros::Time& time)
{
  pid1_controller_.reset();
  pid2_controller_.reset();
  pid3_controller_.reset();
  pid4_controller_.reset();
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{

  //计算运动学逆解
  calWheelSpeed(period);

  //计算运动学正解
  solve();

}

void HeroChassisController::solve(){

    double dx;
    double dy;
    double dth;

    
    dt = (ros::Time::now() - lastTime).toSec();
    lastTime = ros::Time::now();
    if(dt > 2){
         //时间异常
         return;
    }
    

    double v1 = right1_joint_.getVelocity();
    double v2 = left1_joint_.getVelocity();
    double v3 = left2_joint_.getVelocity();
    double v4 = right2_joint_.getVelocity();
    
    dx = radius*0.25*(v1+v2+v3+v4);
    dy = radius*0.25*(-v3+v4+v2-v1);
    dth = radius*(-v3+v4-v2+v1)/((trackWidth/2+wheelBase/2)*4);
    
    vax = dx;
    vay = dy;
    wa = dth;
    
    x += dx*cos(th)*dt - dy*sin(th)*dt;
    y += dx*sin(th)*dt + dy*cos(th)*dt;
    th += dth*dt;
    
    //将角度变为四元数
    tf2::Quaternion q;
    q.setRPY(0,0,th);
    
    nav_msgs::Odometry odom;
    odom.child_frame_id = "base_link";
    odom.header.frame_id = "odom";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.w = q.getW();
    odom.pose.pose.orientation.x = q.getX();
    odom.pose.pose.orientation.y = q.getY();
    odom.pose.pose.orientation.z = q.getZ();
    
    odom.twist.twist.linear.x = dx;
    odom.twist.twist.linear.y = dy;
    odom.twist.twist.angular.z = dth;
    g_odomPub.publish(odom);
    
    //printf("%.3f,%.3f,%.3f\r\n",x,y,th);
    //发布tf变换
    geometry_msgs::TransformStamped odomTrans;
    odomTrans.child_frame_id = "base_link";
    odomTrans.header.frame_id = "odom";
    odomTrans.header.stamp  = ros::Time::now();
    odomTrans.transform.rotation.w = q.getW();
    odomTrans.transform.rotation.x = q.getX();
    odomTrans.transform.rotation.y = q.getY();
    odomTrans.transform.rotation.z = q.getZ();
    
    odomTrans.transform.translation.x = x;
    odomTrans.transform.translation.y = y;
    odomTrans.transform.translation.z = 0;
    
    g_odomBroadcaster->sendTransform(odomTrans);
  
}
void HeroChassisController::calWheelSpeed(const ros::Duration& period){

  //添加加速度程序
  
  /*double now_vx = vax + acc*dt;
  
  if(now_vx < g_vel.linear.x){
      g_vel.linear.x = now_vx;
  }*/

  //分别计算四个轮的速度
  command_[2] = (g_vel.linear.x - g_vel.linear.y + g_vel.angular.z*(wheelBase/2+trackWidth/2))/radius; //d
  command_[0] = (g_vel.linear.x + g_vel.linear.y - g_vel.angular.z*(wheelBase/2+trackWidth/2))/radius; //c
  command_[1] = (g_vel.linear.x - g_vel.linear.y - g_vel.angular.z*(wheelBase/2+trackWidth/2))/radius; //a
  command_[3] = (g_vel.linear.x + g_vel.linear.y + g_vel.angular.z*(wheelBase/2+trackWidth/2))/radius; //b
  
  //计算实际速度与理论PID的误差值，并把速度发送给Joint
  double error = command_[0] - left1_joint_.getVelocity();
  double commanded_effort = pid1_controller_.computeCommand(error, period);
  left1_joint_.setCommand(commanded_effort);
  
  error = command_[1] - left2_joint_.getVelocity();
  commanded_effort = pid2_controller_.computeCommand(error, period);
  left2_joint_.setCommand(commanded_effort);
  
  error = command_[2] - right1_joint_.getVelocity();
  commanded_effort = pid3_controller_.computeCommand(error, period);
  right1_joint_.setCommand(commanded_effort);
  
  error = command_[3] - right2_joint_.getVelocity();
  commanded_effort = pid4_controller_.computeCommand(error, period);
  right2_joint_.setCommand(commanded_effort);
  
}
void HeroChassisController::velCallBack(const geometry_msgs::TwistConstPtr msg){
    g_vel = *msg;    
}
} // namespace

PLUGINLIB_EXPORT_CLASS( hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
