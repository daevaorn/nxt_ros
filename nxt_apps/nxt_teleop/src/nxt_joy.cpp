#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

class NxtTeleop
{
public:
  NxtTeleop();

private:
  void joyCallback(const joy::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle nh_;

  int linear_, angular_, deadman_axis_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  ros::Timer timer_;
  
};

NxtTeleop::NxtTeleop():
  linear_(1),
  angular_(2),
  deadman_axis_(0),
  l_scale_(50.0),
  a_scale_(2.0)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("axis_deadman", deadman_axis_, deadman_axis_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &NxtTeleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&NxtTeleop::publish, this));
}

void NxtTeleop::joyCallback(const joy::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];

}

void NxtTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);  
  if (deadman_pressed_)
    {
      vel_pub_.publish(last_published_);
    }
  
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "nxt_teleop");
  NxtTeleop nxt_teleop;

  ros::NodeHandle n;
  

  ros::spin();
}
