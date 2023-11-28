#include <std_msgs/String.h>  
#include "ros/ros.h"  
#include "sentry_controller/sentry_control_key.h"
#include "string"


std::string cmd_vel_topic;
int velocity_linear;
int velocity_angular;

KeyboardReader input;


TeleopTurtle::TeleopTurtle():
  linear_x(0),
  linear_y(0),
  angular_(0),
  l_scale_(10.0),
  a_scale_(10.0)
{
  
  nhPrivate.getParam("cmd_vel_topic", cmd_vel_topic);
  nhPrivate.getParam("velocity_linear", velocity_linear);
  nhPrivate.getParam("velocity_angular", velocity_angular);
  l_scale_ = velocity_linear;
  a_scale_ = velocity_angular;
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
}

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sentry_control_node");
  TeleopTurtle sentry_control_node;

  signal(SIGINT,quit);

  sentry_control_node.keyLoop();
  quit(0);
  
  return(0);
}


void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the sentry in x,y direction.");
  puts("Use Q/E keys to turn left/right.");
  puts("press 'c' to stop.");



  for(;;)
  {
    // get the next event from the keyboard  
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return;
    }

    linear_x=linear_y=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_LEFT:
        ROS_DEBUG("LEFT");
        linear_y = 1.0;
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        ROS_DEBUG("RIGHT");
        linear_y = -1.0;
        dirty = true;
        break;
      case KEYCODE_UP:
        ROS_DEBUG("UP");
        linear_x = 1.0;
        dirty = true;
        break;
      case KEYCODE_DOWN:
        ROS_DEBUG("DOWN");
        linear_x = -1.0;
        dirty = true;
        break;
      case KEYCODE_Q:
        ROS_DEBUG("TURN_LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_E:
        ROS_DEBUG("TURN_RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_C:
        ROS_DEBUG("quit");
        angular_ = 0.0;
        linear_x = 0.0;
        linear_y = 0.0;
        dirty = true;
        break;

    }
   

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_x;
    twist.linear.y = l_scale_*linear_y;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
  }


  return;
}