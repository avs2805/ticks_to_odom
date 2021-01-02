#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class Accio_odom
{

public:
  Accio_odom();
  void odom_spin();

private:
  ros::NodeHandle n;
  ros::Subscriber lWheelSub;
  ros::Subscriber rWheelSub;
  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;

  // odometry related variables distances, velocities
  double d_left, v_left, d_right, v_right, dc, vc, phi, omega;
  // encoder related variables
  long _prev_left_encoder, _prev_right_encoder;
  // coordinate frame variables
  double x, y, theta, vx, vy, vth;

  // robot constants
  double WHEEL_DIAMETER;
  double WHEEL_TRACK;
  double TPR;
  double TICKS_TO_METERS;

  // initialize variables for elapsed time
  ros::Time current_time, prev_time;

  // methods in class
  void lwheel_ticks_cb(const std_msgs::Int32::ConstPtr &lwheel_ticks_data);
  void rwheel_ticks_cb(const std_msgs::Int32::ConstPtr &rwheel_ticks_data);
  void odometry_update();
  void init_vars();
};

// constructor
Accio_odom::Accio_odom()
{
  // initialize variables
  Accio_odom::init_vars();

  // wheel ticks subscriber
  lWheelSub = n.subscribe("lWheelTicks", 10, &Accio_odom::lwheel_ticks_cb, this);
  rWheelSub = n.subscribe("rWheelTicks", 10, &Accio_odom::rwheel_ticks_cb, this);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

  ROS_INFO("Odometry Publisher Intialized");
}

void Accio_odom::init_vars()
{
  d_left, v_left, d_right, v_right, dc, vc, phi, omega = 0.0;
  _prev_left_encoder, _prev_right_encoder = 0;
  x, y, theta = 0.0;
  vx, vth = 0.1;
  vy = -0.1;

  WHEEL_DIAMETER = 0.1372; //0.1372
  WHEEL_TRACK = 0.254; // Distance between wheels
  TPR = 644.42; // ticks per revolution 644.424
  TICKS_TO_METERS = (3.141592654 * 0.125 )/ 644.424; // meters/tick
  current_time = ros::Time::now();
  prev_time = ros::Time::now();
}

void Accio_odom::lwheel_ticks_cb(const std_msgs::Int32::ConstPtr &lwheel_ticks_data)
{
  int _curr_lwheel_count = lwheel_ticks_data->data;
  int _delta_l_enc = _curr_lwheel_count - _prev_left_encoder;
  d_left = TICKS_TO_METERS * _delta_l_enc;
  _prev_left_encoder = _curr_lwheel_count;
}

void Accio_odom::rwheel_ticks_cb(const std_msgs::Int32::ConstPtr &rwheel_ticks_data)
{
  int _curr_rwheel_count = rwheel_ticks_data->data;
  int _delta_r_enc = _curr_rwheel_count - _prev_right_encoder;
  d_right = TICKS_TO_METERS * _delta_r_enc;
  _prev_right_encoder = _curr_rwheel_count;
}

void Accio_odom::odom_spin()
{

  ros::Rate loop_rate(10.0);

  while (ros::ok())
  {
    odometry_update();
    loop_rate.sleep();
  }
}

void Accio_odom::odometry_update()
{

  current_time = ros::Time::now();

  // get elapsed time
  double dt = (current_time - prev_time).toSec();

  // get velocities from distances travelled
  v_left = d_left / dt;
  v_right = d_right / dt;

  // velocities in x & y
  vc = (v_left + v_right)*0.5;
    // get rotation of the center of the robot
  vth = (v_right - v_left) / WHEEL_TRACK;

  // distance moved in x & y
  x += vc * cos(theta) * dt;
  y += vc * sin(theta) * dt;
  theta += vth * dt;

  // create quaternion from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

  // publish transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // broadcast the transform
  odom_broadcaster.sendTransform(odom_trans);

  //publish odometry message
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //odom positions
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //odom velocities
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = omega;

  //publish the message
  odom_pub.publish(odom);

  prev_time = current_time;

  // ROS_INFO("dt: %f seconds | d_left: %f m at %f m/s | d_right: %f m at %f m/s |", dt,d_left, v_left, d_right, v_right);

  ros::spinOnce();
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "accio_odom");

  Accio_odom accio_obj;

  accio_obj.odom_spin();

  return 0;
}