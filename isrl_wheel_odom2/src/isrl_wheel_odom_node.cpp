#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>

float x = 0.0;
float y = 0.0;
float th = 0.0;

float vx = 0.0;
float vy = 0.0;
float vth = 0.0;

float ur = 0.0;
float ul = 0.0;

const float wheel_radius = 0.115;
const float wheel_distance = 0.385;
const int max_pulse = 420;

void odomCallback(const std_msgs::Float32MultiArray&);
void imuCallback(const sensor_msgs::Imu&);

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  // ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom_raw", 50);
  ros::Subscriber vel_sub = nh.subscribe("motor_angular_vel_data", 100, odomCallback);
  ros::Subscriber imu_sub = nh.subscribe("imu/data", 100, imuCallback);

  tf::TransformBroadcaster odom_broadcaster;

  // float* ptr_x = &x;
  // float* ptr_y = &y;
  // float* ptr_th = &th;
  //
  // float* ptr_vx = &vx;
  // float* ptr_vy = &vy;
  // float* ptr_vth = &vth;
  //
  // float* ptr_ur = &ur;
  // float* ptr_ul = &ul;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);
  while(nh.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    // ROS_INFO("out of function | vth : %f  vx : %f  vy : %f", vth, vx, vy);

    //compute odometry in a typical way given the velocities of the robot
    float dt = (current_time - last_time).toSec();
    // float delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    // float delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    // float delta_th = vth * dt;
    float delta_x = vx * dt;  // 변위 계산
    float delta_y = vy * dt;
    float delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    // odom_trans.header.frame_id = "odom_raw";
    odom_trans.child_frame_id = "base_footprint";
    // odom_trans.child_frame_id = "base_footprint_raw";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    // odom.header.frame_id = "odom_raw";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance = {1e-2,    0,    0,    0,    0,    0,
                               0, 1e-2,    0,    0,    0,    0,
                               0,    0,    0,    0,    0,    0,
                               0,    0,    0,    0,    0,    0,
                               0,    0,    0,    0,    0,    0,
                               0,    0,    0,    0,    0,    1e-2};

    //set the velocity
    odom.child_frame_id = "base_footprint";
    // odom.child_frame_id = "base_footprint_raw";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    odom.twist.covariance = {1e-2,       0,    0,    0,    0,    0,
                                0,    1e-2,    0,    0,    0,    0,
                                0,       0,    0,    0,    0,    0,
                                0,       0,    0,    0,    0,    0,
                                0,       0,    0,    0,    0,    0,
                                0,       0,    0,    0,    0,    1e-2};

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}

void odomCallback(const std_msgs::Float32MultiArray& vel){
  ur = vel.data[0];
  ul = vel.data[1];

  //vth = (wheel_radius/wheel_distance)*(ur-ul);
  vx = (wheel_radius/2)*(ur+ul)*cos(th);
  vy = (wheel_radius/2)*(ur+ul)*sin(th);
  ROS_INFO("velocity    | ur : %f  ul : %f  vth : %f  vx : %f  vy : %f", ur, ul, vth, vx, vy);
  ROS_INFO("wheel odometry | x : %f  y : %f  th : %f", x, y, th);
}

void imuCallback(const sensor_msgs::Imu& data){
  vth = data.angular_velocity.z;
}
