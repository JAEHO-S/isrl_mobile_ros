#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub_image;
ros::Publisher pub_depth;
ros::Publisher pub_camera_info;
ros::Publisher pub_scan;

void callback(const sensor_msgs::ImageConstPtr& image,
              const sensor_msgs::ImageConstPtr& depth,
              const sensor_msgs::CameraInfoConstPtr& camera_info,
              const sensor_msgs::LaserScanConstPtr& scan) {
    // 동기화된 데이터 퍼블리시
    pub_image.publish(image);
    pub_depth.publish(depth);
    pub_camera_info.publish(camera_info);
    pub_scan.publish(scan);
    // ROS_INFO("Published synchronized data.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sync_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/zed_node/rgb/image_rect_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/zed_node/depth/depth_registered", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub(nh, "/zed_node/rgb/camera_info", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 1);

    pub_image = nh.advertise<sensor_msgs::Image>("/sync/rgb/image_rect_color", 1);
    pub_depth = nh.advertise<sensor_msgs::Image>("/sync/depth/depth_registered", 1);
    pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>("/sync/rgb/camera_info", 1);
    pub_scan = nh.advertise<sensor_msgs::LaserScan>("/sync/scan", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                            sensor_msgs::Image,
                                                            sensor_msgs::CameraInfo,
                                                            sensor_msgs::LaserScan> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, depth_sub, camera_info_sub, scan_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    ros::spin();

    return 0;
}
