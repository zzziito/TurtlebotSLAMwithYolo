#include <ros/ros.h>
#include <image_transport/camera_common.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <laser_geometry/laser_geometry.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <iostream>

#include <yoloturtle/filteredbBox.h>
using namespace std;


class YoloTurtle {
private: 
    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;
    Subscriber<Image> image_sub_;
    image_transport::PUblisher image_pub_;

    Subscriber<CameraInfo> info_sub_;
    Subscriber<LaserScan> scan_sub_;

    //publish 할 토픽 목록

    Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes_sub_;

    

}