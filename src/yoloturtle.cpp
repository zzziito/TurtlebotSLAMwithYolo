#include "ros/ros.h"

#include <image_transport/camera_common.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>


#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/pinhole_camera_model.h>
#include <laser_geometry/laser_geometry.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

// #include <sensor_msgs/Header>

#include <iostream>

#include <custom_msgs/CustomMsg.h>
#include "std_msgs/String.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

enum pointClass {ptOutOfImage = 0, ptStructural = 1, ptNonStructural = 2};

pointClass getPointClass(const cv::Point2d& point, 
                        unsigned imgWidth, unsigned imgHeight,
                        const darknet_ros_msgs::BoundingBoxes& bounding_boxes)
{
if (point.x < 0 || point.x >= imgWidth || point.y < 0 || point.y >= imgHeight) {
    return ptOutOfImage;
}
else {
    for (unsigned idx = 0; idx < bounding_boxes.bounding_boxes.size(); idx++) {
        const darknet_ros_msgs::BoundingBox &boundingBox = bounding_boxes.bounding_boxes.at(idx);
        if (point.x >= boundingBox.xmin && point.x <= boundingBox.xmax && point.y >= boundingBox.ymin && point.y <= boundingBox.ymax) {
            return ptNonStructural;
        }
    }
    return ptStructural;
}
}

// void chatterCallback(const custom_msgs::CustomMsg::ConstPtr& custom_msg){
//     ROS_INFO("I heard: [%d]", custom_msg->xmin);
// }


void callback (const ImageConstPtr& image, const CameraInfoConstPtr& cam_info, const LaserScanConstPtr& scan_msg, const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_boxes_msg)
{
    cout << "inside callback" << endl;
    // ROS_INFO("I heard: [%d]", custom_msg->xmin);


    
}

// void callback (const ImageConstPtr& image, const CameraInfoConstPtr& cam_info, const LaserScanConstPtr& scan_msg, const custom_msgs::CustomMsg::ConstPtr& custom_msg)
// {
//     cout << "inside callback" << endl;
//     // ROS_INFO("I heard: [%d]", custom_msg->xmin);
// }

// void callback (const LaserScanConstPtr& scan_msg, const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_boxes_msg)
// {
//     cout << "inside callback" << endl;
// }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "yoloturtle");
    ros::NodeHandle nh;
    

    message_filters::Subscriber<Image> image_sub(nh, "/image_raw", 1);
    message_filters::Subscriber<CameraInfo> info_sub(nh, "/camera_info", 1);
    message_filters::Subscriber<LaserScan> scan_sub(nh, "/scan", 1);
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes_sub(nh, "/darknet_ros/bounding_boxes", 1);
    typedef sync_policies::ApproximateTime<Image, CameraInfo, LaserScan, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub, scan_sub, bounding_boxes_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    cout << "initialized." << endl;
    image_geometry::PinholeCameraModel cam_model_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tf_listener_;

    // message_filters::Subscriber<custom_msgs::CustomMsg> bounding_box_sub(nh, "/custom_topic", 1);
    // typedef sync_policies::ApproximateTime<Image, CameraInfo, LaserScan> MySyncPolicy;
    // typedef sync_policies::ApproximateTime<LaserScan, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;


    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(80), image_sub, info_sub, scan_sub);
    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), scan_sub, bounding_boxes_sub);

    // TimeSynchronizer<Image, CameraInfo, LaserScan> sync(image_sub, info_sub, scan_sub, 10);   
    // sync.registerCallback(boost::bind(&callback, _1, _2));
    
    // ros::Subscriber sub = nh.subscribe("/custom_topic", 1000, chatterCallback);

    ros::spin();
    return 0;
}