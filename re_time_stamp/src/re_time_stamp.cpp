#include <ros/ros.h>
#include <std_msgs/Time.h>
#include "rosgraph_msgs/Clock.h"

#include <sensor_msgs/LaserScan.h>
#include <time.h>


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //퍼블리쉬 할 토픽 선언
    pub_ = n_.advertise<rosgraph_msgs::Clock>("/clock", 10);

    //섭스크라이브 할 토픽픽 선언
    sub_ = n_.subscribe("/scan", 10, &SubscribeAndPublish::callback, this);
  }

  void callback(const sensor_msgs::LaserScan& input)
  {
    // sensor_msgs::LaserScan output;
    // output.header.stamp = ros::Time::now();
    // output.ranges = input.ranges;
    std_msgs::Time time;
    time.data = input.header.stamp;
    //callback 함수에서 받은 input을 사용해서 output을 만들고 이를 pub한다.
    pub_.publish(time);
  }

private: //private으로 NodeHandle과 publisher, subscriber를 선언한다.
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "re_time_stamp");
  SubscribeAndPublish SAPObject; //클래스 객체 선을 하게 되면 모든게 된다.
  ros::spin();
  return 0;
}