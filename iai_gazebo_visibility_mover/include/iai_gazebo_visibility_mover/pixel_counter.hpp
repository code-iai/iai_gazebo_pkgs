#ifndef IAI_GAZEBO_PIXEL_COUNTER_HPP
#define IAI_GAZEBO_PIXEL_COUNTER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace iai_gazebo
{
  class PixelCounter
  {
    public:
      PixelCounter(const ros::NodeHandle& nh);
      ~PixelCounter();
      bool start();

    private:
      // Comunications
      ros::NodeHandle nh_;
      ros::Publisher blue_pixel_publisher_;
      image_transport::ImageTransport it_;
      image_transport::Subscriber image_subscriber_;

      // Callbacks
      void image_callback(const sensor_msgs::Image::ConstPtr& msg);

      // Auxs
      size_t count_pixels(const cv_bridge::CvImagePtr& img);
  };
}

#endif // IAI_GAZEBO_PIXEL_COUNTER_HPP
