#include <iai_gazebo_visibility_mover/pixel_counter.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt64.h>

using namespace iai_gazebo;

PixelCounter::PixelCounter(const ros::NodeHandle& nh) :
    nh_( nh ), it_( nh )
{
}

PixelCounter::~PixelCounter()
{
}

bool PixelCounter::start()
{
  image_subscriber_ = it_.subscribe("image", 1, &PixelCounter::image_callback, this);

  blue_pixel_publisher_ = nh_.advertise<std_msgs::UInt64>("blue_pixels", 1);

  return true;
}

void PixelCounter::image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("[" << nh_.getNamespace() << "] Processing image.");

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  std::vector<cv::Mat> spl(3);
  cv::split(cv_ptr->image, spl);

  cv::Mat blue_channel_blur, blue_channel_thresh;
  cv::medianBlur(spl[0].clone(), blue_channel_blur, 25);
  cv::threshold(blue_channel_blur.clone(), blue_channel_thresh, 180, 255, 0);

//  cv::imwrite("/tmp/images/original.jpg", cv_ptr->image);
//  cv::imwrite("/tmp/images/blue_channel.jpg", spl[0]);
//  cv::imwrite("/tmp/images/blue_channel_blur.jpg", blue_channel_blur);
//  cv::imwrite("/tmp/images/blue_channel_thresh.jpg", blue_channel_thresh);

  std_msgs::UInt64 out_msg;
  out_msg.data = cv::countNonZero(blue_channel_thresh);
  ROS_INFO_STREAM("[" << nh_.getNamespace() << "] Counted " << out_msg.data << " blue pixels.");
  blue_pixel_publisher_.publish(out_msg);
}
