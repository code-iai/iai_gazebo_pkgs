#include <iai_gazebo_visibility_mover/pixel_counter.hpp>
#include <opencv2/highgui/highgui.hpp>
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
  ROS_INFO_STREAM("[" << nh_.getNamespace() << "] Processing image.");

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

  std_msgs::UInt64 out_msg;
  out_msg.data = count_pixels(cv_ptr);
  blue_pixel_publisher_.publish(out_msg);
}

size_t PixelCounter::count_pixels(const cv_bridge::CvImagePtr& img)
{
  // TODO: implement me
  return 125;
}
