#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
/*
*
*
* Captura da imagem por opencv p/ rosmsg
*
*/

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";
using namespace cv;

Mat frame;
VideoCapture cap(0);
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("camera_patrick", 1);
    cv_bridge::CvImage cv_ptr;                  //
    ros::Time time = ros::Time::now();          //ARRUMANDO OS STEPS
    cv_ptr.header.stamp = time;                 //
    cv_ptr.header.frame_id = "camera";          //
    cv_ptr.encoding = "bgr8";                   //
    while("1")
      {
      cap >> cv_ptr.image;
      if (cv_ptr.image.rows > 60 && cv_ptr.image.cols > 60) cv::circle(cv_ptr.image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      //VideoCapture cap(0);
      cv::Mat lala=cv_ptr.image;
      //cv::imshow("MAGAIVER", lala);                  // MOSTRAR IMG DE CAPTURA
      //sensor_msgs::Image im;
      //cv_ptr.toImageMsg(im);
      image_pub_.publish(cv_ptr.toImageMsg());
      if(waitKey(30) >= 100) break;
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_cvtoros");
  ImageConverter ic;
  return 0;
}
