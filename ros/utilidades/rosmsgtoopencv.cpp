#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
/*
*
*
* Captura da imagem do gscam e transformação em imagem do opencv funcionando
*
*/




namespace enc = sensor_msgs::image_encodings;
using namespace cv;

cv_bridge::CvImagePtr cv_ptr;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("camera_patrick", 1, &ImageConverter::imageCb, this);
    //cv::namedWindow(WINDOW);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    Mat frame;
    frame=cv_ptr->image;
    /*
    *
    *
    *
    *                             IMPLEMENTE O PROGRAMA AQUI
    *
    *
    *
    */
    if (frame.rows > 60 && frame.cols > 60)   cv::circle(frame, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    imshow("img",frame);
    cv::waitKey(3); 
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_rostocv");
  ImageConverter ic;
  ros::spin();
  return 0;
}
