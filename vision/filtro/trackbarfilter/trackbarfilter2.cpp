#include <stdio.h>
#include <string.h>
#include <time.h>
//#include <cv.h>
//#include <highgui.h>
//#include <iostream>
//#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <opencv/cvwimage.h>
#include <opencv/cxcore.h>
#include <opencv/cxmisc.h>
#include <opencv/ml.h>

#define WIN_SCALE 3
#define CAMERA    0

using namespace cv;
cv::Mat H, S, fH, fS, img;
int h_thres_value = 100;
int s_thres_value = 35;
int h_clsg_iters = 3;
int s_clsg_iters = 2;
int h_size = 0;
int s_size = 0;

//g++ trackbarfilter.cpp `pkg-config --cflags --libs opencv-2.3.1` -o tfilter

void filter_images(int, void*)
{
  //(src, dst)
  //Apply changes to H
  if( h_thres_value != 0)  {  
    threshold( H, fH, h_thres_value , 255, THRESH_BINARY_INV );
    
    if( h_clsg_iters != 0) {
      int szh = (h_size + 1)*2 + 1;
      Mat kernelh(Size(szh, szh), CV_8UC1);
      dilate(fH, fH, Mat(), Point(-1, -1), h_clsg_iters);
      erode(fH, fH, Mat(), Point(-1, -1), h_clsg_iters);      
    }
    
    imshow("H", fH);
  }
  else
    imshow("H", H);
 
  //cvResizeWindow("H", H.cols/WIN_SCALE, H.rows/WIN_SCALE);
  cvResizeWindow("H", 640, 480);
  //Apply changes to S
  if( s_thres_value != 0)  {  
    threshold( S, fS, s_thres_value , 255, THRESH_BINARY_INV );
    
    if( s_clsg_iters != 0) {
      int szs = (s_size + 1)*2 + 1;
      Mat kernels(Size(szs, szs), CV_8UC1);
      dilate(fS, fS, Mat(), Point(-1, -1), s_clsg_iters);
      erode(fS, fS, Mat(), Point(-1, -1), s_clsg_iters);      
    }
    
    imshow("S", fS);
  }
  else 
    imshow("S", S);
  //const Mat Dest;
  //cvAnd( S, H, Dest,0 );
  //cvResizeWindow("S", S.cols/WIN_SCALE, S.rows/WIN_SCALE);
  Mat Dest;
  Mat fHl=fH;
  for (int i = 0; i < fH.rows; ++i)
  {
    for (int j = 0; j < fH.cols; ++j)
    {
      if (fH.at<cv::Vec3b>(i,j)[0]==255)
      {
        fHl.at<cv::Vec3b>(i,j)[0]=fHl.at<cv::Vec3b>(i,j)[1]=fHl.at<cv::Vec3b>(i,j)[2]=125;  
      }
    }
  }

  compare(fHl,fS,Dest,CMP_NE);
  Mat Dest1;
  compare(fHl,Dest,Dest1,CMP_NE);
  imshow("lalala1",Dest1);
  cvResizeWindow("S", 640, 480);
}

int main(int argc, char** argv){

  VideoCapture cap(CAMERA);
  
  if( strcmp(argv[1], "live") != 0) {
    img = imread(argv[1]);
    imshow("Original", img);
    cvResizeWindow("Original", 640, 480);
  }
  
  cv::Mat f_img(Size(img.cols, img.rows), CV_8UC1);
 
  //LOG Filter (Gray -> Gauss -> Laplace)
  cvtColor(img, img, CV_BGR2HSV);
  std::vector<cv::Mat> three_channels;
 
  cvNamedWindow("H", 0);  
  cvNamedWindow("S", 0);  
  
  createTrackbar( "H Threshold", "H", &h_thres_value, 255, filter_images ); 
  createTrackbar( "H Closings",  "H", &h_clsg_iters,  15,  filter_images ); 
  createTrackbar( "H Kernel Size",  "H", &h_size,  4,  filter_images ); 
  
  createTrackbar( "S Threshold", "S", &s_thres_value, 255, filter_images );
  createTrackbar( "S Closings",  "S", &s_clsg_iters,  15,  filter_images ); 
  createTrackbar( "S Kernel Size",  "S", &s_size,  4,  filter_images ); 

  //aplly shit to H channel
  //f_img = img.clone();
  //threshold( three_channels[1], f_img, 60 , 255, THRESH_BINARY_INV );
  //dilate(f_img, f_img, Mat(), Point(-1, -1), 5);
  //erode(f_img, f_img, Mat(), Point(-1, -1), 5);
  //blur(f_img, f_img, Size(15,15));
  //Laplacian( f_img, f_img, CV_8UC1, /*kernel_size*/3, /*scale*/0, /*delta*/0, BORDER_DEFAULT );
    
  split(img, three_channels);
  H = three_channels[0];
  S = three_channels[1];  
  
  //imshow("Original image", img);
  while(waitKey(15) != 'q') { 
    
      //LIVE MODE NOT YET WORKING
      if( strcmp(argv[1], "live") == 0) {
	cap >> img;
	cv::Mat f_img(Size(img.cols, img.rows), CV_8UC1);
	cvtColor(img, img, CV_BGR2HSV);
	split(img, three_channels);
	H = three_channels[0];
	S = three_channels[1]; 
      }
	
      filter_images(0, NULL);
  }
  return 0;
}