#include <stdio.h>
#include <string.h>
#include <time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <opencv/cvwimage.h>
#include <opencv/cxcore.h>
#include <opencv/cxmisc.h>
#include <opencv/ml.h>

#define LEFT_CAP 0
#define RIGHT_CAP 2

//g++ stereo_capture.cpp `pkg-config --cflags --libs opencv-2.3.1` -o stereo_capture

using namespace cv;

int main(int argc, char ** argv)
{
  VideoCapture capl(LEFT_CAP);
  VideoCapture capr(RIGHT_CAP);
  if(!capl.isOpened()) return 1;// checa se capturou
  if(!capr.isOpened()) return 2;// checa se capturou

  Mat framel, framer;
  
  printf("3...\n\n");
  waitKey(1500);
  
  printf("2...\n\n");
  waitKey(1500);
  
  printf("1...\n\n");
  waitKey(1500);
  
  printf("Smile, bitch.\n\n");
  
  capl >> framel;
  capr >> framer;
  
  imwrite("left.png", framel);
  imwrite("right.png", framer);
}