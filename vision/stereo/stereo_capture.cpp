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

int n_images;
int save_count = 0;

char file_l[64];
char file_r[64];

Size boardSize;
vector<vector<Point2f> > imagePoints[2];
vector<Point2f> corners;
Mat l_frame, r_frame;

bool found_l, found_r;

int main(int argc, char ** argv)
{
  if( argc != 2 ) {
      printf("ERROR: Wrong number of input parameters\n");
      return -1;
  }
    
  n_images = atoi( argv[1] );
  boardSize.width = 7;
  boardSize.height = 4;
  
  VideoCapture  left_cap(LEFT_CAP); // Abre a camera 0 e bota m cap
  VideoCapture right_cap(RIGHT_CAP); // Abre a camera 0 e bota m cap
  if(!left_cap.isOpened()) return 1;// checa se capturou
  if(!right_cap.isOpened()) return 2;// checa se capturou

  while(n_images != 0)
  {
    left_cap >>  l_frame;
    right_cap >> r_frame;

    imshow("Left", l_frame);    //camera 1
    imshow("Right", r_frame);    //camera 2
  
    if(waitKey(10) >= 100) break;   //apertar um botão por algum tempo finaliza as cameras

    found_l = findChessboardCorners(l_frame, boardSize, corners, 
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
    if(found_l) {
      found_r = findChessboardCorners(r_frame, boardSize, corners, 
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
    }
    
    
    if( found_l != 0 && found_r != 0 ) {
      printf("Got chessboard. Capturing...\n");
      sprintf(file_l, "left%d.png", n_images);
      sprintf(file_r, "right%d.png", n_images);

      imwrite(file_l, l_frame);
      imwrite(file_r, r_frame);
    
      printf("Done, %d images to go\n", n_images);
      
      save_count = 0;
      n_images--;
      printf("GET READY\n\n");
      waitKey(2000);
      printf("Searching\n");
    }
    
    
  
    
  } 
  
  return(0);
}