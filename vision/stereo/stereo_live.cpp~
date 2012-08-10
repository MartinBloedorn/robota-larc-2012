#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string>
//#include <pcl/common/common_headers.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <stdio.h>

#define LEFT  0
#define RIGHT 2

using namespace cv;

//fockin' variables
Mat M1, D1, M2, D2, Q, R, T, R1, P1, R2, P2, frame_l, frame_r;
Rect roi1, roi2;
StereoBM bm;
StereoSGBM sgbm;
StereoVar var;

char * intrinsic_filename = "intrinsics.yml";
char * extrinsic_filename = "extrinsics.yml";

int main(int argc, char** argv)
{
  enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
  
  int alg = STEREO_SGBM;
  
  int SADWindowSize = 0, numberOfDisparities = 0;
  bool no_display = false;
  float scale = 1.f;
  
  //Starting video captures
  VideoCapture l_cap(LEFT);
  VideoCapture r_cap(RIGHT);
  if(!l_cap.isOpened()) return LEFT;
  if(!r_cap.isOpened()) return RIGHT;
  
  // reading intrinsic parameters
  FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
  if(!fs.isOpened())
  {
      printf("Failed to open file %s\n", intrinsic_filename);
      return -1;
  }

  fs["M1"] >> M1;
  fs["D1"] >> D1;
  fs["M2"] >> M2;
  fs["D2"] >> D2;

  fs.open(extrinsic_filename, CV_STORAGE_READ);
  if(!fs.isOpened())
  {
      printf("Failed to open file %s\n", extrinsic_filename);
      return -1;
  }
   
  fs["R"] >> R;
  fs["T"] >> T;
  
  //main capture - rectify - match loop
  while( !(waitKey(10) == 'q') )
  {
    l_cap >> frame_l;
    r_cap >> frame_r;
    
    //imshow("Left",  frame_l);    
    //imshow("Right", frame_r);  
    
    Size img_size = frame_l.size();
    
    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
    
    Mat map11, map12, map21, map22;
    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
    
    Mat frame_lr, frame_rr;
    remap(frame_l, frame_lr, map11, map12, INTER_LINEAR);
    remap(frame_r, frame_rr, map21, map22, INTER_LINEAR);
    
    imshow("Left Rectified",  frame_lr);    
    imshow("Right Rectified", frame_rr);     
    
    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
    
    int cn = frame_l.channels();

    //Configure!    
    bm.state->roi1 = roi1;
    bm.state->roi2 = roi2;
    bm.state->preFilterCap = 31;
    bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
    bm.state->minDisparity = 0;
    bm.state->numberOfDisparities = numberOfDisparities;
    bm.state->textureThreshold = 10;
    bm.state->uniquenessRatio = 15;
    bm.state->speckleWindowSize = 100;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1;
    
    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = numberOfDisparities;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = bm.state->speckleWindowSize;
    sgbm.speckleRange = bm.state->speckleRange;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = alg == STEREO_HH;

    var.levels = 3;									// ignored with USE_AUTO_PARAMS
    var.pyrScale = 0.5;								// ignored with USE_AUTO_PARAMS
    var.nIt = 25;
    var.minDisp = -numberOfDisparities;	
    var.maxDisp = 0;
    var.poly_n = 3;
    var.poly_sigma = 0.0;
    var.fi = 15.0f;
    var.lambda = 0.03f;
    var.penalization = var.PENALIZATION_TICHONOV;	// ignored with USE_AUTO_PARAMS
    var.cycle = var.CYCLE_V;						// ignored with USE_AUTO_PARAMS
    var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;    
      
    Mat disp, disp8;
    
    if( alg == STEREO_BM )
        bm(frame_lr, frame_rr, disp);
    else if( alg == STEREO_VAR ) {
        var(frame_lr, frame_rr, disp);
	}
    else if( alg == STEREO_SGBM || alg == STEREO_HH )
        sgbm(frame_lr, frame_rr, disp);
    
    if( alg != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else
        disp.convertTo(disp8, CV_8U);    
    
    imshow("disparity", disp8);
  }
}