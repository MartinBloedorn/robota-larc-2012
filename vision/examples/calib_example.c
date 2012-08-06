// calib.cpp
// Calling convention:
// calib board_w board_h number_of_views
//
// Hit ‘p’ to pause/unpause, ESC to quit
//
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>

//COMPILE W/ gcc calib_example.c -std=c99  `pkg-config --cflags --libs opencv-2.3.1` -o calib

int n_boards = 0; //Will be set by input list
const int board_dt = 20; //Wait 20 frames per chessboard view
int board_w;
int board_h;

int main(int argc, char* argv[]) {
  
  if(argc != 4){
    printf("ERROR: Wrong number of input parameters\n");
    return -1;
  }
  
  board_w = atoi(argv[1]);
  board_h = atoi(argv[2]);
  n_boards = atoi(argv[3]);
  int board_n = board_w * board_h;
  
  CvSize board_sz = cvSize( board_w, board_h );
  CvCapture* capture = cvCreateCameraCapture( 0 );
  //system("ls /dev/video*");
  assert( capture );
  cvNamedWindow( "Calibration", 0 );
  
  //ALLOCATE STORAGE
  CvMat* image_points = cvCreateMat(n_boards*board_n,2,CV_32FC1);
  CvMat* object_points = cvCreateMat(n_boards*board_n,3,CV_32FC1);
  CvMat* point_counts = cvCreateMat(n_boards,1,CV_32SC1);
  CvMat* intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);
  CvMat* distortion_coeffs = cvCreateMat(5,1,CV_32FC1);
  CvPoint2D32f corners[board_n];// = CvPoint2D32f[ board_n ];
  int corner_count;
  int successes = 0;
  int step, frame = 0;
  
  IplImage *image = cvQueryFrame( capture );
  IplImage *gray_image = cvCreateImage(cvGetSize(image),8,1);//subpixel
  // CAPTURE CORNER VIEWS LOOP UNTIL WE’VE GOT n_boards
  // SUCCESSFUL CAPTURES (ALL CORNERS ON THE BOARD ARE FOUND)
  //
  while(successes < n_boards) {
    
    //Skip every board_dt frames to allow user to move chessboard
    if(frame++ % board_dt == 0) {
      //Find chessboard corners:
      int found = cvFindChessboardCorners(
      image, board_sz, corners, &corner_count,
      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
      );
      //Get Subpixel accuracy on those corners
      cvCvtColor(image, gray_image, CV_BGR2GRAY);
      cvFindCornerSubPix(gray_image, corners, corner_count,
      cvSize(11,11),cvSize(-1,-1), cvTermCriteria(
      CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
      //Draw it
      cvDrawChessboardCorners(image, board_sz, corners,
      corner_count, found);
      cvShowImage( "Calibration", image );
      // If we got a good board, add it to our data
      if( corner_count == board_n ) {
	step = successes*board_n;
	for( int i=step, j=0; j<board_n; ++i,++j ) {
	  CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;
	  CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;
	  CV_MAT_ELEM(*object_points,float,i,0) = j/board_w;
	  CV_MAT_ELEM(*object_points,float,i,1) = j%board_w;
	  CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
	  }
	CV_MAT_ELEM(*point_counts, int,successes,0) = board_n;
	successes++;
	printf("Got good board! %d to go.\n", (n_boards - successes));
	}
    } //end skip board_dt between chessboard capture
    
    //Handle pause/unpause and ESC
    int c = cvWaitKey(15);
    if(c == 'p'){
      c = 0;
      while(c != 'p' && c != 27){
      c = cvWaitKey(250);
      }    
    }
    
    if(c == 27)
    return 0;
    image = cvQueryFrame( capture ); //Get next image
  
  } //END COLLECTION WHILE LOOP.
  
  //ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
  CvMat* object_points2 = cvCreateMat(successes*board_n,3,CV_32FC1);
  CvMat* image_points2 = cvCreateMat(successes*board_n,2,CV_32FC1);
  CvMat* point_counts2 = cvCreateMat(successes,1,CV_32SC1);
  
  //TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
  //Below, we write out the details in the next two loops. We could
  //instead have written:
  //image_points->rows = object_points->rows = \
  //successes*board_n; point_counts->rows = successes;
  //
  
  for(int i = 0; i<successes*board_n; ++i) {
    CV_MAT_ELEM( *image_points2, float, i, 0) =
    CV_MAT_ELEM( *image_points, float, i, 0);
    CV_MAT_ELEM( *image_points2, float,i,1) =
    CV_MAT_ELEM( *image_points, float, i, 1);
    CV_MAT_ELEM(*object_points2, float, i, 0) =
    CV_MAT_ELEM( *object_points, float, i, 0) ;
    CV_MAT_ELEM( *object_points2, float, i, 1) =
    CV_MAT_ELEM( *object_points, float, i, 1) ;
    CV_MAT_ELEM( *object_points2, float, i, 2) =
    CV_MAT_ELEM( *object_points, float, i, 2) ;
  }
  
  for(int i=0; i<successes; ++i){ //These are all the same number
    CV_MAT_ELEM( *point_counts2, int, i, 0) =
    CV_MAT_ELEM( *point_counts, int, i, 0);
  }
  
  cvReleaseMat(&object_points);
  cvReleaseMat(&image_points);
  cvReleaseMat(&point_counts);
  // At this point we have all of the chessboard corners we need.
  // Initialize the intrinsic matrix such that the two focal
  // lengths have a ratio of 1.0
  //
  CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
  CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;
  
  //CALIBRATE THE CAMERA!
  cvCalibrateCamera2(
    object_points2, image_points2,
    point_counts2, cvGetSize( image ),
    intrinsic_matrix, distortion_coeffs,
    NULL, NULL,0 //CV_CALIB_FIX_ASPECT_RATIO
  );
  
  // SAVE THE INTRINSICS AND DISTORTIONS
  cvSave("Intrinsics.xml",intrinsic_matrix, 0, 0, cvAttrList(NULL, NULL));
  cvSave("Distortion.xml",distortion_coeffs, 0, 0, cvAttrList(NULL, NULL));
  
  // EXAMPLE OF LOADING THESE MATRICES BACK IN:
  CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml", NULL, NULL, NULL);
  CvMat *distortion = (CvMat*)cvLoad("Distortion.xml", NULL, NULL, NULL);
  // Build the undistort map that we will use for all
  // subsequent frames.
  //
  IplImage* mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
  IplImage* mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
  cvInitUndistortMap(
    intrinsic,
    distortion,
    mapx,
    mapy
  );
  // Just run the camera to the screen, now showing the raw and
  // the undistorted image.
  //
  cvNamedWindow( "Undistort" , 0);
  while(image) {
    IplImage *t = cvCloneImage(image);
    cvShowImage( "Calibration", image ); // Show raw image
    cvRemap( t, image, mapx, mapy , 0, cvScalar(0,0,0,0)); // Undistort image
    cvReleaseImage(&t);
    cvShowImage("Undistort", image); // Show corrected image
    
    //Handle pause/unpause and ESC
    int c = cvWaitKey(15);
    if(c == 'p') {
      c = 0;
      while(c != 'p' && c != 27) {
	c = cvWaitKey(250);
      }
    }
    if(c == 27)
    break;
    image = cvQueryFrame( capture );
  }
  return 0;
}
