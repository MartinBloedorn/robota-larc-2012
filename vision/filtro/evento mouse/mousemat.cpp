#include <iostream>
#include <time.h>
#include <cstdio>
#include <cstring>
// Include OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
using namespace cv;
int xx,yy;
int estado=0;
//bool drawing_box = false;
// Implement mouse callback
CvRect box;
Mat image = imread("frame1"".png",3);
Mat temp=image;
bool drawing_box = false;

void draw_box( Mat images, CvRect rect ){
	rectangle( images,
           Point(box.x, box.y),
           Point( box.x+box.width,box.y+box.height),
           Scalar( 0, 255, 255 ),
           0,
           8 );
}
void onMouse( int event, int x, int y, int, void* )
{
		switch( event ){
		case CV_EVENT_MOUSEMOVE:
			if( drawing_box ){
				box.width = x-box.x;
				box.height = y-box.y;
			}
			break;

		case CV_EVENT_LBUTTONDOWN:
			estado=1;
			xx=x;
			yy=y;
			drawing_box = true;
			box = cvRect( x, y, 0, 0 );
			break;

		case CV_EVENT_LBUTTONUP:
			xx=x;
			yy=y;
			estado=2;
			drawing_box = false;
			if( box.width < 0 ){
				box.x += box.width;
				box.width *= -1;
			}
			if( box.height < 0 ){
				box.y += box.height;
				box.height *= -1;
			}
			draw_box(image, box );
			break;
	}
}
    
int main(int argc,char* argv[])
{
	const char* name = "Box Example";
	// Set up the callback
	setMouseCallback( name,onMouse, 0);
	// Main loop
	while("1")
	{
		temp=image.clone();
		if( drawing_box )   draw_box( temp, box );
		setMouseCallback( name,onMouse, 0);
		if(estado!=3)	printf("%d,%d-%d\n",xx,yy,estado);
		if( cvWaitKey( 15 )==27 )	break;
		imshow(name,temp);
	}
	return 0;
}
