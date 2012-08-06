/**
*       @file main.cpp
*       @brief OpenCV Stereo Webcam.
*       @author Martin Peris (http://www.martinperis.com)
*       @date 21/08/2011
*/

/*
        main.cpp - Implement a simple stereo webcam with OpenCV and C++
        Copyright (c) 2011 Martin Peris (http://www.martinperis.com).
        All right reserved.
        
        This application is free software; you can redistribute it and/or
        modify it under the terms of the GNU Lesser General Public
        License as published by the Free Software Foundation; either
        version 2.1 of the License, or (at your option) any later version.
        
        This application is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        Lesser General Public License for more details.
        
        You should have received a copy of the GNU Lesser General Public
        License along with this application; if not, write to the Free Software
        Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


//This works for me on OpenCV 2.0 with 2 Logicool webcams.

#include "cv.h"
#include "highgui.h"
#include <iostream>

//Maybe in OpenCV2.2 the correct include statement would be:
//#include "opencv2/opencv.hpp"


int main(int, char**)
{
    cv::VideoCapture capLeft(0); // open the Left camera
    cv::VideoCapture capRight(1); // open the Right camera
    
    if(!capLeft.isOpened() || !capRight.isOpened())  // check if we succeeded
    {
        std::cerr << "ERROR: Could not open cameras." << std::endl;
        return -1;
    }
    
    
    cv::namedWindow("Left",1);
    cv::namedWindow("Right",1);
    
    for(;;)
    {
        bool isValid = true;
        
        
        cv::Mat frameLeft;
        cv::Mat frameRight;
        
        try
        {
          capLeft >> frameLeft; // get a new frame from left camera
          capRight >> frameRight; //get a new frame from right camera
        }
        catch( cv::Exception& e )
        {
          std::cout << "An exception occurred. Ignoring frame. " << e.err << std::endl;
          isValid = false;
        }
        
        if (isValid)
        {
          try
          {
            cv::imshow("Left", frameLeft);
            cv::imshow("Right", frameRight);
            
            /************************************************************
            *    This is the place for all the cool stuff that you      *
            *    want to do with your stereo images                     *
            ************************************************************/
            
            //TODO:...
            
          }
          catch( cv::Exception& e )
          {
            /************************************************************ 
            *    Sometimes an "Unrecognized or unsuported array type"   * 
            *    exception is received so we handle it to avoid dying   *
            ************************************************************/
            std::cout << "An exception occurred. Ignoring frame. " << e.err << std::endl;
          }
        }
        if(cv::waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
