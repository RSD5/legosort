#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <string.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <time.h>  
#include <stdio.h>

using namespace cv;
using namespace std;

//////////////////////// FUNCTIONALITY SETUP //////////////////////

//Define Brick Size
#define BLUE_AREAL_MAX		2200
#define BLUE_AREAL_MIN  	800
#define RED_AREAL_MAX		9000
#define RED_AREAL_MIN  		2000
#define YELLOW_AREAL_MAX	8000
#define YELLOW_AREAL_MIN  	5000

//Define Detecting Area
#define AREA_X_MIN		70
#define AREA_X_MAX		560
#define AREA_Y_MIN		50
#define AREA_Y_MAX		400

// HUE
#define H_Blue_MIN		41
#define H_Blue_MAX		256

#define H_Red_MIN		0
#define H_Red_MAX		12

#define	H_Yellow_MIN		12
#define H_Yellow_MAX		256

// SATURATION
#define S_Blue_MIN		2
#define S_Blue_MAX		256

#define S_Red_MIN		177
#define S_Red_MAX		256

#define S_Yellow_MIN		146
#define S_Yellow_MAX		256

// VALUE
#define V_Blue_MIN		70
#define V_Blue_MAX		256

#define V_Red_MIN		156
#define V_Red_MAX		256

#define V_Yellow_MIN		254
#define V_Yellow_MAX		256


///////////////////////// END OF SETUP ///////////////////////////////

Mat HSV_Filter(IplImage* img, char Color, int FilterSize)
{
	// Convert the image into an HSV image
	IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);		
	cvCvtColor(img, imgHSV, CV_BGR2HSV);
	
	// Covertion to Mat
	Mat matBlured(imgHSV); 
	
	//Blure image with a gausianfilter of size FilterSize
	GaussianBlur(matBlured, matBlured, Size(FilterSize, FilterSize), 2.0, 2.0);
	
	//Conversion (mat to IplImage)
	IplImage objHSVBlur = matBlured;
	IplImage* imgHSVBlur = &objHSVBlur;
	
	//Output image
	IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);

	//Apply HSV filtering
	switch (Color)
	{
		case 'R':
			cvInRangeS(imgHSVBlur, cvScalar(H_Red_MIN, S_Red_MIN, V_Red_MIN), cvScalar(H_Red_MAX, S_Red_MAX, V_Red_MAX), imgThreshed);
		break;
		
		case 'Y':
			cvInRangeS(imgHSVBlur, cvScalar(H_Yellow_MIN, S_Yellow_MIN, V_Yellow_MIN), cvScalar(H_Yellow_MAX, S_Yellow_MAX, V_Yellow_MAX), imgThreshed);
		break;
		
		case 'B':
			cvInRangeS(imgHSVBlur, cvScalar(H_Blue_MIN, S_Blue_MIN, V_Blue_MIN), cvScalar(H_Blue_MAX, S_Blue_MAX, V_Blue_MAX), imgThreshed);
		break;
		
		default:
			cout << "Error" << endl;
		break;
	}
	
	Mat matThresh(imgThreshed,true); 
	cvReleaseImage(&imgHSV);
	cvReleaseImage(&imgThreshed);
	
	return matThresh;
}

class BrickDetect 
{
	//Variables
	vector<vector<Point> > contours_red;
	vector<vector<Point> > contours_yellow;
	vector<vector<Point> > contours_blue;
	vector<Vec4i> hierarchy;
	IplImage* frame;
	CvCapture* capture;
	
	Mat HSV_Img;
	Mat Red;
	Mat Yellow;
	Mat Blue;
	Mat drawing;
	Mat canny_output;
	int camInit;
	
	public:
		void set_cam (int);
		void start_cam();
		vector<Brick> get_bricks();
		void release_mem();	
};

void BrickDetect::set_cam (int cam)
{
	camInit = cam;
}

void BrickDetect::start_cam()
{
	// Initialize capturing live feed from the camera
	capture = 0;
	capture = cvCaptureFromCAM(camInit);	

	// Couldn't get a device? Throw an error and quit
	if(!capture)
    {
		//error
    }
}

void BrickDetect::release_mem()
{
	cvReleaseCapture(&capture);
}

vector<Brick> BrickDetect::get_bricks()
{	
	// Will hold a frame captured from the camera
		frame = cvQueryFrame(capture);
		//frame = cvQueryFrame(capture);
		// If we couldn't grab a frame... quit
        if(!frame)
		{
			std::cout << "NO FRAME!!!!!" << std::endl;
		}
		
		vector<Brick> bricks_to_return;
		
		//Get HSV filtering images
		Red = HSV_Filter(frame, 'R', 7);
		Yellow = HSV_Filter(frame, 'Y', 7);
		Blue = HSV_Filter(frame, 'B', 7);
	
		//Getting contours for R, Y, B
		findContours( Red, contours_red, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		findContours( Yellow, contours_yellow, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		findContours( Blue, contours_blue, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		
		//Creating containers of type vector<RotatedRect> (RotatedRect)
		vector<RotatedRect> minRect_blue( contours_blue.size() );
		vector<RotatedRect> minRect_red( contours_red.size() );
		vector<RotatedRect> minRect_yellow( contours_yellow.size() );
		
		//Blue - Filling container (RotatedRect)
		for( unsigned int i = 0; i < contours_blue.size(); i++ )
		{ 
			minRect_blue[i] = minAreaRect( Mat(contours_blue[i]) );
		}
		
		//Red - Filling container (RotatedRect)
		for( unsigned int i = 0; i < contours_red.size(); i++ )
		{ 
			minRect_red[i] = minAreaRect( Mat(contours_red[i]) );
		}
		
		//Yellow - Filling container (RotatedRect)
		for( unsigned int i = 0; i < contours_yellow.size(); i++ )
		{ 
			minRect_yellow[i] = minAreaRect( Mat(contours_yellow[i]) );
		}
		
		//Creating a Mat container for the output image - Rectangles or bricks
		// rawing = Mat::zeros( Red.size(), CV_8UC3 );
		// rectangle(drawing, Point(AREA_X_MIN, AREA_Y_MIN), Point(AREA_X_MAX, AREA_Y_MAX), Scalar(155,155,155), -1, 8);d
		
		//new line for printing (timestamp)
		// cout << endl;
		
		time_t rawtime;
		struct tm * timeinfo;

		time (&rawtime);
		timeinfo = localtime (&rawtime);
		
		// printf ("Current local time and date: %s \n", asctime(timeinfo));
		
		//Plotting blue bricks
		for( unsigned int i = 0; i< contours_blue.size(); i++ )
		{
			if((minRect_blue[i].center.x < AREA_X_MAX) && (minRect_blue[i].center.x > AREA_X_MIN) && (minRect_blue[i].center.y < AREA_Y_MAX) && (minRect_blue[i].center.y > AREA_Y_MIN))
			{
				//Getting areal
				double areal = minRect_blue[i].size.width * minRect_blue[i].size.height;
				
				//Testing areal
				if(areal > BLUE_AREAL_MIN && areal < BLUE_AREAL_MAX)
				{
					// //Defining color to draw with
					// Scalar color = Scalar( 255, 0, 0 );
					
					// //Plot rotated rectangle
					// Point2f rect_points[4]; 
					// minRect_blue[i].points( rect_points );
					// for( int j = 0; j < 4; j++ )
					// line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
					
					//Output to console
					// cout << "Color: Blue" << endl;
					// cout << "Size:	" << minRect_blue[i].size << endl;
					// cout << "Center: " << minRect_blue[i].center << endl;
					// if(minRect_blue[i].size.width > minRect_blue[i].size.height)
					// {
						
					// 	cout << "Orientation: " << minRect_blue[i].angle <<endl << endl;
					// }else{
					// 	cout << "Orientation: " << minRect_blue[i].angle + 90<<endl << endl;
					// }
					
					Brick b;
					b.timestamp = asctime(timeinfo);
					b.color = BLUE;
					b.width = minRect_blue[i].size.width;
					b.height = minRect_blue[i].size.height;
					b.x_coord = minRect_blue[i].center.x;
					b.y_coord = minRect_blue[i].center.y;
					b.angle = (minRect_blue[i].size.width > minRect_blue[i].size.height) ? minRect_blue[i].angle : minRect_blue[i].angle + 90;
					bricks_to_return.push_back(b);
				}
			}
		}
		
		//Plotting red bricks
		for( unsigned int i = 0; i< contours_red.size(); i++ )
		{
			if((minRect_red[i].center.x < AREA_X_MAX) && (minRect_red[i].center.x > AREA_X_MIN) && (minRect_red[i].center.y < AREA_Y_MAX) && (minRect_red[i].center.y > AREA_Y_MIN))
			{
				//Getting areal
				double areal = minRect_red[i].size.width * minRect_red[i].size.height;
				
				//Testing areal
				if(areal > RED_AREAL_MIN && areal < RED_AREAL_MAX)
				{
					//Defining color to draw with
					// Scalar color = Scalar( 0, 0, 255 );
					
					// //Plot rotated rectangle
					// Point2f rect_points[4]; 
					// minRect_red[i].points( rect_points );
					// for( int j = 0; j < 4; j++ )
					// line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
					
					//Output to console
					// cout << "Color: Red" << endl;
					// cout << "Size:	" << minRect_red[i].size << endl;
					// cout << "Center: " << minRect_red[i].center << endl;
					// cout << "Orientation: " << minRect_red[i].angle <<endl << endl;

					Brick b;
					b.timestamp = asctime(timeinfo);
					b.color = RED;
					b.width = minRect_red[i].size.width;
					b.height = minRect_red[i].size.height;
					b.x_coord = minRect_red[i].center.x;
					b.y_coord = minRect_red[i].center.y;
					b.angle = (minRect_red[i].size.width > minRect_red[i].size.height) ? minRect_red[i].angle : minRect_red[i].angle + 90;
					bricks_to_return.push_back(b);
				}
			}
		}
		
		//Plotting yellow bricks
		for( unsigned int i = 0; i< contours_yellow.size(); i++ )
		{
			if((minRect_yellow[i].center.x < AREA_X_MAX) && (minRect_yellow[i].center.x > AREA_X_MIN) && (minRect_yellow[i].center.y < AREA_Y_MAX) && (minRect_yellow[i].center.y > AREA_Y_MIN))
			{
				//Getting areal
				double areal = minRect_yellow[i].size.width * minRect_yellow[i].size.height;
				
				//Testing areal
				if(areal > YELLOW_AREAL_MIN && areal < YELLOW_AREAL_MAX)
				{
					//Defining color to draw with
					// Scalar color = Scalar( 0, 255, 255 );
					
					//Plot rotated rectangle
					// Point2f rect_points[4]; 
					// minRect_yellow[i].points( rect_points );
					// for( int j = 0; j < 4; j++ )
					// line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
					
					//Output to console
					// cout << "Color: Yellow" << endl;
					// cout << "Size:	" << minRect_yellow[i].size << endl;
					// cout << "Center: " << minRect_yellow[i].center << endl;
					// cout << "Orientation: " << minRect_yellow[i].angle <<endl << endl;Brick b;

					Brick b;
					b.timestamp = asctime(timeinfo);
					b.color = YELLOW;
					b.width = minRect_yellow[i].size.width;
					b.height = minRect_yellow[i].size.height;
					b.x_coord = minRect_yellow[i].center.x;
					b.y_coord = minRect_yellow[i].center.y;
					b.angle = (minRect_yellow[i].size.width > minRect_yellow[i].size.height) ? minRect_yellow[i].angle : minRect_yellow[i].angle + 90;
					bricks_to_return.push_back(b);
				}
			}
		}
		
		// Show in a window
		//cvShowImage("CAM", frame);
		//namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
		//imshow( "Contours", drawing );
		return bricks_to_return;
}

// int main()
// {
// 	BrickDetect Detector;
// 	Detector.set_cam(0);
// 	Detector.start_cam();
// 	while(1)
// 	{
// 	Detector.get_bricks();	
// 	while(1)
// 	{
		
// 		int c = cvWaitKey(10);
// 		if(c!=-1)
// 		{
// 			// If pressed, break out of the loop
//             break;
// 		}
// 	}
// 	}
// 	Detector.release_mem();
	
// 	return 0;
// }