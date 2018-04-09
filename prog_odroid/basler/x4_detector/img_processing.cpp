/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "img_processing.h"

/**
 * @function opencv_dist_matrix_from_file
 */
void opencv_dist_matrix_from_file(Mat & cameraMatrix, Mat & distCoeffs, const string path)
{
	FileStorage fs(path,FileStorage::READ);

	fs["camera_matrix"] >> cameraMatrix;
	fs["distortion_coefficients"] >> distCoeffs;

	fs.release();
}

/**
 * @function FLDetector
 */
void FLDetector(const Mat & src, vector<Vec4f> & lines, const int length_thresh_fl, const float dist_thresh_fl, const double canny_th1_fl, const double canny_th2_fl, const int canny_aper_size_fl, const bool do_merge_fl)
{
	using namespace cv::ximgproc;

	// Create and LSD detector with standard parameters for refinement
	Ptr<FastLineDetector> fld = createFastLineDetector(length_thresh_fl,dist_thresh_fl,canny_th1_fl,canny_th2_fl,canny_aper_size_fl,do_merge_fl);

	// Detect lines
	lines.clear();
	fld->detect(src,lines);
}

/**
 * @function HarrisDetector
 */
void HarrisDetector(const Mat & src, priority_queue_corners & corners, const int thresh_harris, const int blockSize_harris, const int kSize_harris, const double k_harris, const BorderTypes borderType_harris)
{
	Mat dst(src.size(),CV_32FC1);

	// Detect corners
	cornerHarris(src, dst, blockSize_harris, kSize_harris, k_harris, borderType_harris );

	// Normalizing
  	normalize( dst, dst, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );

	// Threshold
	for( int j = 0; j < dst.rows ; j++ )
	{
		for( int i = 0; i < dst.cols; i++ )
		{
			if( (int) dst.at<float>(j,i) > thresh_harris )
			{
				Corner c(Point(i,j),dst.at<float>(j,i));
				corners.push(c);
			}
		}
	}
}

/**
 * @function DetectRectangle
 */
void RectangleDetector(const vector<Vec4f> & lines_src, vector<Vec4f> & lines_dst, const float angle_offset, const uint16_t r_square_detect, const float thresh_angle)
{
	// Clear results
	lines_dst.clear();

	// Keep horizontal/vertical lines
	vector<Vec4f> lines_hor; 	// Horizontal lines results
	vector<Vec4f> lines_vert; 	// Vertical lines results
	for (unsigned int i = 0 ; i < lines_src.size() ; i++)
	{
		float angle_line = CV_PI + atan2(lines_src[i][3]-lines_src[i][1],lines_src[i][2]-lines_src[i][0]);
		if (abs(angle_offset - fmod(angle_line,CV_PI)) <= thresh_angle)
		{
			lines_hor.push_back(lines_src[i]);
		}
		else if (abs(CV_PI/2.0 + angle_offset - fmod(angle_line,CV_PI)) <= thresh_angle)
		{
			lines_vert.push_back(lines_src[i]);
		}
	}

	// Keep connected vertical/horizontal lines
	for (unsigned int i = 0 ; i < lines_hor.size() ; i++)
	{
		for (unsigned int j = 0 ; j < lines_vert.size() ; j++)
		{
			if ( ((lines_hor[i][0]-lines_vert[j][0])*(lines_hor[i][0]-lines_vert[j][0]) + (lines_hor[i][1]-lines_vert[j][1])*(lines_hor[i][1]-lines_vert[j][1]) <= r_square_detect) || ((lines_hor[i][0]-lines_vert[j][2])*(lines_hor[i][0]-lines_vert[j][2]) + (lines_hor[i][1]-lines_vert[j][3])*(lines_hor[i][1]-lines_vert[j][3]) <= r_square_detect) )
			{
				lines_dst.push_back(lines_hor[i]);
				lines_dst.push_back(lines_vert[j]);
			}
		}
	}
}

/**
 * @function SquaresDetector
 */
void SquaresDetector(const Mat & src, vector<Square> & squares, const int thresh_bin_square, const float k_approx_square, const float thresh_area_square, const float thresh_cos_square)
{
	Mat gray(src.size(), CV_8UC1);
	vector<vector<Point> > contours;

	// Clear detected squares
	squares.clear();

#ifdef DEBUG_RECT_DETECT
	static int thresh_bin_debug = 0;
	createTrackbar("Trackbar rect : ", "Digitized img", &thresh_bin_debug, 255);
	setTrackbarMin("Trackbar rect : ", "Digitized img", 0);
	gray = src >= thresh_bin_debug;
#else
	// Threshold for binary image, could be remplaced by CANNY
	gray = src >= thresh_bin_square;
#endif

#ifdef PRINT_DIG_IMG
	namedWindow("Digitized img", CV_WINDOW_AUTOSIZE );
	imshow("Digitized img",gray);
#endif

 	// find contours and store them all as a list
    findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE); 

    vector<Point> approx;

    // test each contour
    for( size_t i = 0; i < contours.size(); i++ )
    {
        // approximate contour with accuracy proportional
        // to the contour perimeter (Ramer Douglas Peucker algorithm)
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*k_approx_square, true);

        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if( approx.size() == 4 &&
            fabs(contourArea(Mat(approx))) > thresh_area_square &&
            isContourConvex(Mat(approx)) )
        {
            double maxCosine = 0;

            for( int j = 2; j < 5; j++ )
            {
                // find the maximum cosine of the angle between joint edges
                double cosine = fabs(cst_angle(approx[j%4], approx[j-2], approx[j-1]));
                maxCosine = MAX(maxCosine, cosine);
            }

            // if cosines of all angles are small
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
            if( maxCosine < thresh_cos_square)
			{
                squares.push_back(approx);
			}
        }
    }

#ifdef DEBUG_RECT_DETECT
	cout << "Contours detect : " << contours.size() << endl;
	cout << "Squares detect : " << squares.size() << endl;
#endif
}
