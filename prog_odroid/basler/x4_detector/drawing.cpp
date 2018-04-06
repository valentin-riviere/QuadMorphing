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
#include "drawing.h"

/**
 * @function draw_squares
 */
void draw_squares(const Mat & src, const vector<Square> & squares, Mat & dst, const Scalar & color)
{
	cvtColor(src,dst,COLOR_GRAY2BGR);

	for( size_t i = 0; i < squares.size(); i++ )
	{
		for (unsigned j = 0 ; j < 4 ; j++)
		{
			Point p1(squares[i][j]), p2(squares[i][(j+1)%4]);
			line(dst,p1,p2,color,2);
		}
	}
}
void draw_squares(const Mat & src, const vector<Square> & squares, Mat & dst, const Rect & roi, const Scalar & color)
{
	cvtColor(src,dst,COLOR_GRAY2BGR);
	
	for( size_t i = 0; i < squares.size(); i++ )
	{
		for (unsigned j = 0 ; j < 4 ; j++)
		{
			Point p1(squares[i][j]), p2(squares[i][(j+1)%4]);
			p1.x += roi.x; p1.y += roi.y;
			p2.x += roi.x; p2.y += roi.y;
			line(dst,p1,p2,color,2);
		}
	}
}
void draw_squares(const Mat & src, const Square & square, Mat & dst, const Scalar & color)
{
	cvtColor(src,dst,COLOR_GRAY2BGR);
	
	for (unsigned j = 0 ; j < 4 ; j++)
	{
		Point p1(square[j]), p2(square[(j+1)%4]);
		line(dst,p1,p2,color,2);
	}
}
void draw_squares(const Mat & src, const Square & square, Mat & dst, const Rect & roi, const Scalar & color)
{
	cvtColor(src,dst,COLOR_GRAY2BGR);
	
	for (unsigned j = 0 ; j < 4 ; j++)
	{
		Point p1(square[j]), p2(square[(j+1)%4]);
		p1.x += roi.x; p1.y += roi.y;
		p2.x += roi.x; p2.y += roi.y;
		line(dst,p1,p2,color,2);
	}
	
	// Draw ROI
	Point p1(roi.x,roi.y), p2(roi.x+roi.width-1,roi.y), p3(roi.x+roi.width-1,roi.y+roi.height-1), p4(roi.x,roi.y+roi.height-1);
	line(dst,p1,p2,Scalar(255,0,0),1);
	line(dst,p2,p3,Scalar(255,0,0),1);
	line(dst,p3,p4,Scalar(255,0,0),1);
	line(dst,p4,p1,Scalar(255,0,0),1);
}

/**
 * @function print_squares
 */
void print_squares(const Mat & src, const vector<Square> & squares, const string & win_name, const uint16_t & t_wait, const Scalar & color)
{
	Mat img_print(src.size(),CV_8UC3);

	draw_squares(src, squares, img_print, color);

	namedWindow(win_name, CV_WINDOW_AUTOSIZE );
	imshow(win_name,img_print); waitKey(t_wait);
}
void print_squares(const Mat & src, const Square & square, const string & win_name, const uint16_t & t_wait, const Scalar & color)
{
	Mat img_print(src.size(),CV_8UC3);

	draw_squares(src, square, img_print, color);

	namedWindow(win_name, CV_WINDOW_AUTOSIZE );
	imshow(win_name,img_print); waitKey(t_wait);
}

/**
 * @function print_img
 */
void print_img(const Mat & src, const string & win_name, const uint16_t & t_wait)
{
	namedWindow(win_name, CV_WINDOW_AUTOSIZE );
	imshow(win_name,src); waitKey(t_wait);
}
