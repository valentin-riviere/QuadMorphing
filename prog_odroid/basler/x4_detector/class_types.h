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
#ifndef CORNER_H
#define CORNER_H

#include "opencv2/imgproc/imgproc.hpp"
#include <queue>

using namespace cv;
using namespace std;

class Corner
{
	public:
		Corner();
		Corner(const Point & point, const float value);
		Point* getPoint(void);	// get point
	private:
		Point m_point;	// Point
		float m_value;	// Weight of corner

		friend std::ostream& operator<<( std::ostream & stream, Corner const & c);
		friend bool operator<(const Corner & c1, const Corner & c2);
};

// Square
typedef vector<Point> Square;

// Corners sort by strengh value
typedef priority_queue<Corner, vector<Corner>, less<vector<Corner>::value_type> > priority_queue_corners;

#endif
