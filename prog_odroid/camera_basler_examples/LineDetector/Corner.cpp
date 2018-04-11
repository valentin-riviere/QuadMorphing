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
#include "Corner.h"
#include <iostream>

using namespace std;

Corner::Corner()
{
	m_point = Point(0,0);
	m_value = 0;
}

Corner::Corner(const Point & point, const float value)
{
	m_point = point;
	m_value = value;
}

Point* Corner::getPoint()
{
	return &m_point;
}

ostream& operator<<( ostream &stream, Corner const & c)
{
	return stream << "(" << c.m_point.x << ";" << c.m_point.y << ") : " <<  c.m_value;
}

bool operator<(const Corner & c1, const Corner & c2)
{
	return c1.m_value < c2.m_value;
}
