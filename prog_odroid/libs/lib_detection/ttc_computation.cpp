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
#include "ttc_computation.h"

/**
 * @function blur_resize
 */
void blur_resize(const Mat & src, Mat & dst)
{
	for (uint16_t i = 0 ; i < dst.cols ; i++)
	{
		for (uint16_t j = 0 ; j < dst.rows ; j++)
		{
			dst.at<float>(j,i) = (
src.at<uint8_t>(5*j,5*i)+src.at<uint8_t>(5*j+1,5*i)+src.at<uint8_t>(5*j+2,5*i)+src.at<uint8_t>(5*j+3,5*i)+src.at<uint8_t>(5*j+4,5*i)
+
src.at<uint8_t>(5*j,5*i+1)+src.at<uint8_t>(5*j+1,5*i+1)+src.at<uint8_t>(5*j+2,5*i+1)+src.at<uint8_t>(5*j+3,5*i+1)+src.at<uint8_t>(5*j+4,5*i+1)
+
src.at<uint8_t>(5*j,5*i+2)+src.at<uint8_t>(5*j+1,5*i+2)+src.at<uint8_t>(5*j+2,5*i+2)+src.at<uint8_t>(5*j+3,5*i+2)+src.at<uint8_t>(5*j+4,5*i+2)
+
src.at<uint8_t>(5*j,5*i+3)+src.at<uint8_t>(5*j+1,5*i+3)+src.at<uint8_t>(5*j+2,5*i+3)+src.at<uint8_t>(5*j+3,5*i+3)+src.at<uint8_t>(5*j+4,5*i+3)
+
src.at<uint8_t>(5*j,5*i+4)+src.at<uint8_t>(5*j+1,5*i+4)+src.at<uint8_t>(5*j+2,5*i+4)+src.at<uint8_t>(5*j+3,5*i+4)+src.at<uint8_t>(5*j+4,5*i+4)
)/25.0;
		}
	}
}


/**
 * @function compute_ttc
 */
float compute_ttc(const Mat & img_prev, const Mat & img_now, const float Ts, const float thresh_Et, const uint16_t n_median)
{
	float tau = 0, tau_fil; // Time to contact (in s)
	float num = 0, den = 0;
	static deque<float> tau_list;

	for (uint16_t i = 0 ; i < img_now.cols-1 ; i++)
	{
		for (uint16_t j = 0 ; j < img_now.rows-1 ; j++)
		{
			float Gx = (float) (1.0/4.0)*(img_prev.at<float>(j,i+1)-img_prev.at<float>(j,i)+img_prev.at<float>(j+1,i+1)-img_prev.at<float>(j+1,i)+img_now.at<float>(j,i+1)-img_now.at<float>(j,i)+img_now.at<float>(j+1,i+1)-img_now.at<float>(j+1,i));
			float Gy = (float) (1.0/4.0)*(img_prev.at<float>(j+1,i)-img_prev.at<float>(j,i)+img_prev.at<float>(j+1,i+1)-img_prev.at<float>(j,i+1)+img_now.at<float>(j+1,i)-img_now.at<float>(j,i)+img_now.at<float>(j+1,i+1)-img_now.at<float>(j,i+1));
			float Et = (float) (1.0/4.0)*(img_now.at<float>(j,i)-img_prev.at<float>(j,i)+img_now.at<float>(j+1,i)-img_prev.at<float>(j+1,i)+img_now.at<float>(j,i+1)-img_prev.at<float>(j,i+1)+img_now.at<float>(j+1,i+1)-img_prev.at<float>(j+1,i+1));

			float grad = (float) (i-(img_now.cols-1)/2.0)*Gx+(j-(img_now.rows-1)/2.0)*Gy;

			if (Et > thresh_Et)
			{
				num -= grad*Et;
				den += Et*Et;
			}
		}
	}

	// time to collision
	tau = num/den*Ts;

	// Store in queue
	if (tau_list.size() > n_median)	// Keep only N_median values
		tau_list.pop_back();
	tau_list.push_front(tau);

	// Perform median filtering
	tau_fil = median(tau_list);

	return tau_fil;
}

