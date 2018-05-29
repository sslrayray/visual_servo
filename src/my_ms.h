#ifndef MEANSHIFT_H
#define MEANSHIFT_H

#include <iostream>
#include <math.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

class MeanShift
{
private:
	
	struct config
	{
		/**
		 * Max iteration number to calculate the converged center in meanshift
		 * algorithm
		 */
		int max_iter;

		/**
		 * each color range is [0~255], 256 by default
		 */
		int color_range;

		/**
		 * Level number of each color channel
		 */

		int bin_num;

		/**
		 * color range / bin num, the width of each bin
		 */
		float bin_width;
	  
	}cfg;

	/**
	 * The rect variable of the target region
	 */
	Rect _current_rect;

	/**
	 * kernel function matrix, each element represent the weight, 
	 */
	Mat weight_kernel;
	/**
	 * The PDF value of the target region
	 */
	Mat target_pdf;

public:
	/**
	 * @brief      Constructs the object, initialize bin number
	 */
	MeanShift();

	/**
	 * @brief      compute the kernel(weight matrix) of the region, the center
	 *             point(w/2, h/2) has the highest weight, the sum of all weight
	 *             is supposed to be 1 ,
	 *
	 * @param[in]  rect  The rectangle
	 *
	 * @return     { description_of_the_return_value }
	 */
	Mat Epanechnikov_kernel(int height, int width);

	/**
	 * @brief      Only called in initialization. Calculate initial Probability
	 *             Density Function of the color bins of target region, the
	 *             range of color bin should be [0~ 3*255/bin_width]
	 *
	 * @param[in]  frame  the first image frame of the video
	 * @param[in]  rect   the selected target region in the first frame
	 *
	 * @return     PDF(probability density function) of the selected target
	 *             region
	 */
	Mat calc_target_pdf(const Mat &frame, const Rect &rect); 


	/**
	 * @brief      Called in every iteration during Meanshift algorithm. compute
	 *             pdf of the region
	 *
	 * @param[in]  frame  image
	 * @param[in]  rect   selected region of the image
	 *
	 * @return     The probability density in color range
	 */
	Mat compute_pdf(const Mat &frame, const Rect &rect);
	


	/**
	 * @brief      track the rectangle which is most similar with the initial
	 *             target
	 *
	 * @param[in]  frame  current image frame
	 *
	 * @return     { description_of_the_return_value }
	 */
	Rect track(const Mat& frame);


};

#endif