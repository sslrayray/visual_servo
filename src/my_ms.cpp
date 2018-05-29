#include "my_ms.h"
#include <math.h>

using namespace std;

/**
 * @brief      Constructs the object, initialize bin number
 */
MeanShift::MeanShift()
{
	cfg.max_iter = 30;
	cfg.color_range = 256;
	cfg.bin_num = 16;

	cfg.bin_width = float(cfg.color_range) / cfg.bin_num;
}


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

Mat MeanShift::calc_target_pdf(const Mat &frame, const Rect &rect)
{
	_current_rect = rect;
	/*initalize the kernel mat according to the window size*/
	weight_kernel = Epanechnikov_kernel(rect.height, rect.width);
	target_pdf = compute_pdf(frame, rect);
}



/**
 * @brief      Called at initialization, compute the kernel(weight matrix) of the region, the center
 *             point(w/2, h/2) has the highest weight, the sum of all weight
 *             is supposed to be 1 ,
 *
 * @param[in]  rect  The rectangle
 *
 * @return     { description_of_the_return_value }
 */
Mat MeanShift::Epanechnikov_kernel(int height, int width)
{
	int center_row = height / 2;
	int center_col = width / 2;
	
	Mat kernel(height, width, CV_32F, cv::Scalar(0));
	
	float kernel_sum = 0;

	for (int row = 0; row < height; ++row)
		for (int col = 0; col < width; ++col)
		{
			double x, y, norm;
			x = (double)(col - center_col) / center_col;
			y = (double)(row - center_row) / center_row;
			norm = sqrt(x*x + y*y);
			if (norm > 1)
				kernel.at<float>(row, col) = 0;
			else
				kernel.at<float>(row, col) = 1-norm;

			/*calculate all sum*/
			kernel_sum += kernel.at<float>(row, col);
		}

	/*recompute each pixel's weight value*/
	for (int row = 0; row < height; ++row)
		for (int col = 0; col < width; ++col)
		{
			kernel.at<float>(row, col) /= kernel_sum;
		}

	// cout << kernel << endl;
	return kernel;
}

/**
 * @brief      Called in every iteration during Meanshift algorithm. compute
 *             pdf of the region
 *
 * @param[in]  frame  image
 * @param[in]  rect   selected region of the image
 *
 * @return     The probability density in color range
 */
Mat MeanShift::compute_pdf(const Mat &frame, const Rect &rect)
{
	int region_row_start;
	int region_row_end;
	int region_col_start;
	int region_col_end;

	Mat target_pdf(3, cfg.bin_num, CV_32F, Scalar(1e-10));
	//index of the start point of the region
	region_row_start = rect.y;
	region_col_start = rect.x;

	//index of the end point of the region
	region_row_end = region_row_start + rect.height - 1;
	region_col_end = region_col_start + rect.width - 1;
	
	//compute pdf of the region
	float max = 0;
	for (int row_idx = region_row_start; row_idx <= region_row_end; ++row_idx)
		for (int col_idx = region_col_start; col_idx <= region_col_end; ++col_idx)
		{
			int i = row_idx - region_row_start;
			int j = col_idx - region_col_start;
				
			for (int color = 0; color < 3; color++)
			{
				float curr_pixel_value = frame.at<Vec3b>(row_idx, col_idx)[color];
				
				int target_pdf_idx = curr_pixel_value / cfg.bin_width;
				float weight = weight_kernel.at<float>(i, j);

				target_pdf.at<float>(color, target_pdf_idx) += weight;
			}
		}

	return target_pdf; 
}


Rect MeanShift::track(const Mat& frame)
{
	int region_row_start;
	int region_row_end;
	int region_col_start;
	int region_col_end;

	Mat current_pdf;
	Mat movement_weight(_current_rect.height, _current_rect.width, CV_32F, Scalar(0));

	
	Rect current_rect = _current_rect;

	for (int iter = 0; iter < cfg.max_iter; iter++)
	{


		/**
		 * calculate current pdf of the rect range
		 */
		current_pdf = compute_pdf(frame, current_rect); 
					

		/**
		 * compute the similarity between current p(y) and q(target pdf)
		 */
		region_row_start = current_rect.y;
		region_col_start = current_rect.x;

		//index of the end point of the region
		region_row_end = region_row_start + current_rect.height - 1;
		region_col_end = region_col_start + current_rect.width - 1;
		

		/**
		 * calculate the weight of each pixel 
		 */

		float weight_sum = 0;

		for (int row_idx = region_row_start; row_idx <= region_row_end; ++row_idx)
			for (int col_idx = region_col_start; col_idx <= region_col_end; ++col_idx)
			{
				float x, y;
				int i, j;
				i = row_idx - region_row_start;
				j = col_idx - region_col_start;
				y = float(i - current_rect.height / 2) / (current_rect.height / 2);
				x = float(j - current_rect.width / 2) / (current_rect.width / 2);
				if (sqrt(x*x + y*y) < 1)
				{
					movement_weight.at<float>(i, j) = 0;
					for (int color = 0; color < 3; color++)
					{
						float curr_pixel_value = frame.at<Vec3b>(row_idx, col_idx)[color];
						int target_pdf_idx = curr_pixel_value / cfg.bin_width;				
						movement_weight.at<float>(i, j) += sqrt(target_pdf.at<float>(color, target_pdf_idx)/current_pdf.at<float>(color, target_pdf_idx));
					}
				} else 
					movement_weight.at<float>(i, j) = 0;
				weight_sum += movement_weight.at<float>(i, j);
			}
	
		for (int row_idx = 0; row_idx < current_rect.height; row_idx++)
			for (int col_idx = 0; col_idx < current_rect.width; col_idx++)
				movement_weight.at<float>(row_idx, col_idx) /= weight_sum;
		
		/**
		 * Calculate the estimated center 
		 */
		float dx = 0;
		float dy = 0;

		for (int row_idx = 0; row_idx < current_rect.height; row_idx++)
			for (int col_idx = 0; col_idx < current_rect.width; col_idx++)
			{
				float y = row_idx - (int)current_rect.height / 2;
				float x = col_idx - (int)current_rect.width / 2;

				dy += movement_weight.at<float>(row_idx, col_idx) * y;
				dx += movement_weight.at<float>(row_idx, col_idx) * x;
			}

		
		cout << "dx = " << dx << ", dy = " << dy << endl;
    	// cv::waitKey( 0);		

		

		/**
		 * move the center of current_rect
		 */
		current_rect.y += 3*ceil(dy);
		current_rect.x += 3*ceil(dx);

		/**
		 * norm(dx, dy) < 1 means we have found the most similarity center
		 */
		if (abs(dx) <1  && abs(dy)< 1)
		{
			cout << "iteration end at " << iter << endl;
			break;	
		}
	}

	_current_rect = current_rect;
	return current_rect;
}