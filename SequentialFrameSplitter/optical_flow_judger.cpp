#include "stdafx.h"

#include "optical_flow_judger.h"

using namespace std;
using namespace cv;

#define UNKNOWN_FLOW_THRESH 1e9  

bool OpticalFlowJudge(const Mat& prev_gray, const Mat& gray, bool vis)
{
	// Down sampling to reduce noise.
	Mat gray_down, prev_gray_down;
	pyrDown(prev_gray, prev_gray_down);
	pyrDown(gray, gray_down);

	Mat flow, flow_vis;
	calcOpticalFlowFarneback(prev_gray_down, gray_down, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

	// Split the optical flow map into x and y components.
	Mat flow_components[2];
	split(flow, flow_components);
	// Down sampling to reduce noise.
	pyrDown(flow_components[0], flow_components[0]);
	pyrDown(flow_components[1], flow_components[1]);
	// TODO: Judge whether the neighboring two frames are consecutive according to the turbulence of the optical flow map.

	if (vis) {
		MotionToColor(flow, flow_vis);
		imshow("Optical flow", flow_vis);
	}

	return true;
}

// Color encoding of flow vectors from:  
// http://members.shaw.ca/quadibloc/other/colint.htm  
// This code is modified from:  
// http://vision.middlebury.edu/flow/data/  
void MakeColorWheel(vector<Scalar> &color_wheel)
{
	int RY = 15;
	int YG = 6;
	int GC = 4;
	int CB = 11;
	int BM = 13;
	int MR = 6;

	int i;

	for (i = 0; i < RY; i++) color_wheel.push_back(Scalar(255, 255 * i / RY, 0));
	for (i = 0; i < YG; i++) color_wheel.push_back(Scalar(255 - 255 * i / YG, 255, 0));
	for (i = 0; i < GC; i++) color_wheel.push_back(Scalar(0, 255, 255 * i / GC));
	for (i = 0; i < CB; i++) color_wheel.push_back(Scalar(0, 255 - 255 * i / CB, 255));
	for (i = 0; i < BM; i++) color_wheel.push_back(Scalar(255 * i / BM, 0, 255));
	for (i = 0; i < MR; i++) color_wheel.push_back(Scalar(255, 0, 255 - 255 * i / MR));
}

// This code is modified from
// http://blog.csdn.net/zouxy09/article/details/8683859
void MotionToColor(Mat flow, Mat &color)
{
	if (color.empty())
		color.create(flow.rows, flow.cols, CV_8UC3);

	static vector<Scalar> color_wheel; //Scalar r,g,b  
	if (color_wheel.empty())
		MakeColorWheel(color_wheel);

	// determine motion range:  
	float max_rad = -1;

	// Find max flow to normalize fx and fy  
	for (int i = 0; i < flow.rows; ++i)
	{
		for (int j = 0; j < flow.cols; ++j)
		{
			Vec2f flow_at_point = flow.at<Vec2f>(i, j);
			float fx = flow_at_point[0];
			float fy = flow_at_point[1];
			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
				continue;
			float rad = sqrt(fx * fx + fy * fy);
			max_rad = max_rad > rad ? max_rad : rad;
		}
	}

	for (int i = 0; i < flow.rows; ++i)
	{
		for (int j = 0; j < flow.cols; ++j)
		{
			uchar *data = color.data + color.step[0] * i + color.step[1] * j;
			Vec2f flow_at_point = flow.at<Vec2f>(i, j);

			float fx = (max_rad == 0) ? 0 : flow_at_point[0] / max_rad;
			float fy = (max_rad == 0) ? 0 : flow_at_point[1] / max_rad;
			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
			{
				data[0] = data[1] = data[2] = 0;
				continue;
			}
			float rad = sqrt(fx * fx + fy * fy);

			float angle = atan2(-fy, -fx) / CV_PI;
			float fk = (angle + 1.0) / 2.0 * (color_wheel.size() - 1);
			int k0 = (int)fk;
			int k1 = (k0 + 1) % color_wheel.size();
			float f = fk - k0;
			//f = 0; // uncomment to see original color wheel  

			for (int b = 0; b < 3; b++)
			{
				float col0 = color_wheel[k0][b] / 255.0;
				float col1 = color_wheel[k1][b] / 255.0;
				float col = (1 - f) * col0 + f * col1;
				if (rad <= 1)
					col = 1 - rad * (1 - col); // increase saturation with radius  
				else
					col *= .75; // out of range  
				data[2 - b] = (int)(255.0 * col);
			}
		}
	}
}