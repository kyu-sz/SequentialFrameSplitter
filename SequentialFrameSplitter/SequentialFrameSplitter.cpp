// SequentialFrameSplitter.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <iostream>
#include <fstream>

#include "utils.h"

using namespace std;
using namespace cv;
using std::cout;

string GetName(string path);

int main(int argc, char *argv[]) {
	if (argc < 1) {
		cout << "Usage: SequentialFrameCutter video_list" << endl;
		return 0;
	}
	cout << "Reading video paths from list: " << argv[1] << endl;

	string video_path;
	ifstream video_list(argv[1]);

	int success_cnt = 0, failure_cnt = 0;
	vector<string> failure_list;
	while (video_list) {
		if (!getline(video_list, video_path)) break;
		cout << "Processing video " << video_path << endl;
		const auto name = GetName(video_path);

		VideoCapture cap(video_path);
		if (!cap.isOpened()) {
			cerr << "Cannot open video " << video_path << "!" << endl;
			failure_list.push_back(video_path);
			++failure_cnt;
			continue;
		}

		Mat frame, prev_frame;
		Mat prev_gray, gray, flow, flow_vis;
		namedWindow(name + " optical flow", 1);
		while (true) {
			cap >> frame;
			if (frame.empty()) break;
			resize(frame, frame, Size(frame.cols / 8, frame.rows / 8));
			imshow(name + " origin", frame);

			double t = (double)cvGetTickCount();

			cvtColor(frame, gray, CV_BGR2GRAY);

			if (!prev_frame.empty()) {
				calcOpticalFlowFarneback(prev_gray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

				t = (double)cvGetTickCount() - t;
				cout << "cost time: " << t / ((double)cvGetTickFrequency()*1000.) << endl;

				MotionToColor(flow, flow_vis);
				imshow(name + " optical flow", flow_vis);
				waitKey(1);
			}

			std::swap(prev_frame, frame);
			std::swap(prev_gray, gray);
		}

		++success_cnt;
	}

	cout << "Finished processing " << success_cnt << " videos." << endl;
	cout << "Failed to process " << failure_cnt << " videos:" << endl;
	for (auto path : failure_list) cout << "\t" << path << endl;
	system("PAUSE");
	return 0;
}

string GetName(string path)
{
	const auto last_slash = path.rfind('/');
	const auto last_back_slash = path.rfind('\\');
	const auto starting =
		(last_slash == string::npos || (last_back_slash != string::npos && last_back_slash > last_slash)) ?
		last_back_slash : last_slash;
	return path.substr(starting + 1);
}
