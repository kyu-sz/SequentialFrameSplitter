// SequentialFrameSplitter.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <iostream>
#include <fstream>
#include <sstream>

#include "optical_flow_judger.h"

using namespace std;
using namespace cv;
using std::cout;

#define OPTICAL_FLOW_THRESHOLD 0.5

string GetName(const string& path);

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
		Mat prev_gray, gray;
		int seq_cnt = 0;
		
		stringstream ss;
		ss << name << '_' << seq_cnt << ".avi";
		VideoWriter writer(ss.str(),
			CV_FOURCC_DEFAULT,
			cap.get(CAP_PROP_FPS),
			Size(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT)));
		while (true) {
			cap >> frame;
			if (frame.empty()) break;
			imshow("Origin", frame);

			double t = (double)cvGetTickCount();

			cvtColor(frame, gray, CV_BGR2GRAY);

			if (!prev_frame.empty()) {
				bool is_consecutive = true;

				is_consecutive = is_consecutive && OpticalFlowJudge(prev_gray, gray, true);

				t = (double)cvGetTickCount() - t;
				cout << "cost time: " << t / ((double)cvGetTickFrequency()*1000.) << endl;

				if (is_consecutive) waitKey(1);
				else
				{
					waitKey(0);

					ss.str(std::string());
					ss << name << '_' << ++seq_cnt << ".avi";
					writer.open(ss.str(),
						CV_FOURCC_DEFAULT,
						cap.get(CAP_PROP_FPS),
						Size(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT)));
				}
			}

			prev_frame = frame;
			prev_gray = gray;
		}

		cout << "Finished processing " << video_path << "! Split " << seq_cnt << " sequences." << endl;
		++success_cnt;
	}

	cout << "Finished processing " << success_cnt << " videos." << endl;
	cout << "Failed to process " << failure_cnt << " videos:" << endl;
	for (auto path : failure_list) cout << "\t" << path << endl;
	system("PAUSE");
	return 0;
}

string GetName(const string& path)
{
	const auto last_slash = path.rfind('/');
	const auto last_back_slash = path.rfind('\\');
	const auto dot_pos = path.rfind('.');
	const auto starting =
		(last_slash == string::npos || (last_back_slash != string::npos && last_back_slash > last_slash)) ?
		last_back_slash : last_slash;
	return path.substr(starting + 1, dot_pos - starting - 1);
}
