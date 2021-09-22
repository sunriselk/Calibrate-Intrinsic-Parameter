#pragma once

#include <iostream>
#include "halconcpp/HalconCpp.h"
#include <opencv.hpp>
//#include "GlobalBundleAjustment_global.h"
class CalibCircleDetection
{
public:
	CalibCircleDetection();
	~CalibCircleDetection();
	float computeLengthOfTwoPoint(cv::Point2f p1, cv::Point2f p2);
	int DectectTargetEllipses(HalconCpp::HObject &H_img, const double &trueDistanceX, const double &trueDistanceY, 
		const int &XellipseNum, const int &YellipseNum, std::vector<std::pair<int, int>>sortIndex, std::vector<cv::Point2f> &ImageEllipseSorted, 
		std::vector<std::pair<int, int>>&outSortIndex);
	int GetEllipseHalcon(const HalconCpp::HObject &dst, std::vector<cv::Point2f> &PointsEllipse, std::vector<cv::Point2f> &Point5Ellipse);
	void point2f2Halcon(std::vector<cv::Point2f> & PointsEllipse, HalconCpp::HTuple &RowPointEllipse, HalconCpp::HTuple &ColPointEllipse);
};

