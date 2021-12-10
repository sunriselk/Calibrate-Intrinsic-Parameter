#include <iostream>
#include <opencv.hpp>
#include "CalibCircleDetection.h"
#include <vector>
#include <string>
#include <sstream>
const char strImagePath[] = "../Test";
const std::string strResultPath = "../Test/Camera000.txt";

int main()
{
	HalconCpp::HObject  ho_Image;
	HalconCpp::HTuple  hv_ImageFiles, hv_Index;
	ListFiles(strImagePath, (HalconCpp::HTuple("files").Append("follow_links")),
		&hv_ImageFiles);
	TupleRegexpSelect(hv_ImageFiles, (HalconCpp::HTuple("\\.(tif|tiff|gif|bmp|jpg|jpeg|jp2|png|pcx|pgm|ppm|pbm|xwd|ima|hobj)$").Append("ignore_case")),
		&hv_ImageFiles);
	HalconCpp::HTuple end_val3 = (hv_ImageFiles.TupleLength()) - 1;
	HalconCpp::HTuple step_val3 = 1;

	std::vector<std::vector<cv::Point2f>> vvImgPoints;
	std::vector<std::vector<cv::Point3f>> vvObjPoints;
	HalconCpp::HTuple nImgWidth, nImgHeight;
	for (hv_Index = 0; hv_Index.Continue(end_val3, step_val3); hv_Index += step_val3)
	{
		ReadImage(&ho_Image, HalconCpp::HTuple(hv_ImageFiles[hv_Index]));
		
		if (hv_Index == 0)
			HalconCpp::GetImageSize(ho_Image, &nImgWidth, &nImgHeight);

		//处理图像
		CalibCircleDetection DetectEllipse;
		std::vector<std::pair<int, int>> bigEllipseSortedIndex, outSortIndex;

		bigEllipseSortedIndex.push_back(std::pair<int, int>(8, 5));
		bigEllipseSortedIndex.push_back(std::pair<int, int>(8, 9));
		bigEllipseSortedIndex.push_back(std::pair<int, int>(7, 9));
		bigEllipseSortedIndex.push_back(std::pair<int, int>(5, 7));
		bigEllipseSortedIndex.push_back(std::pair<int, int>(11, 7));

		std::vector<cv::Point2f> sortImageEllipse;
		DetectEllipse.DectectTargetEllipses(ho_Image, 40, 40, 17, 15, bigEllipseSortedIndex, sortImageEllipse, outSortIndex);

		if (0 == sortImageEllipse.size())
			continue;

		/*测试开始*/
		cv::Mat cvSrcImg = cv::imread(std::string(hv_ImageFiles[hv_Index]));
		for (uint i = 0; i < sortImageEllipse.size(); i++)
		{
			cv::Scalar color_red = cv::Scalar(0, 0, 255);
			cv::line(cvSrcImg, cv::Point2i(int(sortImageEllipse[i].x) - 5, int(sortImageEllipse[i].y)),
				cv::Point2i(int(sortImageEllipse[i].x) + 5, int(sortImageEllipse[i].y)), color_red);
			cv::line(cvSrcImg, cv::Point2i(int(sortImageEllipse[i].x), int(sortImageEllipse[i].y) - 5),
				cv::Point2i(int(sortImageEllipse[i].x), int(sortImageEllipse[i].y) + 5), color_red);
			std::stringstream ss;
			ss << "(" << outSortIndex[i].first << "," << outSortIndex[i].second << ")";
			std::string Index;
			ss >> Index;
			cv::putText(cvSrcImg, Index, cv::Point2i(int(sortImageEllipse[i].x) - 30, int(sortImageEllipse[i].y) - 10),
				cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 0, 255));
		}
		/*测试结束*/

		//准备标定板物体坐标
		double dPatternlenth = 33;		//设置标定板相邻特征点之间的距离
		std::vector<cv::Point3f> vecObjectPoints;
		for (uint i = 0; i < sortImageEllipse.size(); i++)
		{
			cv::Point3f temp_object_point;
			temp_object_point.x = float(dPatternlenth) * outSortIndex[i].first;
			temp_object_point.y = float(dPatternlenth) * outSortIndex[i].second;
			temp_object_point.z = 0.0;
			vecObjectPoints.push_back(temp_object_point);
		}
		vvImgPoints.push_back(sortImageEllipse);
		vvObjPoints.push_back(vecObjectPoints);
	}

	if (vvImgPoints.size() < 5)
		exit(0);
	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
	cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
	std::vector<cv::Mat> rvecsMat;
	std::vector<cv::Mat> tvecsMat;

	
	double error = calibrateCamera(vvObjPoints, vvImgPoints, cv::Size(nImgWidth,nImgHeight),
		cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);

	//保存标定结果
	std::fstream calibration_data;
	calibration_data.open(strResultPath, std::ofstream::out);

	std::cout << "开始保存定标结果…" << std::endl;

	calibration_data << cameraMatrix.ptr<double>(0)[0] << std::endl	//fu
		<< cameraMatrix.ptr<double>(1)[1] << std::endl				//fv
		<< cameraMatrix.ptr<double>(0)[2] << std::endl				//u0
		<< cameraMatrix.ptr<double>(1)[2] << std::endl				//v0
		<< distCoeffs.ptr<double>(0)[0] << std::endl				//k1
		<< distCoeffs.ptr<double>(0)[1] << std::endl				//k2
		<< distCoeffs.ptr<double>(0)[4] << std::endl				//k3
		<< distCoeffs.ptr<double>(0)[2] << std::endl				//p1
		<< distCoeffs.ptr<double>(0)[3] << std::endl;				//p2


	//cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
	//calibration_data << "相机内参矩阵: " << std::endl;
	//calibration_data << cameraMatrix << std::endl << std::endl;
	//calibration_data << "相机畸变系数: " << std::endl;
	//calibration_data << distCoeffs << std::endl << std::endl << std::endl;
	//for (int i = 0; i < vvImgPoints.size(); i++)
	//{
	//	calibration_data << "第" << i + 1 << "图像旋转矢量：" << std::endl;
	//	calibration_data << rvecsMat[i] << std::endl << std::endl;

	//	/* 将旋转向量转换为相对应的旋转矩阵 */
	//	Rodrigues(rvecsMat[i], rotation_matrix);
	//	calibration_data << "第" << i + 1 << "图像旋转矩阵：" << std::endl;
	//	calibration_data << rotation_matrix << std::endl << std::endl;
	//	calibration_data << "第" << i + 1 << "图像平移矢量：" << std::endl;
	//	calibration_data << tvecsMat[i] << std::endl << std::endl << std::endl;
	//}
	std::cout << "定标结果完成保存！" << std::endl;
	calibration_data << std::endl;
	
	return 0;
}