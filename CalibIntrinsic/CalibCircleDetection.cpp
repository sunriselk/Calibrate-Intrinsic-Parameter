#include "CalibCircleDetection.h"
//#include <opencv2/core.hpp>

CalibCircleDetection::CalibCircleDetection()
{
}


CalibCircleDetection::~CalibCircleDetection()
{
}
float CalibCircleDetection::computeLengthOfTwoPoint(cv::Point2f p1, cv::Point2f p2)
{
	return (float)sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y,2));
}
int CalibCircleDetection::DectectTargetEllipses(HalconCpp::HObject &H_img, 
	const double &trueDistanceX, const double &trueDistanceY,
	const int &XellipseNum, const int &YellipseNum,
	std::vector<std::pair<int,int>>sortIndex,
	std::vector<cv::Point2f> &ImageEllipseSorted, 
	std::vector<std::pair<int, int>>&outSortIndex)
{
	using namespace HalconCpp;
	if (!ImageEllipseSorted.empty() || trueDistanceX < 0 || trueDistanceY < 0 ||/* XellipseNum < 11 || YellipseNum < 9 || */sortIndex.size()!=5)
		return -1;

	std::vector<cv::Point2f> PointsEllipse, Point5Ellipse;

	int is2 = GetEllipseHalcon(H_img, PointsEllipse, Point5Ellipse);
	if (is2 != 1)
		return -2;
	int ellipseNum = (int)PointsEllipse.size();
	HTuple imageWidth, imageHeight;
	GetImageSize(H_img, &imageWidth, &imageHeight);

	std::vector<cv::Point2f> Target5Ellipse;

	Target5Ellipse.push_back(cv::Point2f(float(sortIndex[0].first * trueDistanceX), float(sortIndex[0].second * trueDistanceX)));
	Target5Ellipse.push_back(cv::Point2f(float(sortIndex[1].first * trueDistanceX), float(sortIndex[1].second * trueDistanceX)));
	Target5Ellipse.push_back(cv::Point2f(float(sortIndex[2].first * trueDistanceX), float(sortIndex[2].second * trueDistanceX)));
	Target5Ellipse.push_back(cv::Point2f(float(sortIndex[3].first * trueDistanceX), float(sortIndex[3].second * trueDistanceX)));
	Target5Ellipse.push_back(cv::Point2f(float(sortIndex[4].first * trueDistanceX), float(sortIndex[4].second * trueDistanceX)));


	cv::Mat homography = cv::findHomography(Target5Ellipse, Point5Ellipse);////**从标定板映射到图像
	homography.convertTo(homography, CV_32F);
	////**
	cv::Mat objectPoints(3, XellipseNum*YellipseNum, CV_32F, cv::Scalar(1));
	float *pobjectX = objectPoints.ptr<float>(0);
	float *pobjectY = objectPoints.ptr<float>(1);
	for (int i = 0; i < XellipseNum; i++)
	{
		for (int j = 0; j < YellipseNum; j++)
		{
			/*	pobjectX[i*YellipseNum + j] = float(i*trueDistanceX);
				pobjectY[i*YellipseNum + j] = float(j*trueDistanceY);*/
			cv::Mat temp = (cv::Mat_<float>(3, 1)<< i*trueDistanceX, j*trueDistanceY, 1);
			cv::Mat imagePoints = homography *temp;
			imagePoints.row(0) = imagePoints.row(0) / imagePoints.row(2);
			imagePoints.row(1) = imagePoints.row(1) / imagePoints.row(2);
			imagePoints.row(2).setTo(1);
			float *pimageX = imagePoints.ptr<float>(0);
			float *pimageY = imagePoints.ptr<float>(1);
			cv::Point2f tempPoint(pimageX[0], pimageY[0]);
			std::vector<float>distantpp;
			for (int j = 0; j < ellipseNum; j++)
			{
				float d = float(computeLengthOfTwoPoint(tempPoint, PointsEllipse[j]));
				distantpp.push_back(d);

			}
			cv::Mat tempIndex;

			cv::sortIdx(distantpp, tempIndex, cv::SORT_EVERY_ROW + cv::SORT_ASCENDING);
			int *pTempIndex = tempIndex.ptr<int>(0);
			if (distantpp[pTempIndex[0]] < 10)
			{
				ImageEllipseSorted.push_back(PointsEllipse[pTempIndex[0]]);
				outSortIndex.push_back(std::pair<int, int>(i, j));
			}
		}
	}
	return 1;
}
int CalibCircleDetection::GetEllipseHalcon( const HalconCpp::HObject &dst,std::vector<cv::Point2f> &PointsEllipse, std::vector<cv::Point2f> &Point5Ellipse)
{
	////**halcon10就是Halcon::与#include "HalconCpp.h"与HObject
	//using namespace HalconCpp;
	using namespace HalconCpp;
	int m_length_min = 13, m_length_max = 400;
	double anisometry_max = 1.8;
	double circularity_min = 0.62;

	// Local iconic variables 
	HObject  Img1, Edges, SelectedXLD, UnionContours1, ClosedContours;
	HObject  SelectedXLD1, SelectedXLD11, UnionContours, SelectedXLD12;
	HObject  Contour, Contour_roi, Region, Region1Dilation, ImageReduced;
	HObject  Edges_roi, SelectedXLD_1, UnionContours_roi, SelectedXLD_roi;
	HObject  Contour1, ho_Region_t;


	// Local control variables 
	HTuple  Index_img, Number_all, m_JudgeGray, m_Judge5Ellipse;
	HTuple   I1, Row1, Column1;
	HTuple  Phi, Radius1, Radius2, StartPhi1, EndPhi1, PointOrder1;
	HTuple  Row1_temp, Column1_temp, Grayval_out, kk, gray_out;
	HTuple  Grayval_in, Row2, Column2, gray_in, Number_roi;
	HTuple  Length, II, length_judge, Index, Id_obj, Row_obj;
	HTuple  Column1_obj, Radius1_obj, Radius2_obj, length_all;
	HTuple  Row_X, Column_Y, Row_5Ellipse, Col_5Ellipse, Index_ellipse;
	HTuple  Radius_ellipse_1, Radius_ellipse_2, Radius_ellipse_3;
	HTuple  Radius_ellipse_4, Radius_ellipse_5, Radius_add_small;
	HTuple  Radius_average_small, Dist, Ind_1, Ind_2, k, dist;
	HTuple  Index_Dist, dist_max, dist_min, Ind_use, Id_Sum;
	HTuple  Ind_Obj, Id_firstPoint, id_1, id_2, id_3, id_4;
	HTuple  id_5, Angle1, Dist_1, Angle2, Dist_2, cha1, cha2;
	HTuple  theat_cha, Dist_3, Dist_4, dist_cha1, Dist_31, Dist_41;
	HTuple  dist_cha2, Id_secondPoint, Id_thirdPoint, dist31;
	HTuple  dist32, Id_fourthPoint, Id_fifthPoint, Row_5Ellipse_sorted;
	HTuple  Col_5Ellipse_sorted, hv_Number_all, hv_Area, hv_Row_c, hv_Column_c, Num_1, Num_2;
	
	
	HalconCpp::EdgesSubPix(dst, &Edges, "canny", 1, 20, 40);
	SelectShapeXld(Edges, &SelectedXLD, "contlength", "and", 10, 500);
	UnionCocircularContoursXld(SelectedXLD, &UnionContours1, 0.5, 0.1, 0.2, 30,	10, 10, "true", 1);
	CloseContoursXld(UnionContours1, &ClosedContours);

	SelectShapeXld(ClosedContours, &SelectedXLD1, "circularity", "and", 0.7, 1);
	CountObj(SelectedXLD1, &hv_Number_all);
	if (hv_Number_all<42)
		return -1;
	UnionAdjacentContoursXld(SelectedXLD1, &UnionContours, 10, 1, "attr_keep");
	SelectShapeXld(UnionContours, &SelectedXLD12, "contlength", "and", 10, 99999);

	
	CountObj(SelectedXLD12, &Number_all);
	CountObj(SelectedXLD1, &hv_Number_all);

	//std::cout << "Number_all：" << Number_all[0].I() << std::endl;

	if (hv_Number_all<42)
		return -1;
	FitEllipseContourXld(SelectedXLD12, "fitzgibbon", -1, 0, 0, 200, 3, 2, &Row1,
		&Column1, &Phi, &Radius1, &Radius2, &StartPhi1, &EndPhi1, &PointOrder1);
	//**m_Judge5Ellipse:=0用于判断是否内部灰度小于外部灰度
	//***m_Judge5Ellipse判断是否找到5个大圆

	TupleLength(Radius1, &length_all);
	

	TupleSortIndex(Radius1, &Index_ellipse);
	//std::cout << "个数" << Radius1_all.Num() << std::endl;
	//for (int kkk = 0;kkk< Radius1_all.Num();kkk++)
	//{
	//	std::cout << "Radius1_all[kk]：" << Radius1_all[kkk].D() << std::endl;
	//}
	if (length_all<6)
	{
		return -1;
	}

	Radius_ellipse_1 = Radius1[HTuple(Index_ellipse[length_all - 1])];
	Radius_ellipse_2 = Radius1[HTuple(Index_ellipse[length_all - 2])];
	Radius_ellipse_3 = Radius1[HTuple(Index_ellipse[length_all - 3])];
	Radius_ellipse_4 = Radius1[HTuple(Index_ellipse[length_all - 4])];
	Radius_ellipse_5 = Radius1[HTuple(Index_ellipse[length_all - 5])];

	Radius_add_small = 0;
	for (kk = 0; kk <= length_all - 6; kk += 1)
	{
		Radius_add_small += HTuple(Radius1[HTuple(Index_ellipse[kk])]);
	}
	Radius_average_small = Radius_add_small / (length_all - 5);
	if (0 != ((Radius_ellipse_5 - Radius_average_small)>5.1))
	{
		m_Judge5Ellipse = 1;
	}
	//**************************
	//std::cout << "Radius_average_small：" << Radius_average_small[0].I() << std::endl;
	//std::cout << "Radius_ellipse_5：" << Radius_ellipse_5[0].I() << std::endl;
	//std::cout << "m_Judge5Ellipse：" << m_Judge5Ellipse[0].I() << std::endl;
	if (0 != (m_Judge5Ellipse == 1))
	{
		//*计算任意两个圆心之间的距离对应大椭圆
		for (kk = 1; kk <= 5; kk += 1)
		{
			Row_5Ellipse.Append(HTuple(Row1[HTuple(Index_ellipse[length_all - kk])]));
			Col_5Ellipse.Append(HTuple(Column1[HTuple(Index_ellipse[length_all - kk])]));
		}
		//std::cout << "Row_5Ellipse：" << Row_5Ellipse[0].D() << "Col_5Ellipse:" << Col_5Ellipse[0].D() << std::endl;
		//std::cout << "Row_5Ellipse：" << Row_5Ellipse[1].D() << "Col_5Ellipse:" << Col_5Ellipse[1].D() << std::endl;
		//std::cout << "Row_5Ellipse：" << Row_5Ellipse[2].D() << "Col_5Ellipse:" << Col_5Ellipse[2].D() << std::endl;
		//std::cout << "Row_5Ellipse：" << Row_5Ellipse[3].D() << "Col_5Ellipse:" << Col_5Ellipse[3].D() << std::endl;
		//std::cout << "Row_5Ellipse：" << Row_5Ellipse[4].D() << "Col_5Ellipse:" << Col_5Ellipse[4].D() << std::endl;
		//***Ind_1与Ind_2存当前计算距离所用椭圆索引，应该用结构体
		Dist = HTuple();
		Ind_1 = HTuple();
		Ind_2 = HTuple();

		for (k = 1; k <= 4; k += 1)
		{
			for (kk = k + 1; kk <= 5; kk += 1)
			{
				DistancePp(HTuple(Row_5Ellipse[k - 1]), HTuple(Col_5Ellipse[k - 1]), HTuple(Row_5Ellipse[kk - 1]),
					HTuple(Col_5Ellipse[kk - 1]), &dist);
				Dist.Append(dist);
				Ind_1.Append(k);
				Ind_2.Append(kk);
			}
		}
		//****找到第一个点
		TupleSortIndex(Dist, &Index_Dist);
		dist_max = Dist[HTuple(Index_Dist[9])];
		dist_min = Dist[HTuple(Index_Dist[0])];
		Ind_use = HTuple();
		Ind_use.Append(HTuple(Ind_1[HTuple(Index_Dist[9])]));
		Ind_use.Append(HTuple(Ind_1[HTuple(Index_Dist[0])]));
		Ind_use.Append(HTuple(Ind_2[HTuple(Index_Dist[9])]));
		Ind_use.Append(HTuple(Ind_2[HTuple(Index_Dist[0])]));
		Id_Sum = 0;
		for (k = 1; k <= 4; k += 1)
		{
			Id_Sum += HTuple(Ind_use[k - 1]);
		}

		//****Ind_Obj保存排序
		Ind_Obj = HTuple();
		Id_firstPoint = 15 - Id_Sum;
		Ind_Obj.Append(Id_firstPoint);
		//****找第2个点

		//***id_1 , id_2是距离最远的2点，分别拟合直线，看在此直线的点的个数,找到第2个点
		id_1 = Ind_1[HTuple(Index_Dist[9])];
		id_2 = Ind_2[HTuple(Index_Dist[9])];
		id_3 = Id_firstPoint;
		id_4 = Ind_1[HTuple(Index_Dist[0])];
		id_5 = Ind_2[HTuple(Index_Dist[0])];

		Num_1 = 0;
		Num_2 = 0;
		Dist_1 = 0;
		Dist_2 = 0;
		for (kk = 0; kk <= length_all - 1; kk += 1)
		{
			DistancePl(HTuple(Row1[kk]), HTuple(Column1[kk]), HTuple(Row_5Ellipse[id_4 - 1]),
				HTuple(Col_5Ellipse[id_4 - 1]), HTuple(Row_5Ellipse[id_3 - 1]), HTuple(Col_5Ellipse[id_3 - 1]),
				&Dist_1);
			DistancePl(HTuple(Row1[kk]), HTuple(Column1[kk]), HTuple(Row_5Ellipse[id_5 - 1]),
				HTuple(Col_5Ellipse[id_5 - 1]), HTuple(Row_5Ellipse[id_3 - 1]), HTuple(Col_5Ellipse[id_3 - 1]),
				&Dist_2);
			if (0 != (Dist_1<2.0))
			{
				Num_1 += 1;
			}
			if (0 != (Dist_2<2.0))
			{
				Num_2 += 1;
			}
		}

		if (0 != (Num_2<Num_1))
		{
			Id_secondPoint = id_4;
			Id_thirdPoint = id_5;
		}
		else
		{
			Id_secondPoint = id_5;
			Id_thirdPoint = id_4;
		}

		Ind_Obj.Append(Id_secondPoint);
		Ind_Obj.Append(Id_thirdPoint);
		//****找到第4，、5个点
		DistancePp(HTuple(Row_5Ellipse[Id_thirdPoint - 1]), HTuple(Col_5Ellipse[Id_thirdPoint - 1]),
			HTuple(Row_5Ellipse[id_1 - 1]), HTuple(Col_5Ellipse[id_1 - 1]), &dist31);
	DistancePp(HTuple(Row_5Ellipse[Id_thirdPoint - 1]), HTuple(Col_5Ellipse[Id_thirdPoint - 1]),
			HTuple(Row_5Ellipse[id_2 - 1]), HTuple(Col_5Ellipse[id_2 - 1]), &dist32);
		if (0 != (dist31<dist32))
		{
			Id_fourthPoint = id_1;
			Id_fifthPoint = id_2;
		}
		else
		{
			Id_fourthPoint = id_2;
			Id_fifthPoint = id_1;
		}
		Ind_Obj.Append(Id_fourthPoint);
		Ind_Obj.Append(Id_fifthPoint);
		//************5个大圆按照顺序排序
		Row_5Ellipse_sorted = HTuple();
		Col_5Ellipse_sorted = HTuple();
		for (kk = 1; kk <= 5; kk += 1)
		{
			Row_5Ellipse_sorted.Append(HTuple(Row_5Ellipse[HTuple(Ind_Obj[kk - 1]) - 1]));
			Col_5Ellipse_sorted.Append(HTuple(Col_5Ellipse[HTuple(Ind_Obj[kk - 1]) - 1]));
			//**stop()
		}
		//******5个大圆对应完毕
		for (size_t ii = 0; ii<5; ii++)
		{
			float row = (float)Row_5Ellipse_sorted[ii].D();
			float col = (float)Col_5Ellipse_sorted[ii].D();
			Point5Ellipse.push_back(cv::Point2f(col, row));
		}
		//****对所有圆排序
		int length1 = length_all[0].I();
		for (size_t ii = 0; ii<length1; ii++)
		{
			float row = (float)Row1[ii].D();
			float col = (float)Column1[ii].D();
			PointsEllipse.push_back(cv::Point2f(col, row));
		}
	}
	else
	{
		return -2;
	}

	return 1;
}
void CalibCircleDetection::point2f2Halcon(std::vector<cv::Point2f> & PointsEllipse, HalconCpp::HTuple &RowPointEllipse, HalconCpp::HTuple &ColPointEllipse)
{
	//for each(cv::Point2f p in PointsEllipse)
	//{
	//	TupleConcat(RowPointEllipse, p.y, &RowPointEllipse);
	//	TupleConcat(ColPointEllipse, p.x, &ColPointEllipse);
	//}
	for (int i = 0; i < PointsEllipse.size(); i++)
	{
		TupleConcat(RowPointEllipse, PointsEllipse[i].y, &RowPointEllipse);
		TupleConcat(ColPointEllipse, PointsEllipse[i].x, &ColPointEllipse);
	}
}