#include "stdafx.h"
#include "..\inc\Segment.h"

#include <time.h>



Segment::Segment()
{

}

Segment::~Segment()
{

}

void Segment::clean()
{
	m_imgDetect.release();
	m_refineMat.release();
	m_M.release();
	m_roiImgBlur.release();
	m_roiImgGray.release();
	m_roiImgThresh.release();
	m_templateGrayImg.release();
	m_detectGrayImg.release();
	m_detectDescriptor.release();
	m_TnumRoi = 0;
	m_Drois_rectVec.clear();
	m_DchipPosPointVec.clear();
	m_DroisPosRectVec.clear();
	m_matches.clear();
	m_detectKeyPoint.clear();
	m_Drois_pointVec.clear();
	m_roiBinaryImgVec.clear();
	m_roisOffsetPosVec.clear();
}

void Segment::loadPara(cv::Mat& img, std::vector<cv::KeyPoint>& keypoints,
	cv::Mat& descriptor, std::vector<cv::Point2f>& chipPos, double refAngle)
{
	/*********************************************************************
	* function name:
	* called   by  :
	* Parameters IN:
	* Parameter OUT:
	* Remarks      :
	* function desc:init template picture's parameters
	* changed date :
	* user         :
	*********************************************************************/
	m_imgTemplate = img;
	m_templateKeyPoint = keypoints;
	m_templateDescriptor = descriptor;
	m_TchipPosPointVec = chipPos;
	m_DchipNum = 1;

	m_centerPoint = cv::Point2f(m_imgTemplate.cols / 2, m_imgTemplate.rows / 2);
	m_refPoint = cv::Point2f(m_imgTemplate.cols, m_imgTemplate.rows / 2);

	m_TrefAngle = refAngle;	
}
bool Segment::InitTemplate(cv::Mat templateChipImg, std::vector<cv::Point2f> chipPosPointVec)
{
	m_DchipNum = 1;

	m_imgTemplate = templateChipImg;

	m_TchipPosPointVec = chipPosPointVec;


	int rowsImg = m_imgTemplate.rows;
	int colsImg = m_imgTemplate.cols;

	m_centerPoint = cv::Point2f(colsImg / 2, rowsImg / 2);
	m_refPoint = cv::Point2f(colsImg, rowsImg / 2);
	return true;
}

void Segment::caliImgDetect()
{
	double caliedAngle = m_DchipAngleVec[0];
	int rowsImg = m_imgDetect.rows;
	int colsImg = m_imgDetect.cols;

	cv::Point center = cv::Point(colsImg / 2, rowsImg / 2);
	cv::Mat M = getRotationMatrix2D(center, caliedAngle, 1);
	warpAffine(m_imgDetect, m_imgDetectCalied, M, cv::Size(colsImg, rowsImg));	
}

bool Segment::InitDetect(cv::Mat detectImg)
{
	m_imgDetect = detectImg;
	return true;
}

bool Segment::initImage(ALGO_IN_OCR* ptr_algoDataIn, cv::Mat& img)
{
	/*********************************************************************
	* function name:
	* called   by  :
	* Parameters IN:
	* Parameter OUT:
	* Remarks      :
	* function desc:init the picture of under detection
	* changed date :
	* user         :
	*********************************************************************/
	if (ptr_algoDataIn->ptr_bits == NULL)
	{
		return false;
	}

	long width = ptr_algoDataIn->img_wh.cx;
	long height = ptr_algoDataIn->img_wh.cy;
	int depth = 0;
	int channels = 0;

	if (ptr_algoDataIn->img_type == 0)
	{
		depth = 1;
		channels = 1;
	}
	else if (ptr_algoDataIn->img_type == 3)
	{
		depth = 8;
		channels = 3;
	}
	else
	{
		//default 
		depth = 8;
		channels = 3;
	}
	
	IplImage* pImg = cvCreateImageHeader(cvSize(width, height), depth, channels);

	int step = ptr_algoDataIn->img_widthstep;
	BYTE* data = ptr_algoDataIn->ptr_bits;
	cvSetData(pImg, data, step);
	img = cv::cvarrToMat(pImg,true);

	cvReleaseImageHeader(&pImg);

	return true;
}

void Segment::InitKeyPoints(std::vector<cv::KeyPoint> c_templateKeyPoint, cv::Mat c_TemplateDescriptor)
{
	m_templateKeyPoint = c_templateKeyPoint;
	m_templateDescriptor = c_TemplateDescriptor;
}

bool Segment::pointVecVec2rectVecVec(std::vector<std::vector<cv::Point2f>> charsPoints, std::vector<std::vector<cv::Rect>> &charsRects)
{
	/*********************************************************************
	* function name:
	* called   by  :
	* Parameters IN:
	* Parameter OUT:
	* Remarks      :
	* function desc:convert points to rects of characters' position
	* changed date :
	* user         :
	*********************************************************************/
	int NumROI = charsPoints.size();
	charsRects.clear();
	charsRects.resize(0);

	for (int i = 0; i < NumROI; ++i)
	{
		std::vector<cv::Point2f> pointVec = charsPoints[i];
		int rectVecNum = pointVec.size() / 4;

		if (rectVecNum < 1)
		{
			return false; 
		}

		std::vector<cv::Rect> rectVec(rectVecNum);
		int rectIdx = 0;
		
		for (int j = 0; j < pointVec.size(); j += 4)
		{
			cv::Rect rect;

			if (0) //method1:translate four points to a rectangle
			{
				rect.x = pointVec[j].x;
				rect.y = pointVec[j].y;
				rect.width = pointVec[j + 1].x - pointVec[j].x;
				rect.height = pointVec[j + 2].y - pointVec[j].y;
			}
		
		
			if (1) //method2: translate four points to a rectangle, support 90 degree and 180 degree rotate
			{
				std::vector<cv::Point2f> fourPointVec(4);
				fourPointVec[0] = pointVec[j];
				fourPointVec[1] = pointVec[j + 1];
				fourPointVec[2] = pointVec[j + 2];
				fourPointVec[3] = pointVec[j + 3];


				double xMin = fourPointVec[0].x;
				double xMax = fourPointVec[0].x;
				double yMin = fourPointVec[0].y;
				double yMax = fourPointVec[0].y;
				for (int pointindex = 1; pointindex < 4; ++pointindex)
				{
					if (fourPointVec[pointindex].x > xMax)
						xMax = fourPointVec[pointindex].x;

					if (fourPointVec[pointindex].x < xMin)
						xMin = fourPointVec[pointindex].x;

					if (fourPointVec[pointindex].y > yMax)
						yMax = fourPointVec[pointindex].y;

					if (fourPointVec[pointindex].y < yMin)
						yMin = fourPointVec[pointindex].y;
				}

				cv::Point2f point1, point2, point3, point4;
				point1.x = xMin;
				point1.y = yMin;

				point2.x = xMax;
				point2.y = yMin;

				point3.x = xMax;
				point3.y = yMax;

				point4.x = xMin;
				point4.y = yMax;

				rect.x = xMin;
				rect.y = yMin;
				rect.width = xMax - xMin;
				rect.height = yMax - yMin;

				if (rect.width <= 0 || rect.height < 0)
				{
					return false;
				}
			}
						
			rectVec[rectIdx++] = rect;
		}

		charsRects.push_back(rectVec);
	}

	return true;
}

double Segment::calcAngle(cv::Point2f point1, cv::Point2f point2)
{
	double angle = 0;
	float deltaX = point2.x - point1.x;
	float deltaY = point2.y - point1.y;

	if (deltaX > 0)
	{
		angle = atan((deltaY) / deltaX);
		angle = 180 / PI * angle;
	}
	else
	{
		angle = atan((deltaY) / deltaX);
		angle = 180 / PI * angle - 180;
	}
	
	return angle;
}

void Segment::detectChip(cv::Mat img, std::vector<cv::Rect> &rectVec, int &chipNum)
{
	if (img.empty())
	{
		std::cout << "image used to detect chip number is empty!" << std::endl;
		return;
	}

	rectVec.clear();

	cv::Mat imgDenoise, imgBlur, imgGray, imgThresh, imgCanny;
	int rowsImg = img.rows;
	int colsImg = img.cols;

	cv::GaussianBlur(img, imgBlur, cv::Size(3, 3), 1);
	cv::cvtColor(imgBlur, imgGray, cv::COLOR_BGR2GRAY);

	int thresh = calcThreshold(imgGray);
	cv::threshold(imgGray, imgThresh, thresh-10, 255, cv::THRESH_BINARY);
	cv::Canny(imgThresh, imgCanny, 20, 55);

	/*cv::imshow("gray", imgGray);
	cv::imshow("thresh", imgThresh);
	cv::imshow("canny", imgCanny);
	cv::waitKey(0);*/

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<double> contAreaVec;

	cv::findContours(imgCanny, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	int numContours = int(contours.size());

	for (int i = 0; i < numContours; ++i)
	{
		cv::Rect bRect = cv::boundingRect(contours[i]);
		int bArea = bRect.area();
		contAreaVec.push_back(bArea);
	}

	std::vector<double> contAreaTmpVec = contAreaVec;
	std::vector<std::vector<cv::Point>> contoursTmp;
	std::sort(contAreaTmpVec.begin(), contAreaTmpVec.end(), std::greater<double>());
	std::vector<double> maxBoundRecAreaVec;
	double maxArea = contAreaTmpVec[0];
	double scalRatio = 1.5;

	for (int i = 0; i < contAreaTmpVec.size(); ++i)
	{
		if (contAreaTmpVec[i] >(maxArea / scalRatio) && contAreaTmpVec[i] < (maxArea * scalRatio))
			maxBoundRecAreaVec.push_back(contAreaTmpVec[i]);
	}

	chipNum = maxBoundRecAreaVec.size();

	for (int i = 0; i < contAreaVec.size(); ++i)
	{
		if (contAreaVec[i] >(maxArea / scalRatio) && contAreaVec[i] < (maxArea * scalRatio))
		{
			cv::Rect bRect = boundingRect(contours[i]);
			rectVec.push_back(bRect);
		}
	}

	//for (int i = 0; i < rectVec.size(); ++i)
	//{
	//	rectangle(img, rectVec[i], cv::Scalar(0, 0, 255));
	//}

}

int Segment::calcThreshold(cv::Mat img)
{
	if (img.empty())
	{
		std::cout << "image used to calc thresh is empty!" << std::endl;
		return -1;
	}

	int rowsImg = img.rows;
	int colsImg = img.cols;
	double hist[256];
	double var[256];
	int valThresh = 0;
	int numPixels = rowsImg * colsImg;

	for (int i = 0; i < 256; i++)
	{
		hist[i] = 0.0;
		var[i] = 0.0;
	}

	for (int i = 0; i < rowsImg; i++)
	{
		for (int j = 0; j < colsImg; j++)
		{
			unsigned char pixVal = img.at<uchar>(i, j);
			hist[pixVal]++;
		}
	}

	double probBackground = 0.0;
	double probForeground = 0.0;
	double valBackAveGray = 0.0;
	double valForeAveGray = 0.0;
	double valGlobAveGray = 0.0;
	double data1 = 0.0;
	double data2 = 0.0;

	for (int i = 0; i < 256; i++)
	{
		hist[i] /= numPixels;
		valGlobAveGray += (i * hist[i]);
	}

	for (int i = 0; i < 256; i++)
	{
		probBackground += hist[i];
		probForeground = 1 - probBackground;
		data1 += i * hist[i];
		data2 = valGlobAveGray - data1;
		valBackAveGray = data1 / probBackground;
		valForeAveGray = data2 / probForeground;
		double val = probBackground * probForeground * pow((valBackAveGray - valForeAveGray), 2);
		var[i] = val;
	}

	double tmp = 0.0;
	for (int i = 0; i < 256; i++)
	{
		if (var[i] > tmp)
		{
			tmp = var[i];
			valThresh = i;
		}
	}

	return valThresh;
}


bool Segment::slicePosition()
{
	/*********************************************************************
	* function name:
	* called   by  :
	* Parameters IN:
	* Parameter OUT:
	* Remarks      :
	* function desc:the concreat implementation of segment
	* changed date :
	* user         :
	*********************************************************************/
	if (m_imgTemplate.empty() || m_imgDetect.empty())
	{
		return false;
	}

	bool b_rVal = true;
	m_Drois_pointVec.clear();
	std::vector<std::vector<cv::Rect>> roiVec = m_Trois_rectVec;
	
	//compute under detectoin picuter's keypoints and descriptors
	extractor(m_imgDetect, cv::Mat(), m_detectKeyPoint, m_detectDescriptor);
			
	cv::BFMatcher bfmatcher(cv::NORM_HAMMING, true);
	bfmatcher.match(m_templateDescriptor, m_detectDescriptor, m_matches);

	//refine the keypoints...
	refineKeyPoints(m_templateKeyPoint, m_detectKeyPoint, 3, m_matches, m_refineMat);
	double numMatched = m_matches.size();
	double ratio = numMatched / FEATURE_POINTS_NUM;
	if (ratio < 0.02)
	{
		return false;
	}

	std::vector<cv::Point2f> chipKeyPointVec(numMatched);
	std::vector<cv::Point2f> detectKeyPointVec(numMatched);

	for (int j = 0; j < m_matches.size(); j++)
	{
		chipKeyPointVec[j] = m_templateKeyPoint[m_matches[j].queryIdx].pt;
		detectKeyPointVec[j] = m_detectKeyPoint[m_matches[j].trainIdx].pt;
	}

	m_M = cv::findHomography(chipKeyPointVec, detectKeyPointVec, CV_RANSAC);

	//test start: center point mapping	
	if (0)
	{
		std::vector<cv::Point2f> sceneCenterPoint(1);
		std::vector<cv::Point2f> objCenerPoint(1);
		objCenerPoint[0] = cvPoint(m_imgTemplate.cols / 2, m_imgTemplate.rows / 2);
		if (DRAW_INFO)
			cv::circle(m_imgTemplate, objCenerPoint[0], 3, cv::Scalar(0, 255, 0), 3);

		cv::perspectiveTransform(objCenerPoint, sceneCenterPoint, m_M);

		if (DRAW_INFO)
			cv::circle(m_imgDetect, sceneCenterPoint[0], 3, cv::Scalar(0, 255, 0), 3);
	}	
	//test end

	//map every ROI's points
	for (int num = 0; num < m_TnumRoi; ++num)
	{
		std::vector<cv::Rect> rectVec = roiVec[num];
		std::vector<cv::Point2f> T_roiPointVec(rectVec.size() * 4);
		std::vector<cv::Point2f> D_roiPointVec;

		cv::Point2f point1, point2, point3, point4;
		int pointIdx = 0;
		for (int r = 0; r < rectVec.size(); ++r)
		{	
			point1.x = rectVec[r].x;
			point1.y = rectVec[r].y;

			point2.x = rectVec[r].x + rectVec[r].width;
			point2.y = rectVec[r].y;

			point3.x = rectVec[r].x + rectVec[r].width;
			point3.y = rectVec[r].y + rectVec[r].height;

			point4.x = rectVec[r].x;
			point4.y = rectVec[r].y + rectVec[r].height;

			T_roiPointVec[pointIdx++] = point1;
			T_roiPointVec[pointIdx++] = point2;
			T_roiPointVec[pointIdx++] = point3;
			T_roiPointVec[pointIdx++] = point4;
		}

		cv::perspectiveTransform(T_roiPointVec, D_roiPointVec, m_M);

		m_Drois_pointVec.push_back(D_roiPointVec);

	}

	//map chip's position points
	cv::perspectiveTransform(m_TchipPosPointVec, m_DchipPosPointVec, m_M);

	if (DRAW_INFO)
	{
		for (int pointIndex = 0; pointIndex < m_DchipPosPointVec.size(); ++pointIndex)
		{
			cv::circle(m_imgDetect, m_DchipPosPointVec[pointIndex], 3, cv::Scalar(0, 0, 255), 3);
		}
	}
	
	//draw the mapped points on the detect image
	int count = 0;

	if (DRAW_INFO)
	{
		for (int i = 0; i < m_TnumRoi; ++i)
		{
			std::vector<cv::Point2f> roiPointVec = m_Drois_pointVec[count++];
			for (int pointIndex = 0; pointIndex < roiPointVec.size(); ++pointIndex)
			{
				//cv::circle(imgDetect, cv::Point2f(roiPointVec[pointIndex].x, roiPointVec[pointIndex].y), 3, cv::Scalar(0, 0, 255), 1);
			}

		}
	}

	
	m_Drois_rectVec.clear();

	//convert rectangular vertices to rects 
	b_rVal = pointVecVec2rectVecVec(m_Drois_pointVec, m_Drois_rectVec);
	if (!b_rVal)
	{
		return false;
	}

	//plot for test  start
	cv::Mat detectImg1;
	if (DRAW_INFO)
	{
		m_imgDetect.copyTo(detectImg1);
	}
	
	//plot for test end

	//draw the mapped rect on the detect image
	count = 0;	
	if (0)
	{

		for (int i = 0; i < m_TnumRoi; ++i)
		{
			std::vector<cv::Rect> roiRecVec = m_Drois_rectVec[count++];
			for (int rectIndex = 0; rectIndex < roiRecVec.size(); ++rectIndex)
			{
				if (rectIndex % 2 == 0)
					cv::rectangle(m_imgDetect, roiRecVec[rectIndex], cv::Scalar(0, 0, 255), 1);
				else
					cv::rectangle(m_imgDetect, roiRecVec[rectIndex], cv::Scalar(255, 0, 0), 1);
			}
		}
	
	}

	//compute each of roi's rectangular vertices
	b_rVal = roisRectVec2roisPosRectVec(m_Drois_rectVec);
	if (!b_rVal)
	{
		return false;
	}

	b_rVal = roiBinary();
	if (!b_rVal)
	{
		return false;
	}

	if (0)
	{
		cv::imshow("roiImg0", m_roiBinaryImgVec[0]);
	}
	//adjust character's rectangle 
	b_rVal = refineCharPos();
	if (!b_rVal)
	{
		return false;
	}

	
	count = 0;
	if (DRAW_INFO)
	{

		for (int i = 0; i < m_TnumRoi; ++i)
		{
			std::vector<cv::Rect> roiRecVec = m_Drois_rectVec[count++];
			for (int rectIndex = 0; rectIndex < roiRecVec.size(); ++rectIndex)
			{
				if (rectIndex % 2 == 0)
					cv::rectangle(detectImg1, roiRecVec[rectIndex], cv::Scalar(0, 255, 0), 1);
				else
					cv::rectangle(detectImg1, roiRecVec[rectIndex], cv::Scalar(0, 0, 255), 1);
			}
		}
		cv::imshow("finepos", detectImg1);
	}
	
	//compute the slant angle of a chip lies in test pic
	cv::Rect rect;
	rect.x = 0;
	rect.y = 0;
	rect.width = m_imgDetect.cols;
	rect.height = m_imgDetect.rows;
	double angle = chipsAngle(rect, m_M);
	angle += m_TrefAngle;
	m_DchipAngleVec.push_back(angle);

	caliImgDetect();

	return true;
}

bool Segment::refineKeyPoints
(
	const std::vector<cv::KeyPoint>& queryKeypoints,
	const std::vector<cv::KeyPoint>& trainKeypoints,
	float threshold,
	std::vector<cv::DMatch>& matches,
	cv::Mat& matrix
)
{
	const int minNumberMatchesAllowed = 5;
	if (matches.size() < minNumberMatchesAllowed)
		return false;

	std::vector<cv::Point2f> srcPoints(matches.size());
	std::vector<cv::Point2f> dstPoints(matches.size());
	for (size_t i = 0; i < matches.size(); i++)
	{
		srcPoints[i] = trainKeypoints[matches[i].trainIdx].pt;
		dstPoints[i] = queryKeypoints[matches[i].queryIdx].pt;
	}
	// Find homography matrix and get inliers mask  
	std::vector<unsigned char> inliersMask(srcPoints.size());
	matrix = cv::findHomography(srcPoints, dstPoints, CV_FM_RANSAC, threshold, inliersMask);
	std::vector<cv::DMatch> inliers;
	for (size_t i = 0; i<inliersMask.size(); i++)
	{
		if (inliersMask[i])
			inliers.push_back(matches[i]);
	}
	matches.swap(inliers);
	return matches.size() > minNumberMatchesAllowed;

}

double Segment::chipsAngle(cv::Rect detectChipRect, cv::Mat M)
{
	//std::vector<double>& angleVec
	std::vector<cv::Point2f> centerPointVec(1);
	std::vector<cv::Point2f> refPointVec(1);
	std::vector<cv::Point2f> detectCenterPointVec;
	std::vector<cv::Point2f> detectRefPointVec;
	centerPointVec[0] = m_centerPoint;
	refPointVec[0] = m_refPoint;

	cv::perspectiveTransform(centerPointVec, detectCenterPointVec, M);
	cv::perspectiveTransform(refPointVec, detectRefPointVec, M);

	cv::Point2f detectCenterPoint;
	detectCenterPoint.x = detectCenterPointVec[0].x + detectChipRect.x;
	detectCenterPoint.y = detectCenterPointVec[0].y + detectChipRect.y;

	cv::Point2f detectRefPoint;
	detectRefPoint.x = detectRefPointVec[0].x + detectChipRect.x;
	detectRefPoint.y = detectRefPointVec[0].y + detectChipRect.y;

	double angle = calcAngle(detectCenterPoint, detectRefPoint);

	return angle;
}

bool Segment::roisRectVec2roisPosRectVec(std::vector<std::vector<cv::Rect>> rois_rectVec)
{
	/*********************************************************************
	* function name:
	* called   by  :
	* Parameters IN:
	* Parameter OUT:
	* Remarks      :
	* function desc:compute each of roi's rectangular vertices 
	* changed date :
	* user         :
	*********************************************************************/
	m_DroisPosRectVec.clear();

	if (rois_rectVec.empty())
		return false;

	for (int i = 0; i < m_TnumRoi; ++i)
	{
		std::vector<cv::Rect> singleRoiRectVec;
		singleRoiRectVec = rois_rectVec[i];

		int xMin = singleRoiRectVec[0].x;
		int xMax = singleRoiRectVec[0].x;
		int yMin = singleRoiRectVec[0].y;
		int yMax = singleRoiRectVec[0].y;
		int wMax = singleRoiRectVec[0].width;
		int hMax = singleRoiRectVec[0].height;
		int wIndex = 0;
		int hIndex = 0;
		int rectIndex = 1;
		for (; rectIndex < singleRoiRectVec.size(); ++rectIndex)
		{
			if (singleRoiRectVec[rectIndex].x < xMin)
				xMin = singleRoiRectVec[rectIndex].x;

			if (singleRoiRectVec[rectIndex].x > xMax)
			{
				xMax = singleRoiRectVec[rectIndex].x;
				wIndex = rectIndex;
			}
				

			if (singleRoiRectVec[rectIndex].y < yMin)
				yMin = singleRoiRectVec[rectIndex].y;

			if (singleRoiRectVec[rectIndex].y > yMax)
			{
				yMax = singleRoiRectVec[rectIndex].y;
				hIndex = rectIndex;
			}
				

			if (singleRoiRectVec[rectIndex].width > wMax)
			{
				wMax = singleRoiRectVec[rectIndex].width;	
			}
				

			if (singleRoiRectVec[rectIndex].height > hMax)
			{
				hMax = singleRoiRectVec[rectIndex].height;		
			}
				
		}
		cv::Rect roiRec;
		if (xMin - 5 > 0)
		{
			roiRec.x = xMin - 5;
		}
		else
		{
			roiRec.x = xMin;
		}
		
		if (yMin - 3 > 0)
		{
			roiRec.y = yMin - 3;
		}
		else
		{
			roiRec.y = yMin;
		}

		if (xMax + 8 > m_imgDetect.cols)
		{
			roiRec.width = xMax - xMin + singleRoiRectVec[wIndex].width;
		}
		else
		{
			roiRec.width = xMax - xMin + singleRoiRectVec[wIndex].width + 8;
		}
		
		if (yMax + 3 > m_imgDetect.rows)
		{
			roiRec.height = yMax - yMin + singleRoiRectVec[hIndex].height;
		}
		else
		{
			roiRec.height = yMax - yMin + singleRoiRectVec[hIndex].height + 3;
		}
		
		m_DroisPosRectVec.push_back(roiRec);

		if (0)
		{
			cv::rectangle(m_imgDetect, roiRec, cv::Scalar(255, 255, 0), 1);
		}
		
	}

	return true;

}

bool Segment::roiBinary()
{
	/*********************************************************************
	* function name:
	* called   by  :
	* Parameters IN:
	* Parameter OUT:
	* Remarks      :
	* function desc:compute the binary images responding to all of the rois
	* changed date :
	* user         :
	*********************************************************************/
	cv::Mat roiImg;

	m_roisOffsetPosVec.clear();
	m_roiBinaryImgVec.clear();

	for (int roiNumIndex = 0; roiNumIndex < m_TnumRoi; ++roiNumIndex)
	{
		cv::Rect rect = m_DroisPosRectVec[roiNumIndex];

		if (rect.y + rect.height > m_imgDetect.rows || rect.x + rect.width > m_imgDetect.cols)
			return false;

		roiImg = m_imgDetect(cv::Range(rect.y, rect.y + rect.height), cv::Range(rect.x, rect.x + rect.width));

		std::vector<int> offset;
		offset.push_back(rect.x);
		offset.push_back(rect.y);
		m_roisOffsetPosVec.push_back(offset);

		cv::GaussianBlur(roiImg, m_roiImgBlur, cv::Size(3, 3), 1);
		cv::cvtColor(m_roiImgBlur, m_roiImgGray, cv::COLOR_BGR2GRAY);

		int threshVal = calcThreshold(m_roiImgGray);
		cv::threshold(m_roiImgGray, m_roiImgThresh, threshVal , 255, cv::THRESH_BINARY);

		m_roiBinaryImgVec.push_back(m_roiImgThresh);
	}

	return true;
}

bool Segment::refineCharPos()
{
	/*********************************************************************
	* function name:
	* called   by  :
	* Parameters IN:
	* Parameter OUT:
	* Remarks      :
	* function desc:adjust character's rect according to binary image info
	* changed date :
	* user         :
	*********************************************************************/
	int ii;
	for (int i = 0; i < m_roiBinaryImgVec.size(); ++i)
	{
		for (int j = 0; j < m_Drois_rectVec[i].size(); ++j)
		{
			ii = i;
			m_Drois_rectVec[ii][j].x -= m_roisOffsetPosVec[ii][0];
			m_Drois_rectVec[ii][j].y -= m_roisOffsetPosVec[ii][1];

			if (m_Drois_rectVec[ii][j].x < 0 || m_Drois_rectVec[ii][j].y < 0)
			{
				return false;
			}
		}

		tuneCharRectPos(m_roiBinaryImgVec[i], m_Drois_rectVec[i]);

		for (int j = 0; j < m_Drois_rectVec[i].size(); ++j)
		{
			ii = i;
			m_Drois_rectVec[ii][j].x += m_roisOffsetPosVec[ii][0];
			m_Drois_rectVec[ii][j].y += m_roisOffsetPosVec[ii][1];

			if (m_Drois_rectVec[ii][j].x > m_imgDetect.cols || m_Drois_rectVec[ii][j].y > m_imgDetect.rows)
			{
				return false;
			}
		}
	}

	return true;
}

void Segment::tuneCharRectPos(cv::Mat binaryImg, std::vector<cv::Rect>& rectVec)
{
	int val = backGroundVal(binaryImg);
	int foregroundVal = 0;
	if (val > 125)
	{
		foregroundVal = 0;
	}
	else
	{
		foregroundVal = 255;
	}

	for (int rectIndex = 0; rectIndex < rectVec.size(); ++rectIndex)
	{	
		verticalForgroundPixVal(binaryImg, rectVec[rectIndex], foregroundVal);
	}
}

void Segment::verticalForgroundPixVal(cv::Mat img, cv::Rect &rect, int pixVal)
{
	/*********************************************************************
	* function name:
	* called   by  :
	* Parameters IN:
	* Parameter OUT:
	* Remarks      :
	* function desc:adjust rect according to the number of foreground pixels
	* changed date :
	* user         :
	*********************************************************************/
	int xMin = rect.x;
	int xMax = rect.x + rect.width > img.cols? rect.x: rect.x + rect.width;

	int yMin = rect.y;
	int yMax = rect.y + rect.height;
	
	int LtotalPixNum = verPixNum(img, xMin,yMin, yMax, pixVal);
	int RtotalPixNum = verPixNum(img, xMax, yMin, yMax, pixVal);
	for (int i = 1; i <= FINE_TUNE_PIXEL_NUM; ++i)
	{
		int xIndex = xMin - i;
		if (xIndex <= 0)
		{
			xIndex = 0;
			continue;
		}

		int shiftPosPixNuM = verPixNum(img, xIndex, yMin, yMax, pixVal);
		if (LtotalPixNum > 0)
		{
			if (shiftPosPixNuM < LtotalPixNum)
			{
				if (xIndex > 0 && xIndex < img.cols)
					rect.x = xIndex;
				LtotalPixNum = shiftPosPixNuM;
			}
		}

		
		xIndex = xMax + i;
		if (xIndex >= img.cols)
		{
			xIndex = img.cols;
			continue;
		}

		shiftPosPixNuM = verPixNum(img, xIndex, yMin, yMax, pixVal);
		if (RtotalPixNum > 0)
		{
			if (shiftPosPixNuM < RtotalPixNum)
			{
				if (RtotalPixNum == 0)
					continue;

				if (xIndex - rect.x > 0)
					rect.width = xIndex - rect.x;
				RtotalPixNum = shiftPosPixNuM;
			}
		}
			
	}
}

int Segment::verPixNum(cv::Mat img, int xPos, int yMin, int yMax, int pixVal)
{
	int pixNum = 0;
	for (int h = yMin; h < yMax && h < img.rows; ++h)
	{
		if (img.at<uchar>(h, xPos) == pixVal)
		{
			pixNum += 1;
		}
	}

	return pixNum;
}

int Segment::backGroundVal(cv::Mat binaryImg)
{
	int backGroundVal = 0;
	int val1 = binaryImg.at<uchar>(1, 1);
	int val2 = binaryImg.at<uchar>(binaryImg.rows - 1, 1);
	int val3 = binaryImg.at<uchar>(1, binaryImg.cols - 1);
	int val4 = binaryImg.at<uchar>(binaryImg.rows - 1, binaryImg.cols - 1);

	backGroundVal = val1 + val2 + val3 + val4;
	
	backGroundVal /= 4;
	return backGroundVal;
}

