#include "stdafx.h"

#include "..\inc\TemplateImg.h"

TemplateImg::TemplateImg()
{

}

TemplateImg::~TemplateImg()
{

}

int TemplateImg::calcThreshold(cv::Mat& img)
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

double TemplateImg::calcAngle(cv::Point2f& point1, cv::Point2f& point2)
{
	double angle = 0;

	if (point2.x < point1.x)
	{
		cv::Point2f point = point1;
		point1 = point2;
		point2 = point;
	}

	float deltaX = point2.x - point1.x;
	float deltaY = point2.y - point1.y;

	if (deltaX > 0)
	{
		angle = atan((deltaY) / deltaX);
		angle = 180 / CV_PI * angle;
	}
	else
	{
		angle = atan((deltaY) / deltaX);
		angle = 180 / CV_PI * angle - 180;
	}

	return angle;
}

bool TemplateImg::caliImage()
{
	cv::Mat imgDenoise;
	cv::Mat imgBlur;
	cv::Mat imgGray;
	cv::Mat imgThresh;
	cv::Mat imgCanny;

	GaussianBlur(m_objectImg, imgBlur, cv::Size(5, 5), 1);

	int rows = m_objectImg.rows;
	int cols = m_objectImg.cols;
	cv::Mat imgScal;
	imgScal = imgBlur(cv::Range(rows / 10, 9 * rows / 10), cv::Range(cols / 10, 9 * cols / 10));

	cvtColor(imgScal, imgGray, cv::COLOR_BGR2GRAY);
	int thresh = calcThreshold(imgGray);
	cv::threshold(imgGray, imgThresh, thresh, 255, cv::THRESH_BINARY);
	if (0)
	{
		cv::imshow("thresh", imgThresh);
		cv::waitKey(0);
	}
	
	cv::Canny(imgThresh, imgCanny, 20, 55);

	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(imgCanny, lines, 1, CV_PI / 180, 80, 5, 10);

	cv::Point2f point1, point2;
	std::vector<double> lineAngleVec(lines.size());
	int numLine = 0;
	float sumAng = 0.0;

	for (size_t i = 0; i < lines.size(); i++)
	{
		cv::line(imgScal, cv::Point(lines[i][0], lines[i][1]),
			cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 1, 8);
		point1.x = lines[i][0];
		point1.y = lines[i][1];
		point2.x = lines[i][2];
		point2.y = lines[i][3];

		double angle = calcAngle(point1, point2);
		lineAngleVec[i] = angle;

		if (abs(angle) < 30 && abs(angle) > 0)
		{
			sumAng += angle;
			numLine++;
		}

	}

	float aveAng;
	if (numLine > 0)
	{
		aveAng = sumAng / numLine;
	}
	else
	{
		aveAng = 0;
	}

	int rowsImg = m_objectImg.rows;
	int colsImg = m_objectImg.cols;

	cv::Point center = cv::Point(colsImg / 2, rowsImg / 2);
	cv::Mat M = getRotationMatrix2D(center, aveAng, 1);
	warpAffine(m_objectImg, m_caliedObjectImg, M, cv::Size(colsImg, rowsImg));

	return true;
}

bool TemplateImg::InitTemplate(cv::Mat& templateChipImg, std::vector<cv::Point2f>& chipPosPointVec)
{
	m_templateImg = templateChipImg;
	m_chipPosPointVec = chipPosPointVec;

	return true;
}

void TemplateImg::InitKeyPoints(std::vector<cv::KeyPoint>& c_templateKeyPoint, cv::Mat& c_TemplateDescriptor)
{
	m_templateKeyPoints = c_templateKeyPoint;
	m_TemplateDescriptor = c_TemplateDescriptor;
}