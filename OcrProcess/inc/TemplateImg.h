#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <highgui.hpp>

#include "definetype.h"

class TemplateImg
{
public:
	TemplateImg();
	~TemplateImg();
	bool InitTemplate(cv::Mat& templateChipImg, std::vector<cv::Point2f>& chipPosPointVec);
	void InitKeyPoints(std::vector<cv::KeyPoint>& c_templateKeyPoint, cv::Mat& c_TemplateDescriptor);
	bool caliImage();
	int calcThreshold(cv::Mat& img);
	double calcAngle(cv::Point2f& point1, cv::Point2f& point2);

public:

	cv::Mat m_templateImg;
	cv::Mat m_objectImg;
	cv::Mat m_caliedObjectImg;
	int m_offsetX;
	int m_offsetY;

	//tow points of the chip in the template picture(seem the line connected them as the horizontal ref line)
	cv::Point2f m_topLeftPoint;
	cv::Point2f m_topRightPoint;
	double m_refAngle;//the ref angle computed by the topLeftPoint and topRightPoint

	std::vector<cv::Point2f> m_chipPosPointVec;
	std::vector<cv::KeyPoint> m_templateKeyPoints;
	cv::Mat m_TemplateDescriptor;
	cv::Ptr<cv::ORB> m_templateImgORB;
};