#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <highgui.hpp>


class TemplateImg
{
public:
	TemplateImg();
	~TemplateImg();
	bool InitTemplate(cv::Mat templateChipImg, std::vector<cv::Point2f> chipPosPointVec);
	void InitKeyPoints(std::vector<cv::KeyPoint> c_templateKeyPoint, cv::Mat c_TemplateDescriptor);
	bool caliImage();
	int calcThreshold(cv::Mat img);
	double calcAngle(cv::Point2f point1, cv::Point2f point2);

public:

	cv::Mat CT_imgTemplate;
	cv::Mat CT_imgObj;
	cv::Mat CT_caliedImgObj;
	int CT_offset_x;
	int CT_offset_y;

	//tow points of the chip in the template picture(seem the line connected them as the horizontal ref line)
	cv::Point2f topLeftPoint;
	cv::Point2f topRightPoint;
	double refAngle;//the ref angle computed by the topLeftPoint and topRightPoint

	std::vector<cv::Point2f> CT_chipPosPointVec;
	std::vector<cv::KeyPoint> CT_templateKeyPoint;
	cv::Mat CT_TemplateDescriptor;
	cv::Ptr<cv::ORB> CT_orb1;
};