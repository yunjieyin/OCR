#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <highgui.hpp>
#include <OCR_Kernel.h>
#include "orb_extractor.h"

#define PI 3.1415926
#define DRAW_INFO 0
#define FOREGROUND_VALUE 255
#define BACKGROUND_VALUE 0
#define FINE_TUNE_PIXEL_NUM 2
#define FEATURE_POINTS_NUM 1000
#define DEBUG 0

#include "definetype.h"

class Segment
{
public:
	Segment();
	~Segment();
	
public:
	bool InitTemplate(cv::Mat, std::vector<cv::Point2f> chipPosPointVec);
	bool InitDetect(cv::Mat);
	bool slicePosition();
	bool roisRectVec2roisPosRectVec(std::vector<std::vector<cv::Rect>>);
	static bool initImage(ALGO_IN_OCR*, cv::Mat&);
	void InitKeyPoints(std::vector<cv::KeyPoint> templateKeyPoint, cv::Mat TemplateDescriptor);
	void clean();
	void loadPara(cv::Mat& img, std::vector<cv::KeyPoint>& keypoints,
		cv::Mat& descriptor, std::vector<cv::Point2f>& chipPos,double);
	

private:
	bool refineKeyPoints
	(
		const std::vector<cv::KeyPoint>& queryKeypoints,
		const std::vector<cv::KeyPoint>& trainKeypoints,
		float threshold,
		std::vector<cv::DMatch>& matches,
		cv::Mat& matrix
	);

	double chipsAngle(cv::Rect detectChipRect, cv::Mat M);

	double calcAngle(cv::Point2f point1, cv::Point2f point2);
	void detectChip(cv::Mat img, std::vector<cv::Rect> &rectVec, int &chipNum);
	int calcThreshold(cv::Mat img);
	bool pointVecVec2rectVecVec(std::vector<std::vector<cv::Point2f>> pointVecVec,
		std::vector<std::vector<cv::Rect>> &rectVecVec);
	bool roiBinary();
	bool refineCharPos();
	void tuneCharRectPos(cv::Mat binaryImg, std::vector<cv::Rect>& rectVec);
	void verticalForgroundPixVal(cv::Mat img, cv::Rect &rect, int pixVal);
	int verPixNum(cv::Mat img, int xPos,int yMin, int yMax, int pixVal);
	int backGroundVal(cv::Mat binaryImg);
	void caliImgDetect();
	

public:
	cv::Mat m_imgTemplate;
	cv::Mat m_imgDetect;
	cv::Mat m_imgDetectCalied;

	std::vector<double> m_DchipAngleVec;
	int m_DchipNum;
	int m_TnumRoi;
	std::vector<cv::Point2f> m_TchipPosPointVec;
	std::vector<std::vector<cv::Rect>> m_Trois_rectVec;
	std::vector<std::vector<cv::Rect>> m_Drois_rectVec;
	std::vector<cv::Point2f> m_DchipPosPointVec;
	std::vector<cv::Rect> m_DroisPosRectVec;
	cv::Ptr<cv::ORB> m_orb;
	std::vector<cv::KeyPoint> m_templateKeyPoint;
	cv::Mat m_templateDescriptor;

private:
	std::vector<cv::DMatch> m_matches;
	cv::Mat m_refineMat;
	cv::Mat m_M; //mapping matrix
	cv::Point2f m_centerPoint;
	cv::Point2f m_refPoint;

	cv::Mat m_roiImgBlur;
	cv::Mat m_roiImgGray;
	cv::Mat m_roiImgThresh;
	cv::Mat m_templateGrayImg;
	cv::Mat m_detectGrayImg;
	double m_TrefAngle;//slant degree computed by the chip's top left point and top right point
	
	std::vector<cv::KeyPoint> m_detectKeyPoint;
	cv::Mat m_detectDescriptor;
	std::vector<cv::Mat> m_roiBinaryImgVec;
	std::vector<std::vector<int>> m_roisOffsetPosVec;
	//all of points of ROIs corresponding to under detection picture, each of point vector is the set of ROI points
	std::vector<std::vector<cv::Point2f>> m_Drois_pointVec;

	orb::ORBextractor extractor;

};
