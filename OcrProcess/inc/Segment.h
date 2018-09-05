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
	cv::Mat imgTemplate;
	cv::Mat imgDetect;
	cv::Mat imgDetectCalied;

	std::vector<double> D_chipAngleVec;
	int D_chipNum;
	int T_numRoi;
	std::vector<cv::Point2f> T_chipPosPointVec;
	std::vector<std::vector<cv::Rect>> T_rois_rectVec;
	std::vector<std::vector<cv::Rect>> D_rois_rectVec;
	std::vector<cv::Point2f> D_chipPosPointVec;
	std::vector<cv::Rect> D_roisPosRectVec;
	cv::Ptr<cv::ORB> orb2;
	std::vector<cv::KeyPoint> templateKeyPoint;
	cv::Mat TemplateDescriptor;

private:
	std::vector<cv::DMatch> matches;
	cv::Mat refineMat;
	cv::Mat M; //mapping matrix
	cv::Point2f centerPoint;
	cv::Point2f refPoint;

	cv::Mat roiImgBlur;
	cv::Mat roiImgGray;
	cv::Mat roiImgThresh;
	cv::Mat templateGrayImg;
	cv::Mat detectGrayImg;
	double T_refAngle;//slant degree computed by the chip's top left point and top right point
	
	std::vector<cv::KeyPoint> detectKeyPoint;
	cv::Mat detectDescriptor;
	std::vector<cv::Mat> roiBinaryImgVec;
	std::vector<std::vector<int>> roisOffsetPosVec;
	//all of points of ROIs corresponding to under detection picture, each of point vector is the set of ROI points
	std::vector<std::vector<cv::Point2f>> D_rois_pointVec;

	orb::ORBextractor extractor;

};
