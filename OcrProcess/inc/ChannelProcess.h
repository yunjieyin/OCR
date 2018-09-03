#pragma once
#include <opencv2/opencv.hpp>
#include <opencv.hpp>
#include <highgui.hpp>

#include "OCR_Kernel.h"
#include "Proc_Kernel.h"
#include "ChannelResource.h"
#include "..\inc\Segment.h"
#include "..\inc\TemplateImg.h"


#define CALI_TMP 0
namespace OCRProcess
{
	class  CChannelProcess
	{
	public:
		CChannelProcess();
		~CChannelProcess();
	public:
		int SetBaseInfo(ALGO_CFG_OCR* _segmentcfg);
		int RunProcess(ALGO_IN_OCR * _segmentIn, std::vector< ALGO_RES_OCR_OBJECT_SEGMENT> & _segmentOut);

	private:
		ALGO_CFG_OCR *mptr_Ocr_CFG;
	public:
		TemplateImg* tempImg;

		openslam::slam::ORBextractor extractor;

		std::vector< ALGO_RES_OCR_OBJECT_SEGMENT>  ocr_out;
#pragma region Field
		cv::Mat imgROI;//原始图片的ROI
		cv::Mat binaryROI;//原始图片的ROI的二值图
		cv::Mat rotatedROI;//原始图片的ROI的二值图旋转图
		cv::Mat imgSrc;//原始图片
		cv::Mat imgSrc_copy;//原始图片的拷贝
		std::string middleImageDirPath = segmentDirPath;
		char dirPath_MiddleImage[256] = "C:\\";
		char dirPath_ResultImage[256] = "C:\\";
		bool issavemiddleimage;
#pragma endregion
	};
}

