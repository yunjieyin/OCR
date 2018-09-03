#pragma once
#ifdef OCRPROCESS_EXPORTS
#define OCRPROCESS_DLL __declspec(dllexport)
#else
#define OCRPROCESS_DLL __declspec(dllimport)
#endif

#include <opencv2/opencv.hpp>
#include <opencv.hpp>
#include <highgui.hpp>

#include "OCR_Kernel.h"
#include "..\inc\Segment.h"

#include "Proc_Kernel.h"

namespace OCRProcess
{
	//一个CSegmentProcess对应一张图片的处理
	class  OCRPROCESS_DLL  CSegmentProcess
	{
	public:
		CSegmentProcess();
		~CSegmentProcess();
#pragma region New
	private:
		ALGO_CFG_OCR *mptr_Ocr_CFG;
	protected:
		//ALGO_RES_OCR_SEGMENT m_OCR_ROI_SEGMENT[_PROC_ROI_MAX_COUNT];
	public:
		int SetBaseInfo(ALGO_CFG_OCR* _segmentcfg);
		//int RunCharImgageSegment(ALGO_IN_OCR* _segmentIn, std::vector< ALGO_RES_OCR_SEGMENT> &_segmentOut);
		int RunCharImgageSegment(ALGO_IN_OCR * _segmentIn, std::vector< ALGO_RES_OCR_OBJECT_SEGMENT> & _segmentOut);
#pragma endregion

	public:
		cv::Mat imgTemplate;
		cv::Mat imgObj;
		int offset_x;
		int offset_y;
		
		Segment* seg;//一张图的处理时，依次遍历不同的object，调用同一个segment进行运算

	};



}