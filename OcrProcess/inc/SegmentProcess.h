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
	//һ��CSegmentProcess��Ӧһ��ͼƬ�Ĵ���
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
		
		Segment* seg;//һ��ͼ�Ĵ���ʱ�����α�����ͬ��object������ͬһ��segment��������

	};



}