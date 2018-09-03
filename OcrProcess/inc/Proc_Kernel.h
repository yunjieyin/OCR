#include <vector>
#include "OCR_Kernel.h"

#define FOREGROUND_VALUE 255
#define BACKGROUND_VALUE 0
#define SLICE_POS_DELTA 5
#define isoutname 0
#define segmentDirPath " E:\\OCR\\segment\\"

#include <opencv2/opencv.hpp>
#include <opencv.hpp>
#include <highgui.hpp>



typedef struct tagALGO_RES_OCR_ROI_SEGMENT {//ÿһ��ģ�壨ROI��

	unsigned int	nbr_char;//һ��ROI�ڵ��ַ�������
	RECT			rect[_PROC_CHAR_MAX_COUNT];//һ��ROI�ڵ�ÿ���ַ���ˮƽ����
	unsigned int	nbr_text_vertexes;
	POINT			sp_text_vertexes[_PROC_POLYGON_VERTEXES_MAX_COUNT];//һ��ROI���ַ�����Ӷ���ζ�������
}ALGO_RES_OCR_ROI_SEGMENT;
typedef struct tagALGO_RES_OCR_OBJECT_SEGMENT {///һ��Ŀ��

	unsigned int	nbr_roi;;//ROI����
	std::vector<ALGO_RES_OCR_ROI_SEGMENT> tbl_roi;

	unsigned int	nbr_object_vertexes;
	POINT			sp_object_vertexes[_PROC_POLYGON_VERTEXES_MAX_COUNT];//һ��Object����Ӷ���ζ�������

}ALGO_RES_OCR_OBJECT_SEGMENT;
