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



typedef struct tagALGO_RES_OCR_ROI_SEGMENT {//每一个模板（ROI）

	unsigned int	nbr_char;//一个ROI内的字符的数量
	RECT			rect[_PROC_CHAR_MAX_COUNT];//一个ROI内的每个字符的水平矩形
	unsigned int	nbr_text_vertexes;
	POINT			sp_text_vertexes[_PROC_POLYGON_VERTEXES_MAX_COUNT];//一个ROI内字符的外接多边形顶点坐标
}ALGO_RES_OCR_ROI_SEGMENT;
typedef struct tagALGO_RES_OCR_OBJECT_SEGMENT {///一个目标

	unsigned int	nbr_roi;;//ROI数量
	std::vector<ALGO_RES_OCR_ROI_SEGMENT> tbl_roi;

	unsigned int	nbr_object_vertexes;
	POINT			sp_object_vertexes[_PROC_POLYGON_VERTEXES_MAX_COUNT];//一个Object的外接多边形顶点坐标

}ALGO_RES_OCR_OBJECT_SEGMENT;
