#ifndef _OCR_Kernel_H
#define _OCR_Kernel_H
/*
LastUpdate:2018-6-4 11:59:26
*/

//#include<windef.h>
#pragma region typedef
/*--------------------------------typedef------------------------------------------*/

typedef unsigned char       BYTE;
typedef long				LONG;
typedef unsigned int        UINT;
typedef struct tagSIZE
{
	LONG        cx;
	LONG        cy;
} SIZE;

typedef struct tagPOINT
{
	LONG  x;
	LONG  y;
} POINT;

typedef struct tagRECT
{
	LONG    left;
	LONG    top;
	LONG    right;
	LONG    bottom;
} RECT;
typedef unsigned int        UINT;
//typedef _W64 unsigned int UINT_PTR;
//typedef UINT_PTR            WPARAM;
//typedef _W64 long LONG_PTR;
//typedef LONG_PTR            LPARAM;
//typedef LONG_PTR            LRESULT;
/*---------------------------------------------------------------------------------*/
#pragma endregion



#pragma region CONST
#define _PROC_CHAR_BYTELENGTH				2
#define _PROC_CHAR_MAX_COUNT				64
#define	_PROC_POLYGON_VERTEXES_MAX_COUNT	8
#define _PROC_OBJECT_MAX_COUNT				8
#define _PROC_ROI_MAX_COUNT					8
#define _PROC_MATCHAREA_MAX_COUNT			8
#pragma endregion


#pragma region Message
#define WM_USER						0x0400
#define LSMSG_OCR_START				(WM_USER + 1001)//启动算法
#define LSMSG_OCR_END				(WM_USER + 1002)//停止算法
#define LSMSG_OCR_RECOGNIZE			(WM_USER + 1003)//执行识别
#pragma endregion


#pragma region ENUMOPTION
#define OCR_CHAR_FOREGROUND_WHITE		0x01;
#define OCR_CHAR_FOREGROUND_BLACK		0x02;

#define OCR_CHAR_BACKGROUND_WHITE		0x01;
#define OCR_CHAR_BACKGROUND_BLACK		0x02;

#define OCR_SEGMENT_OUTTYPE_NONE		0x00;
#define OCR_SEGMENT_OUTTYPE_IMGPATH		0x01;
#define OCR_SEGMENT_OUTTYPE_DIRPATH		0x02;

#define	OCR_CHAR_TYPE_SINGLE_NUMBER					0x01
#define	OCR_CHAR_TYPE_SINGLE_ALPHABET_EN_LOW		0x02
#define	OCR_CHAR_TYPE_SINGLE_ALPHABET_EN_UP			0x04
#define	OCR_CHAR_TYPE_SINGLE_SPECIFICSYMBOL1		0x08
#define	OCR_CHAR_TYPE_SINGLE_SPECIFICSYMBOL2		0x016

enum ImageType
{
	GRAY,			// 灰度图像
	YUV422,			// 排列顺序“Y0Y1……”、“U0-U1……”、“V0V1……”
	YUV420,			// 排列顺序“Y0Y1……”、“U0-U1……”、“V0V1……”
	RGB24,			// 排列顺序 “RGB”、“RGB”
	RGB565,
	RGB555,
	RGB32,
	YV12,			// 排列顺序“Y0Y1……”、“V0V1……”、“U0-U1……”
	UYVY,			// 排列顺序“U0Y0V0Y1U2Y2V2Y3 ……”，第一个像素位于图像左上角
	YUY2,			// 排列顺序“Y0 U0 Y1 V0 Y2 U2 Y3 V2……”
	YUV444,			// 排列顺序“Y0Y1……”、“U0-U1……”、“V0V1……”
};
#pragma endregion

#pragma region Sub Struct

typedef struct tagAlgoTask {

	unsigned char		index;
	unsigned int		id_task;//Message
	BYTE				* ptr_in;
	BYTE				* ptr_out;
}ALGO_TASK;

#pragma endregion


#pragma region CFG
/*--------------------------------CFG------------------------------------------*/
typedef struct tagALGO_CFG_OCR_OBJECT_ROI {//一个目标里的某个模板(ROI)

	//unsigned int			n_foreground;//字符颜色的选项。OCR_CHAR_FOREGROUND_XXX
	//unsigned int			n_background;//字符背景色的选项。OCR_CHAR_BACKGROUND_XXX
	POINT					pos_foreground;
	POINT					pos_background;
	unsigned int			n_char_type = 0;//一个模板(ROI)内字符的类型。OCR_CHAR_TYPE_XXX
	unsigned int			n_char_derection = 0;//一个模板(ROI)内字符按clockwise旋转的角度

	unsigned int			nbr_char;//一个ROI内所有字符的数量
	RECT					tbl_char_rect[_PROC_CHAR_MAX_COUNT];//一个模板(ROI)内所有字符的矩形框

	unsigned int			n_segment_caltype;//分割模块的逻辑处理选项
	float					f_segment_max_slant_angle = 3;//the max slant angle of text region
	float					f_segment_angle_precision = 0.5;//the precision of angle
}ALGO_CFG_OCR_OBJECT_ROI;

typedef struct tagALGO_CFG_OCR_OBJECT {//一个目标(OBJECT)

	unsigned int				nbr_vertices;//多边形的顶点数量
	POINT						sp_vertices_pos[_PROC_POLYGON_VERTEXES_MAX_COUNT];//多边形的顶点集合

	unsigned int				nbr_roi;//模板(ROI)的数量
	ALGO_CFG_OCR_OBJECT_ROI		tbl_roi_data[_PROC_ROI_MAX_COUNT];//所有模板(ROI)的数据
}ALGO_CFG_OCR_OBJECT;

typedef struct tagALGO_CFG_OCR {//一个图片对象

	char					img_path[256] = "E:\\charDir\\test1.bmp";
	SIZE					wh;//图片尺寸

	unsigned int			nbr_matcharea;//特征区域的的数量
	RECT					tbl_matcharea_rect[_PROC_MATCHAREA_MAX_COUNT];//所有特征区域的水平矩阵

	unsigned int			nbr_object;//目标(OBJECT)的数量
	ALGO_CFG_OCR_OBJECT		tbl_object[_PROC_OBJECT_MAX_COUNT];//所有的目标(OBJECT)
}ALGO_CFG_OCR;
/*--------------------------------------------------------------------------*/
#pragma endregion

#pragma region IN
/*-------------------------------IN-------------------------------------------*/
typedef struct tagALGO_IN_OCR {

	char					img_path[256] = "E:\\charDir\\test1.bmp";
	BYTE					* ptr_bits;//图片
	SIZE					img_wh;//图片尺寸
	unsigned int			img_widthstep;//每行存储的行宽
	unsigned int			img_ori;//图形数据存储的方向，0：左上	1：左下
	unsigned int			img_type;//图片类型，枚举ImageType

	bool					b_char_location = false;//单字符选项。是否定位单字符，返回外接多边形顶点位置
	bool					b_char_recognize = false;//单字符选项。是否识别单字符
	char					b_rsv[2];
}ALGO_IN_OCR;

/*--------------------------------------------------------------------------*/
#pragma endregion

#pragma region RES
/*------------------------------RES-------------------------------------------*/

typedef struct tagALGO_RES_OCR_ROI {//一个目标里的某个模板(ROI)

	unsigned int			nbr_recognize_char;//一个模板(ROI)内的识别字符char的数量
	RECT					tbl_recognize_char_rect[_PROC_CHAR_MAX_COUNT];//一个ROI内所有字符的水平矩形

	char					tbl_recognize_char_text[_PROC_CHAR_MAX_COUNT][3];
	//unsigned int	nbr_text_vertexes;
	//POINT			sp_text_vertexes[_PROC_POLYGON_VERTEXES_MAX_COUNT];

}ALGO_RES_OCR_ROI;

typedef struct tagALGO_RES_OCR_OBJECT {//一个目标(OBJECT)

	bool					b_ok;
	unsigned int			nbr_roi;//模板(ROI)的数量
	ALGO_RES_OCR_ROI		tbl_roi[_PROC_ROI_MAX_COUNT];//所有模板(ROI)集合

	//unsigned int	nbr_object_vertexes;
	//POINT			sp_object_vertexes[_PROC_POLYGON_VERTEXES_MAX_COUNT];
}ALGO_RES_OCR_OBJECT;

typedef struct tagALGO_RES_OCR {//一个图片对象

	bool					b_ok;
	unsigned int			nbr_object;//目标（OBJECT）数量
	ALGO_RES_OCR_OBJECT		tbl_object[_PROC_ROI_MAX_COUNT];//所有目标（OBJECT）集合
}ALGO_RES_OCR;

/*--------------------------------------------------------------------------*/
#pragma endregion

///<summary>
// 
///</summary>
/// <param name="msg">消息类型</param>
/// <param name="wParam">算法通道</param>
/// <param name="lParam">输入</param>
long ocr_algo_msg_proc(UINT msg, long long wParam, long long lParam);

long ocr_algo_data_proc(ALGO_TASK *sp_task);
#endif
