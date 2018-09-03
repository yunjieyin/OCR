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
#define LSMSG_OCR_START				(WM_USER + 1001)//�����㷨
#define LSMSG_OCR_END				(WM_USER + 1002)//ֹͣ�㷨
#define LSMSG_OCR_RECOGNIZE			(WM_USER + 1003)//ִ��ʶ��
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
	GRAY,			// �Ҷ�ͼ��
	YUV422,			// ����˳��Y0Y1����������U0-U1����������V0V1������
	YUV420,			// ����˳��Y0Y1����������U0-U1����������V0V1������
	RGB24,			// ����˳�� ��RGB������RGB��
	RGB565,
	RGB555,
	RGB32,
	YV12,			// ����˳��Y0Y1����������V0V1����������U0-U1������
	UYVY,			// ����˳��U0Y0V0Y1U2Y2V2Y3 ����������һ������λ��ͼ�����Ͻ�
	YUY2,			// ����˳��Y0 U0 Y1 V0 Y2 U2 Y3 V2������
	YUV444,			// ����˳��Y0Y1����������U0-U1����������V0V1������
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
typedef struct tagALGO_CFG_OCR_OBJECT_ROI {//һ��Ŀ�����ĳ��ģ��(ROI)

	//unsigned int			n_foreground;//�ַ���ɫ��ѡ�OCR_CHAR_FOREGROUND_XXX
	//unsigned int			n_background;//�ַ�����ɫ��ѡ�OCR_CHAR_BACKGROUND_XXX
	POINT					pos_foreground;
	POINT					pos_background;
	unsigned int			n_char_type = 0;//һ��ģ��(ROI)���ַ������͡�OCR_CHAR_TYPE_XXX
	unsigned int			n_char_derection = 0;//һ��ģ��(ROI)���ַ���clockwise��ת�ĽǶ�

	unsigned int			nbr_char;//һ��ROI�������ַ�������
	RECT					tbl_char_rect[_PROC_CHAR_MAX_COUNT];//һ��ģ��(ROI)�������ַ��ľ��ο�

	unsigned int			n_segment_caltype;//�ָ�ģ����߼�����ѡ��
	float					f_segment_max_slant_angle = 3;//the max slant angle of text region
	float					f_segment_angle_precision = 0.5;//the precision of angle
}ALGO_CFG_OCR_OBJECT_ROI;

typedef struct tagALGO_CFG_OCR_OBJECT {//һ��Ŀ��(OBJECT)

	unsigned int				nbr_vertices;//����εĶ�������
	POINT						sp_vertices_pos[_PROC_POLYGON_VERTEXES_MAX_COUNT];//����εĶ��㼯��

	unsigned int				nbr_roi;//ģ��(ROI)������
	ALGO_CFG_OCR_OBJECT_ROI		tbl_roi_data[_PROC_ROI_MAX_COUNT];//����ģ��(ROI)������
}ALGO_CFG_OCR_OBJECT;

typedef struct tagALGO_CFG_OCR {//һ��ͼƬ����

	char					img_path[256] = "E:\\charDir\\test1.bmp";
	SIZE					wh;//ͼƬ�ߴ�

	unsigned int			nbr_matcharea;//��������ĵ�����
	RECT					tbl_matcharea_rect[_PROC_MATCHAREA_MAX_COUNT];//�������������ˮƽ����

	unsigned int			nbr_object;//Ŀ��(OBJECT)������
	ALGO_CFG_OCR_OBJECT		tbl_object[_PROC_OBJECT_MAX_COUNT];//���е�Ŀ��(OBJECT)
}ALGO_CFG_OCR;
/*--------------------------------------------------------------------------*/
#pragma endregion

#pragma region IN
/*-------------------------------IN-------------------------------------------*/
typedef struct tagALGO_IN_OCR {

	char					img_path[256] = "E:\\charDir\\test1.bmp";
	BYTE					* ptr_bits;//ͼƬ
	SIZE					img_wh;//ͼƬ�ߴ�
	unsigned int			img_widthstep;//ÿ�д洢���п�
	unsigned int			img_ori;//ͼ�����ݴ洢�ķ���0������	1������
	unsigned int			img_type;//ͼƬ���ͣ�ö��ImageType

	bool					b_char_location = false;//���ַ�ѡ��Ƿ�λ���ַ���������Ӷ���ζ���λ��
	bool					b_char_recognize = false;//���ַ�ѡ��Ƿ�ʶ���ַ�
	char					b_rsv[2];
}ALGO_IN_OCR;

/*--------------------------------------------------------------------------*/
#pragma endregion

#pragma region RES
/*------------------------------RES-------------------------------------------*/

typedef struct tagALGO_RES_OCR_ROI {//һ��Ŀ�����ĳ��ģ��(ROI)

	unsigned int			nbr_recognize_char;//һ��ģ��(ROI)�ڵ�ʶ���ַ�char������
	RECT					tbl_recognize_char_rect[_PROC_CHAR_MAX_COUNT];//һ��ROI�������ַ���ˮƽ����

	char					tbl_recognize_char_text[_PROC_CHAR_MAX_COUNT][3];
	//unsigned int	nbr_text_vertexes;
	//POINT			sp_text_vertexes[_PROC_POLYGON_VERTEXES_MAX_COUNT];

}ALGO_RES_OCR_ROI;

typedef struct tagALGO_RES_OCR_OBJECT {//һ��Ŀ��(OBJECT)

	bool					b_ok;
	unsigned int			nbr_roi;//ģ��(ROI)������
	ALGO_RES_OCR_ROI		tbl_roi[_PROC_ROI_MAX_COUNT];//����ģ��(ROI)����

	//unsigned int	nbr_object_vertexes;
	//POINT			sp_object_vertexes[_PROC_POLYGON_VERTEXES_MAX_COUNT];
}ALGO_RES_OCR_OBJECT;

typedef struct tagALGO_RES_OCR {//һ��ͼƬ����

	bool					b_ok;
	unsigned int			nbr_object;//Ŀ�꣨OBJECT������
	ALGO_RES_OCR_OBJECT		tbl_object[_PROC_ROI_MAX_COUNT];//����Ŀ�꣨OBJECT������
}ALGO_RES_OCR;

/*--------------------------------------------------------------------------*/
#pragma endregion

///<summary>
// 
///</summary>
/// <param name="msg">��Ϣ����</param>
/// <param name="wParam">�㷨ͨ��</param>
/// <param name="lParam">����</param>
long ocr_algo_msg_proc(UINT msg, long long wParam, long long lParam);

long ocr_algo_data_proc(ALGO_TASK *sp_task);
#endif
