#include "stdafx.h"
#include "..\inc\SegmentProcess.h"

#include <time.h>
using namespace std;



std::vector<cv::Point2f> chipPosPointVec(4);
OCRProcess::CSegmentProcess::CSegmentProcess()
{
	mptr_Ocr_CFG = new ALGO_CFG_OCR;
	memset(mptr_Ocr_CFG, 0, sizeof(ALGO_CFG_OCR));
	seg = new Segment();
	seg->orb1 = cv::ORB::create(FEATURE_POINTS_NUM, 1.2, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31);
	seg->orb2 = cv::ORB::create(FEATURE_POINTS_NUM, 1.2, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31);
}

OCRProcess::CSegmentProcess::~CSegmentProcess()
{
	if (mptr_Ocr_CFG != 0) { delete mptr_Ocr_CFG;mptr_Ocr_CFG = 0; }
	if (seg != 0) 
	{
		//seg->orb1。。。。。release
		delete seg; seg = 0;
	}
}



#pragma region Field
static int indexImg = 0;
cv::Mat imgROI;//原始图片的ROI
cv::Mat binaryROI;//原始图片的ROI的二值图
cv::Mat rotatedROI;//原始图片的ROI的二值图旋转图
cv::Mat imgSrc;//原始图片
cv::Mat imgSrc_copy;//原始图片的拷贝
#define segmentDirPath " E:\\OCR\\segment\\"
std::string middleImageDirPath= segmentDirPath;
char dirPath_MiddleImage[256] = "C:\\";
char dirPath_ResultImage[256] = "C:\\";
bool issavemiddleimage;
#pragma endregion


int OCRProcess::CSegmentProcess::SetBaseInfo(ALGO_CFG_OCR * _segmentcfg)
{
	int s1 = sizeof(ALGO_CFG_OCR);
	int s2 = sizeof(*_segmentcfg);
	int s3 = sizeof(*mptr_Ocr_CFG);
	memcpy(mptr_Ocr_CFG, _segmentcfg, sizeof(ALGO_CFG_OCR));

	///
	imgTemplate = cv::imread(mptr_Ocr_CFG->img_path);//sample picture

	for (int _index_object = 0; _index_object < mptr_Ocr_CFG->nbr_object; _index_object++)
	{
		//use rectangle match polygon start
		int objContPointNum = mptr_Ocr_CFG->tbl_object[_index_object].nbr_vertices;
		int xMin = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[0].x;
		int xMax = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[0].x;
		int yMin = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[0].y;
		int yMax = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[0].y;
		for (int i = 1; i < objContPointNum; ++i)
		{
			if (mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].x > xMax)
				xMax = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].x;

			if (mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].x < xMin)
				xMin = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].x;

			if (mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].y > yMax)
				yMax = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].y;

			if (mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].y < yMin)
				yMin = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].y;
		}

		cv::Point2f point1, point2, point3, point4;
		point1.x = xMin;
		point1.y = yMin;

		point2.x = xMax;
		point2.y = yMin;

		point3.x = xMax;
		point3.y = yMax;

		point4.x = xMin;
		point4.y = yMax;

		if (xMin < 0 || yMin < 0 || xMax > imgTemplate.cols || yMax > imgTemplate.rows || xMax <= xMin || yMax <= yMin)
		{
			return 2; //template picture's position points(4) not valid
		}

		//use rectangle match polygon end

		imgObj = imgTemplate(cv::Range(yMin, yMax), cv::Range(xMin, xMax));// template picture matched the polygon

		offset_x = xMin;
		offset_y = yMin;

		// template chip position point vector
		
		point1.x -= offset_x;
		point1.y -= offset_y;
		point2.x -= offset_x;
		point2.y -= offset_y;
		point3.x -= offset_x;
		point3.y -= offset_y;
		point4.x -= offset_x;
		point4.y -= offset_y;
		chipPosPointVec[0] = point1;
		chipPosPointVec[1] = point2;
		chipPosPointVec[2] = point3;
		chipPosPointVec[3] = point4;		
	}

	//Segment::calTemplateORB(imgObj);

	seg->InitTemplate(imgObj, chipPosPointVec);
	
	return 1;
}

//int OCRProcess::CSegmentProcess::RunCharImgageSegment(ALGO_IN_OCR * _segmentIn,std::vector< ALGO_RES_OCR_SEGMENT> & _segmentOut)
//{
//	memset(&_segmentOut, 0, sizeof(_segmentOut));
//#pragma region 创建可以输出的文件夹目录
//	//{
//	//	std::string dir;
//	//	time_t now = time(NULL);
//	//	const tm *_time = localtime(&now);
//
//	//	dir = segmentDirPath;
//	//	strftime(dirPath_MiddleImage, sizeof(dirPath_MiddleImage), dir.append("%Y-%m-%d_%H-%M-%S\\_middle\\").c_str(), _time);//将获得的时间路径存入wenjian
//	//	dir = segmentDirPath;
//	//	strftime(dirPath_ResultImage, sizeof(dirPath_ResultImage), dir.append("%Y-%m-%d_%H-%M-%S\\_result\\").c_str(), _time);//将获得的时间路径存入wenjian
//	//	char cmdchar[256];
//	//	sprintf(cmdchar, "md %s", dirPath_MiddleImage);//将创建文件的命令存入cmdchar中
//	//	system(cmdchar);//创建文件夹
//	//	sprintf(cmdchar, "md %s", dirPath_ResultImage);//将创建文件的命令存入cmdchar中
//	//	system(cmdchar);//创建文件夹
//	//}
//#pragma endregion
//	///数据源：mptr_Ocr_CFG、_segmentIn
//	///输出对象：_segmentOut
//	///已定义的参数
//	///dirPath_MiddleImage：中间结果的文件夹目录
//	///dirPath_ResultImage：最终结果的文件夹目录
//
//
//	for (int _index_object = 0; _index_object < mptr_Ocr_CFG->nbr_object; _index_object++)//遍历每个目标Object=>chip
//	{
//		int _nbr_vertices = mptr_Ocr_CFG->tbl_object[_index_object].nbr_vertices;
//			
//		//START
//		//Segment seg();
//
//		//END
//
//
//		cv::Mat img_1 = cv::imread(mptr_Ocr_CFG->img_path);//sample picture
//		cv::Mat img_2 = cv::imread(_segmentIn->img_path);// under detectoin picture
//
//		if (img_1.empty() || img_2.empty())
//		{
//			return 1; //sample picture or test picture is empty
//		}
//
//		//use rectangle match polygon start
//		int objContPointNum = mptr_Ocr_CFG->tbl_object[_index_object].nbr_vertices;
//		int xMin = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[0].x;
//		int xMax = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[0].x;
//		int yMin = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[0].y;
//		int yMax = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[0].y;
//		for (int i = 1; i < objContPointNum; ++i)
//		{
//			if (mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].x > xMax)
//				xMax = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].x;
//
//			if (mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].x < xMin)
//				xMin = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].x;
//
//			if (mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].y > yMax)
//				yMax = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].y;
//
//			if (mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].y < yMin)
//				yMin = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[i].y;
//		}
//
//		cv::Point2f point1, point2, point3, point4;
//		point1.x = xMin;
//		point1.y = yMin;
//
//		point2.x = xMax;
//		point2.y = yMin;
//
//		point3.x = xMax;
//		point3.y = yMax;
//
//		point4.x = xMin;
//		point4.y = yMax;
//
//		if (xMin < 0 || yMin < 0 || xMax > img_1.cols || yMax > img_1.rows || xMax <= xMin || yMax <= yMin)
//		{
//			return 2; //template picture's position points(4) not valid
//		}
//
//		//use rectangle match polygon end
//
//		cv::Mat imgObj = img_1(cv::Range(yMin, yMax),cv::Range(xMin, xMax));// template picture matched the polygon
//					
//		int offset_x = xMin;
//		int offset_y = yMin;
//
//		// template chip position point vector
//		std::vector<cv::Point2f> chipPosPointVec(4);
//		point1.x -= offset_x;
//		point1.y -= offset_y;
//		point2.x -= offset_x;
//		point2.y -= offset_y;
//		point3.x -= offset_x;
//		point3.y -= offset_y;
//		point4.x -= offset_x;
//		point4.y -= offset_y;
//		chipPosPointVec[0] = point1;
//		chipPosPointVec[1] = point2;
//		chipPosPointVec[2] = point3;
//		chipPosPointVec[3] = point4;
//
//
//		Segment seg(imgObj, img_2, chipPosPointVec);
//
//		#pragma region 处理每个目标
//		seg->T_numRoi = mptr_Ocr_CFG->tbl_object[_index_object].nbr_roi;
//		seg->T_rois_rectVec.clear();
//
//		for (int _index_Temp = 0; _index_Temp < mptr_Ocr_CFG->tbl_object[_index_object].nbr_roi; _index_Temp++)//遍历每个模板ROI
//		{
//			#pragma region T_rois_rectVec
//			std::vector<cv::Rect> rectVec;
//			rectVec.clear();
//			for (int _index_charlist = 0; _index_charlist < mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].nbr_char; _index_charlist ++)
//			{
//				cv::Rect rect;
//				rect.x = mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].left;
//				rect.y = mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].top;
//				rect.width = (
//					mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].right-
//					mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].left);
//				rect.height = (
//					mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].bottom-
//					mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].top);
//
//				rect.x = rect.x-offset_x;
//				rect.y = rect.y - offset_y;
//
//				if (rect.x < 0 || rect.y < 0 || rect.width <= 0 || rect.height <= 0)
//				{
//					return 3;
//				}
//
//				rectVec.push_back(rect);
//			}
//			seg->T_rois_rectVec.push_back(rectVec);
//			seg->T_numRoi = seg->T_rois_rectVec.size();
//			#pragma endregion	
//
//		}
//
//
//		//test start
//		if (0)
//		{
//			std::vector<std::vector<cv::Rect>> roiVec;
//			std::vector<cv::Rect> rectVec, rectVec1;
//
//			cv::Rect rect;
//			rect.x = seg->imgTemplate.cols / 4;
//			rect.y = seg->imgTemplate.rows / 4;
//			rect.width = 30;
//			rect.height = 30;
//
//			cv::Rect rect1;
//			rect1.x = seg->imgTemplate.cols / 3;
//			rect1.y = seg->imgTemplate.rows / 3;
//			rect1.width = 30;
//			rect1.height = 30;
//
//			//cv::circle(seg->imgTemplate, cv::Point(rect.x, rect.y), 3, cv::Scalar(0, 0, 255), 3);
//
//			rectVec.push_back(rect);
//			rectVec1.push_back(rect1);
//
//			roiVec.push_back(rectVec);
//			roiVec.push_back(rectVec1);
//			
//		}
//		//test end
//		
//
//		if (!seg->slicePosition())
//		{
//			return 4; //template picture don't match with test picture
//		}
//
//
//		//output the mapped rect to file	
//		for (int D_chipIndex = 0; D_chipIndex < seg->D_chipNum; ++D_chipIndex)
//		{
//			ALGO_RES_OCR_SEGMENT outResult;
//			_segmentOut.push_back(outResult);
//
//			int count = 0;
//			for (int i = 0; i < seg->T_numRoi; ++i)
//			{
//				std::vector<cv::Rect> roiRecVec = seg->D_rois_rectVec[count++];
//				int rectIndex = 0;
//				for (; rectIndex < roiRecVec.size(); ++rectIndex)
//				{
//					/*all of the rect value of any ROI*/
//					RECT rect;
//					rect.left = roiRecVec[rectIndex].x;
//					rect.right = rect.left + roiRecVec[rectIndex].width;
//					rect.top = roiRecVec[rectIndex].y;
//					rect.bottom = rect.top + roiRecVec[rectIndex].height;
//
//					/*assign outResult's rect member value*/
//					_segmentOut[D_chipIndex].rect[rectIndex] = rect;			
//				}
//
//					/*assign outResult's nbr_obj member value*/
//				_segmentOut[D_chipIndex].nbr_char = rectIndex ;
//
//					/*assign outResult's vertexes member value*/
//				int vertexesIndex = 0;
//				for (int j = 0; j < seg->D_roisPosRectVec.size(); ++j)
//				{
//					int x = seg->D_roisPosRectVec[j].x;
//					int y = seg->D_roisPosRectVec[j].y;	
//					_segmentOut[D_chipIndex].sp_text_vertexes[vertexesIndex].x = x;
//					_segmentOut[D_chipIndex].sp_text_vertexes[vertexesIndex].y = y;
//					vertexesIndex += 1;
//
//					x = seg->D_roisPosRectVec[j].x + seg->D_roisPosRectVec[j].width;
//					y = seg->D_roisPosRectVec[j].y;
//					_segmentOut[D_chipIndex].sp_text_vertexes[vertexesIndex].x = x;
//					_segmentOut[D_chipIndex].sp_text_vertexes[vertexesIndex].y = y;
//					vertexesIndex += 1;
//
//					x = seg->D_roisPosRectVec[j].x + seg->D_roisPosRectVec[j].width;
//					y = seg->D_roisPosRectVec[j].y + seg->D_roisPosRectVec[j].height;
//					_segmentOut[D_chipIndex].sp_text_vertexes[vertexesIndex].x = x;
//					_segmentOut[D_chipIndex].sp_text_vertexes[vertexesIndex].y = y;
//					vertexesIndex += 1;
//
//					x = seg->D_roisPosRectVec[j].x;
//					y = seg->D_roisPosRectVec[j].y + seg->D_roisPosRectVec[j].height;
//					_segmentOut[D_chipIndex].sp_text_vertexes[vertexesIndex].x = x;
//					_segmentOut[D_chipIndex].sp_text_vertexes[vertexesIndex].y = y;
//					vertexesIndex += 1;
//				}
//			
//				/*assign outResult's nbr_text_vertexes member value*/
//				_segmentOut[D_chipIndex].nbr_text_vertexes = 4;
//			}
//			
//			/*assign outResult's nbr_object_vertexes member value*/
//			_segmentOut[D_chipIndex].nbr_object_vertexes = 4;
//			
//			/*assign outResult's sp_object_vertexes member value*/
//			for (int chipPosPointIndex = 0; chipPosPointIndex < seg->D_chipPosPointVec.size(); ++chipPosPointIndex)
//			{
//				cv::Point2f point = seg->D_chipPosPointVec[chipPosPointIndex];
//				_segmentOut[D_chipIndex].sp_object_vertexes[chipPosPointIndex].x = point.x;
//				_segmentOut[D_chipIndex].sp_object_vertexes[chipPosPointIndex].y = point.y;
//
//			}
//	
//		}
//
//		//seg->roisRectVec2roisPosRectVec(seg->D_rois_rectVec);
//		//cv::imshow("detectImg", seg->imgDetect);
//		//cv::imshow("templateImg", img_1);
//
//	}
//#pragma endregion
//
//
//
//	return 0;
//}
//


int OCRProcess::CSegmentProcess::RunCharImgageSegment(ALGO_IN_OCR * _segmentIn, std::vector< ALGO_RES_OCR_OBJECT_SEGMENT> & _segmentOut)
{
	memset(&_segmentOut, 0, sizeof(_segmentOut));
#pragma region 创建可以输出的文件夹目录
	//{
	//	std::string dir;
	//	time_t now = time(NULL);
	//	const tm *_time = localtime(&now);

	//	dir = segmentDirPath;
	//	strftime(dirPath_MiddleImage, sizeof(dirPath_MiddleImage), dir.append("%Y-%m-%d_%H-%M-%S\\_middle\\").c_str(), _time);//将获得的时间路径存入wenjian
	//	dir = segmentDirPath;
	//	strftime(dirPath_ResultImage, sizeof(dirPath_ResultImage), dir.append("%Y-%m-%d_%H-%M-%S\\_result\\").c_str(), _time);//将获得的时间路径存入wenjian
	//	char cmdchar[256];
	//	sprintf(cmdchar, "md %s", dirPath_MiddleImage);//将创建文件的命令存入cmdchar中
	//	system(cmdchar);//创建文件夹
	//	sprintf(cmdchar, "md %s", dirPath_ResultImage);//将创建文件的命令存入cmdchar中
	//	system(cmdchar);//创建文件夹
	//}
#pragma endregion
	///数据源：mptr_Ocr_CFG、_segmentIn
	///输出对象：_segmentOut
	///已定义的参数
	///dirPath_MiddleImage：中间结果的文件夹目录
	///dirPath_ResultImage：最终结果的文件夹目录

	double consumetime;
	double startTime;
	double endTime;

	cv::Mat imgDetect;

	
	if (!Segment::initImage(_segmentIn, imgDetect))
	{
		return 5; //initilize under detection image failed;
	}

	if (imgTemplate.empty() || imgDetect.empty())
	{
		return 1; //sample picture or test picture is empty
	}

	for (int _index_object = 0; _index_object < mptr_Ocr_CFG->nbr_object; _index_object++)//遍历每个目标Object=>chip
	{
		seg->InitDetect( imgDetect);
#pragma region 处理每个目标
		seg->T_numRoi = mptr_Ocr_CFG->tbl_object[_index_object].nbr_roi;
		seg->T_rois_rectVec.clear();

		for (int _index_Temp = 0; _index_Temp < mptr_Ocr_CFG->tbl_object[_index_object].nbr_roi; _index_Temp++)//遍历每个模板ROI
		{
#pragma region T_rois_rectVec

			int charNum = mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].nbr_char;
			std::vector<cv::Rect> rectVec(charNum);
			for (int _index_charlist = 0; _index_charlist < mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].nbr_char; _index_charlist++)
			{
				cv::Rect rect;
				rect.x = mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].left;
				rect.y = mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].top;
				rect.width = (
					mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].right -
					mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].left);
				rect.height = (
					mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].bottom -
					mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].top);

				rect.x = rect.x - offset_x;
				rect.y = rect.y - offset_y;

				if (rect.x < 0 || rect.y < 0 || rect.width <= 0 || rect.height <= 0)
				{
					return 3;
				}

				rectVec[_index_charlist] = rect;
			}
			seg->T_rois_rectVec.push_back(rectVec);
#pragma endregion	

		}
		if (DEBUG)
		{
			startTime = cv::getTickCount();
		}

		if (!seg->slicePosition())
		{
			return 4; //template picture don't match with test picture
		}
		
		if (DEBUG)
		{
			endTime = cv::getTickCount();
			consumetime = (double)((endTime - startTime) / cv::getTickFrequency()) * 1000;
		}		

		//output the mapped rect to file	
		for (int D_chipIndex = 0; D_chipIndex < seg->D_chipNum; ++D_chipIndex)
		{
			ALGO_RES_OCR_OBJECT_SEGMENT outResult;
			_segmentOut.push_back(outResult);
			_segmentOut[D_chipIndex].nbr_roi = seg->T_numRoi;

			//all of member of rois in a object
			for (int i = 0; i < seg->T_numRoi; ++i)
			{
				std::vector<cv::Rect> roiRecVec = seg->D_rois_rectVec[i];
				ALGO_RES_OCR_ROI_SEGMENT ROIdata;

				_segmentOut[D_chipIndex].tbl_roi.push_back(ROIdata);
				_segmentOut[D_chipIndex].tbl_roi[i].nbr_char = roiRecVec.size();

				int rectIndex = 0;
				for (; rectIndex < roiRecVec.size(); ++rectIndex)
				{
					/*all of the rect value of any ROI*/
					RECT rect;
					rect.left = roiRecVec[rectIndex].x;
					rect.right = rect.left + roiRecVec[rectIndex].width;
					rect.top = roiRecVec[rectIndex].y;
					rect.bottom = rect.top + roiRecVec[rectIndex].height;

					/*assign outResult's rect member value*/
					_segmentOut[D_chipIndex].tbl_roi[i].rect[rectIndex] = rect;
				}

				/*assign outResult's vertexes member value*/	
				int vertexesIndex = 0;
				int x = seg->D_roisPosRectVec[i].x;
				int y = seg->D_roisPosRectVec[i].y;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].x = x;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].y = y;
				vertexesIndex += 1;

				x = seg->D_roisPosRectVec[i].x + seg->D_roisPosRectVec[i].width;
				y = seg->D_roisPosRectVec[i].y;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].x = x;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].y = y;
				vertexesIndex += 1;

				x = seg->D_roisPosRectVec[i].x + seg->D_roisPosRectVec[i].width;
				y = seg->D_roisPosRectVec[i].y + seg->D_roisPosRectVec[i].height;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].x = x;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].y = y;
				vertexesIndex += 1;

				x = seg->D_roisPosRectVec[i].x;
				y = seg->D_roisPosRectVec[i].y + seg->D_roisPosRectVec[i].height;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].x = x;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].y = y;
				vertexesIndex += 1;

				/*assign outResult's nbr_text_vertexes member value*/
				_segmentOut[D_chipIndex].tbl_roi[i].nbr_text_vertexes = 4;

			}

			/*assign outResult's nbr_object_vertexes member value*/
			_segmentOut[D_chipIndex].nbr_object_vertexes = 4;
	
			/*assign outResult's sp_object_vertexes member value*/
			for (int chipPosPointIndex = 0; chipPosPointIndex < seg->D_chipPosPointVec.size(); ++chipPosPointIndex)
			{
				cv::Point2f point = seg->D_chipPosPointVec[chipPosPointIndex];
				_segmentOut[D_chipIndex].sp_object_vertexes[chipPosPointIndex].x = point.x;
				_segmentOut[D_chipIndex].sp_object_vertexes[chipPosPointIndex].y = point.y;

			}

		}

	}
	seg->roiBinaryImgVec.clear();
	
#pragma endregion

	if (DEBUG)
	{
		ofstream f1("d:\\1.txt", ios::app);
		if (!f1) return -1;//打开文件失败则结束运行
		{
			time_t now = time(NULL);
			const tm *_time = localtime(&now);
			char d1[256];
			string dir;
			strftime(d1, sizeof(d1), dir.append("%Y-%m-%d %H-%M-%S").c_str(), _time);//将获得的时间路径存入wenjian
			f1 << d1 << setw(20) << _segmentIn->img_path << setw(20) << "times：" << consumetime << endl;
			//f1 << d1 << setw(20) << "times：" << consumetime << endl;
		}
		f1.close();//关闭文件
	}
	

	return 0;
}
