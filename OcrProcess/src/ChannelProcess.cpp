#include "stdafx.h"
#include "..\inc\ChannelProcess.h"
#include <time.h>


using namespace std;

OCRProcess::CChannelProcess::CChannelProcess()
{
	mptr_Ocr_CFG = new ALGO_CFG_OCR;
	memset(mptr_Ocr_CFG, 0, sizeof(ALGO_CFG_OCR));
	tempImg = new TemplateImg();
}


OCRProcess::CChannelProcess::~CChannelProcess()
{
	if (mptr_Ocr_CFG != 0) { delete mptr_Ocr_CFG;mptr_Ocr_CFG = 0; }

	if (tempImg != 0)
	{
		delete tempImg;
		tempImg = 0;
	}

}

int OCRProcess::CChannelProcess::SetBaseInfo(ALGO_CFG_OCR * _segmentcfg)
{
	int s1 = sizeof(ALGO_CFG_OCR);
	int s2 = sizeof(*_segmentcfg);
	int s3 = sizeof(*mptr_Ocr_CFG);
	memcpy(mptr_Ocr_CFG, _segmentcfg, sizeof(ALGO_CFG_OCR));
	//OCRProcess::CChannelResource::gmp_imgTemplate = cv::imread(mptr_Ocr_CFG->img_path);
	///
	tempImg->CT_imgTemplate = cv::imread(mptr_Ocr_CFG->img_path);//sample picture

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

		if (xMin < 0 || yMin < 0 || xMax > tempImg->CT_imgTemplate.cols || yMax > tempImg->CT_imgTemplate.rows || xMax <= xMin || yMax <= yMin)
		{
			return 2; //template picture's position points(4) not valid
		}

		//use rectangle match polygon end

		tempImg->CT_imgObj = tempImg->CT_imgTemplate(cv::Range(yMin, yMax), cv::Range(xMin, xMax));// template picture matched the polygon

		tempImg->CT_offset_x = xMin;
		tempImg->CT_offset_y = yMin;

		if(CALI_TMP)
			tempImg->caliImage();

		//openslam::slam::ORBextractor extractor;
		
		//tempImg->CT_orb1 = cv::ORB::create(FEATURE_POINTS_NUM, 1.2, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31);
		if (CALI_TMP)
			extractor(tempImg->CT_caliedImgObj, cv::Mat(), tempImg->CT_templateKeyPoint, tempImg->CT_TemplateDescriptor);
		else
			extractor(tempImg->CT_imgObj, cv::Mat(), tempImg->CT_templateKeyPoint, tempImg->CT_TemplateDescriptor);

		/*tempImg->CT_orb1 = cv::ORB::create(FEATURE_POINTS_NUM, 1.2, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31);
		if (CALI_TMP)
			tempImg->CT_orb1->detectAndCompute(tempImg->CT_caliedImgObj, cv::Mat(), tempImg->CT_templateKeyPoint, tempImg->CT_TemplateDescriptor, false);
		else
			tempImg->CT_orb1->detectAndCompute(tempImg->CT_imgObj, cv::Mat(), tempImg->CT_templateKeyPoint, tempImg->CT_TemplateDescriptor, false);*/

		// template chip position point vector
		point1.x -= tempImg->CT_offset_x;
		point1.y -= tempImg->CT_offset_y;
		point2.x -= tempImg->CT_offset_x;
		point2.y -= tempImg->CT_offset_y;
		point3.x -= tempImg->CT_offset_x;
		point3.y -= tempImg->CT_offset_y;
		point4.x -= tempImg->CT_offset_x;
		point4.y -= tempImg->CT_offset_y;
		tempImg->CT_chipPosPointVec.clear();
		tempImg->CT_chipPosPointVec.push_back(point1);
		tempImg->CT_chipPosPointVec.push_back(point2);
		tempImg->CT_chipPosPointVec.push_back(point3);
		tempImg->CT_chipPosPointVec.push_back(point4);

		tempImg->topLeftPoint.x = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[0].x;
		tempImg->topLeftPoint.y = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[0].y;
		
		tempImg->topRightPoint.x = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[1].x;
		tempImg->topRightPoint.y = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[1].y;
	}

	tempImg->refAngle = tempImg->calcAngle(tempImg->topLeftPoint, tempImg->topRightPoint);


	return 1;
}

int OCRProcess::CChannelProcess::RunProcess(ALGO_IN_OCR * _segmentIn, std::vector<ALGO_RES_OCR_OBJECT_SEGMENT>& _segmentOut)
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

	Segment seg;

	if(CALI_TMP)
		seg.loadPara(tempImg->CT_caliedImgObj, tempImg->CT_templateKeyPoint, tempImg->CT_TemplateDescriptor, tempImg->CT_chipPosPointVec, tempImg->refAngle);
	else
		seg.loadPara(tempImg->CT_imgObj, tempImg->CT_templateKeyPoint, tempImg->CT_TemplateDescriptor, tempImg->CT_chipPosPointVec, tempImg->refAngle);
	
	if (seg.imgTemplate.empty() || imgDetect.empty())
	{
		return 1; //sample picture or test picture is empty
	}

	seg.InitDetect(imgDetect);

	for (int _index_object = 0; _index_object < mptr_Ocr_CFG->nbr_object; _index_object++)//遍历每个目标Object=>chip
	{
#pragma region 处理每个目标
		seg.T_numRoi = mptr_Ocr_CFG->tbl_object[_index_object].nbr_roi;
		seg.T_rois_rectVec.clear();

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

				rect.x = rect.x - tempImg->CT_offset_x;
				rect.y = rect.y - tempImg->CT_offset_y;

				if (rect.x < 0 || rect.y < 0 || rect.width <= 0 || rect.height <= 0)
				{
					return 3;
				}

				rectVec[_index_charlist] = rect;
			}
			seg.T_rois_rectVec.push_back(rectVec);
#pragma endregion	

		}
		if (DEBUG)
		{
			startTime = cv::getTickCount();
		}

 		if (!seg.slicePosition())
		{
			return 4; //template picture don't match with test picture
		}

		if (DEBUG)
		{
			endTime = cv::getTickCount();
			consumetime = (double)((endTime - startTime) / cv::getTickFrequency()) * 1000;
		}

		/*output the mapped rect to file*/
		for (int D_chipIndex = 0; D_chipIndex < seg.D_chipNum; ++D_chipIndex)
		{
			ALGO_RES_OCR_OBJECT_SEGMENT outResult;
			_segmentOut.push_back(outResult);
			_segmentOut[D_chipIndex].nbr_roi = seg.T_numRoi;

			//all of member of rois in a object
			for (int i = 0; i < seg.T_numRoi; ++i)
			{
				std::vector<cv::Rect> roiRecVec = seg.D_rois_rectVec[i];
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
				int x = seg.D_roisPosRectVec[i].x;
				int y = seg.D_roisPosRectVec[i].y;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].x = x;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].y = y;
				vertexesIndex += 1;

				x = seg.D_roisPosRectVec[i].x + seg.D_roisPosRectVec[i].width;
				y = seg.D_roisPosRectVec[i].y;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].x = x;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].y = y;
				vertexesIndex += 1;

				x = seg.D_roisPosRectVec[i].x + seg.D_roisPosRectVec[i].width;
				y = seg.D_roisPosRectVec[i].y + seg.D_roisPosRectVec[i].height;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].x = x;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].y = y;
				vertexesIndex += 1;

				x = seg.D_roisPosRectVec[i].x;
				y = seg.D_roisPosRectVec[i].y + seg.D_roisPosRectVec[i].height;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].x = x;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].y = y;
				vertexesIndex += 1;

				/*assign outResult's nbr_text_vertexes member value*/
				_segmentOut[D_chipIndex].tbl_roi[i].nbr_text_vertexes = 4;

			}

			/*assign outResult's nbr_object_vertexes member value*/
			_segmentOut[D_chipIndex].nbr_object_vertexes = 4;

			/*assign outResult's sp_object_vertexes member value*/
			for (int chipPosPointIndex = 0; chipPosPointIndex < seg.D_chipPosPointVec.size(); ++chipPosPointIndex)
			{
				cv::Point2f point = seg.D_chipPosPointVec[chipPosPointIndex];
				_segmentOut[D_chipIndex].sp_object_vertexes[chipPosPointIndex].x = point.x;
				_segmentOut[D_chipIndex].sp_object_vertexes[chipPosPointIndex].y = point.y;

			}

		}

	}

	if (0)
	{
		cv::imshow("caliedDetect", seg.imgDetectCalied);
		cv::waitKey(0);
	}
	
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


