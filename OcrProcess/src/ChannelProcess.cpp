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
	/*********************************************************************
	* function name:
	* called   by  :
	* Parameters IN:
	* Parameter OUT:
	* Remarks      :
	* function desc:init template picture's parameters according to config info
	* changed date :
	* user         :
	*********************************************************************/
	int s1 = sizeof(ALGO_CFG_OCR);
	int s2 = sizeof(*_segmentcfg);
	int s3 = sizeof(*mptr_Ocr_CFG);
	memcpy(mptr_Ocr_CFG, _segmentcfg, sizeof(ALGO_CFG_OCR));

	//original template picture
	tempImg->m_templateImg = cv::imread(mptr_Ocr_CFG->img_path);

	//init objects's postions of the template picture
	cv::Point2f point1, point2, point3, point4;
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

		point1.x = xMin;
		point1.y = yMin;

		point2.x = xMax;
		point2.y = yMin;

		point3.x = xMax;
		point3.y = yMax;

		point4.x = xMin;
		point4.y = yMax;

		if (xMin < 0 || yMin < 0 || xMax > tempImg->m_templateImg.cols
			|| yMax > tempImg->m_templateImg.rows || xMax <= xMin || yMax <= yMin)
		{
			return 2; //tailored template picture's position points(4) not valid
		}

		// init tailored template picture that mathch the polygon
		tempImg->m_objectImg = tempImg->m_templateImg(cv::Range(yMin, yMax), cv::Range(xMin, xMax));

		tempImg->m_offsetX = xMin;
		tempImg->m_offsetY = yMin;

		if(CALI_TMP)
			tempImg->caliImage();

		//compute tailored template picture's keypoints and descriptors
		if (CALI_TMP)
			extractor(tempImg->m_caliedObjectImg, cv::Mat(), tempImg->m_templateKeyPoints, tempImg->m_TemplateDescriptor);
		else
			extractor(tempImg->m_objectImg, cv::Mat(), tempImg->m_templateKeyPoints, tempImg->m_TemplateDescriptor);

		if (0)
		{
			//compute keypoints and descriptors by opencv's ORB method
			tempImg->m_templateImgORB = cv::ORB::create(FEATURE_POINTS_NUM, 1.2, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31);
			if (CALI_TMP)
				tempImg->m_templateImgORB->detectAndCompute(tempImg->m_caliedObjectImg, cv::Mat(), tempImg->m_templateKeyPoints, 
					tempImg->m_TemplateDescriptor, false);
			else
				tempImg->m_templateImgORB->detectAndCompute(tempImg->m_objectImg, cv::Mat(), tempImg->m_templateKeyPoints, 
					tempImg->m_TemplateDescriptor, false);
		}
		
		//chip's position that lies in tailored template picture
		point1.x -= tempImg->m_offsetX;
		point1.y -= tempImg->m_offsetY;
		point2.x -= tempImg->m_offsetX;
		point2.y -= tempImg->m_offsetY;
		point3.x -= tempImg->m_offsetX;
		point3.y -= tempImg->m_offsetY;
		point4.x -= tempImg->m_offsetX;
		point4.y -= tempImg->m_offsetY;
		tempImg->m_chipPosPointVec.clear();
		tempImg->m_chipPosPointVec.push_back(point1);
		tempImg->m_chipPosPointVec.push_back(point2);
		tempImg->m_chipPosPointVec.push_back(point3);
		tempImg->m_chipPosPointVec.push_back(point4);

		tempImg->m_topLeftPoint.x = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[0].x;
		tempImg->m_topLeftPoint.y = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[0].y;
		
		tempImg->m_topRightPoint.x = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[1].x;
		tempImg->m_topRightPoint.y = mptr_Ocr_CFG->tbl_object[_index_object].sp_vertices_pos[1].y;
	}

	//compute template picture's slant angle
	tempImg->m_refAngle = tempImg->calcAngle(tempImg->m_topLeftPoint, tempImg->m_topRightPoint);


	return 1;
}

int OCRProcess::CChannelProcess::RunProcess(ALGO_IN_OCR * _segmentIn, std::vector<ALGO_RES_OCR_OBJECT_SEGMENT>& _segmentOut)
{
	/*********************************************************************
	* function name:
	* called   by  :
	* Parameters IN:info of template picture and under detect picture
	* Parameter OUT:segment position lies in under detect picture
	* Remarks      :
	* function desc:main entrance of segment
	* changed date :
	* user         :
	*********************************************************************/
	memset(&_segmentOut, 0, sizeof(_segmentOut));

	double consumetime;
	double startTime;
	double endTime;

	cv::Mat imgDetect;

	if (!Segment::initImage(_segmentIn, imgDetect))
	{
		return 5; //initilize under detected image failed;
	}

	Segment seg;

	//load the template picture's parameters
	if(CALI_TMP)
		seg.loadPara(tempImg->m_caliedObjectImg, 
					tempImg->m_templateKeyPoints,
					tempImg->m_TemplateDescriptor,
					tempImg->m_chipPosPointVec,
					tempImg->m_refAngle);
	else
		seg.loadPara(tempImg->m_objectImg,
					tempImg->m_templateKeyPoints,
					tempImg->m_TemplateDescriptor,
					tempImg->m_chipPosPointVec,
					tempImg->m_refAngle);
	
	if (seg.m_imgTemplate.empty() || imgDetect.empty())
	{
		return 1; //template picture or under detection picture is empty
	}

	seg.InitDetect(imgDetect);

	//compute all of characters, rois and objects position parameters according to config file
	for (int _index_object = 0; _index_object < mptr_Ocr_CFG->nbr_object; _index_object++)//遍历每个目标Object=>chip
	{
#pragma region 处理每个目标
		//the number of roi responding to a object
		seg.m_TnumRoi = mptr_Ocr_CFG->tbl_object[_index_object].nbr_roi;
		seg.m_Trois_rectVec.clear();

		//all of the rois's parameters
		for (int _index_Temp = 0; _index_Temp < mptr_Ocr_CFG->tbl_object[_index_object].nbr_roi; _index_Temp++)//遍历每个模板ROI
		{
#pragma region T_rois_rectVec

			//the number of characters of a roi
			int charNum = mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].nbr_char;
			std::vector<cv::Rect> charRectVec(charNum);

			//compute the position of  all of the characters
			cv::Rect charRect;
			for (int _index_charlist = 0; _index_charlist < mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].nbr_char; _index_charlist++)
			{
				charRect.x = mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].left;
				charRect.y = mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].top;
				charRect.width = (
					mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].right -
					mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].left);
				charRect.height = (
					mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].bottom -
					mptr_Ocr_CFG->tbl_object[_index_object].tbl_roi_data[_index_Temp].tbl_char_rect[_index_charlist].top);

				charRect.x = charRect.x - tempImg->m_offsetX;
				charRect.y = charRect.y - tempImg->m_offsetY;

				if (charRect.x < 0 || charRect.y < 0 || charRect.width <= 0 || charRect.height <= 0)
				{
					return 3;
				}

				charRectVec[_index_charlist] = charRect;
			}
			seg.m_Trois_rectVec.push_back(charRectVec);
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
		for (int D_chipIndex = 0; D_chipIndex < seg.m_DchipNum; ++D_chipIndex)
		{
			ALGO_RES_OCR_OBJECT_SEGMENT outResult;
			_segmentOut.push_back(outResult);
			_segmentOut[D_chipIndex].nbr_roi = seg.m_TnumRoi;

			//all of member of rois in a object
			for (int i = 0; i < seg.m_TnumRoi; ++i)
			{
				std::vector<cv::Rect> roiRecVec = seg.m_Drois_rectVec[i];
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
				int x = seg.m_DroisPosRectVec[i].x;
				int y = seg.m_DroisPosRectVec[i].y;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].x = x;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].y = y;
				vertexesIndex += 1;

				x = seg.m_DroisPosRectVec[i].x + seg.m_DroisPosRectVec[i].width;
				y = seg.m_DroisPosRectVec[i].y;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].x = x;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].y = y;
				vertexesIndex += 1;

				x = seg.m_DroisPosRectVec[i].x + seg.m_DroisPosRectVec[i].width;
				y = seg.m_DroisPosRectVec[i].y + seg.m_DroisPosRectVec[i].height;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].x = x;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].y = y;
				vertexesIndex += 1;

				x = seg.m_DroisPosRectVec[i].x;
				y = seg.m_DroisPosRectVec[i].y + seg.m_DroisPosRectVec[i].height;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].x = x;
				_segmentOut[D_chipIndex].tbl_roi[i].sp_text_vertexes[vertexesIndex].y = y;
				vertexesIndex += 1;

				/*assign outResult's nbr_text_vertexes member value*/
				_segmentOut[D_chipIndex].tbl_roi[i].nbr_text_vertexes = 4;

			}

			/*assign outResult's nbr_object_vertexes member value*/
			_segmentOut[D_chipIndex].nbr_object_vertexes = 4;

			/*assign outResult's sp_object_vertexes member value*/
			for (int chipPosPointIndex = 0; chipPosPointIndex < seg.m_DchipPosPointVec.size(); ++chipPosPointIndex)
			{
				cv::Point2f point = seg.m_DchipPosPointVec[chipPosPointIndex];
				_segmentOut[D_chipIndex].sp_object_vertexes[chipPosPointIndex].x = point.x;
				_segmentOut[D_chipIndex].sp_object_vertexes[chipPosPointIndex].y = point.y;

			}

		}

	}

	if (0)
	{
		cv::imshow("caliedDetect", seg.m_imgDetectCalied);
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


