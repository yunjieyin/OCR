#include "stdafx.h"
#include "..\inc\OcrProcessCenter.h"


#include "..\inc\SegmentProcess.h"
#include "..\inc\SegmentProcess.h"

#include <iostream>
#include <thread>
#include <chrono>

#include "..\BMP_M.h"
namespace OcrProcess
{
	BITMAPHEAD_M header;


	COcrProcessCenter::COcrProcessCenter()
	{
		mptr_ocr_cfg = new ALGO_CFG_OCR();
		mptr_ocr_res = new ALGO_RES_OCR();
		mptr_ocr_in = new ALGO_IN_OCR();
	}


	COcrProcessCenter::~COcrProcessCenter()
	{
		if (mptr_ocr_cfg != 0) { delete mptr_ocr_cfg; mptr_ocr_cfg = 0; }
		if (mptr_ocr_res != 0) { delete mptr_ocr_res; mptr_ocr_res = 0; }
		if (mptr_ocr_in != 0) { delete mptr_ocr_in; mptr_ocr_in = 0; }
	}

	long COcrProcessCenter::msg_proc(UINT message, long long wParamIn, long long wParamOut)
	{
		long result = 0;
		switch (message)
		{
		case ORCMSG_LOAD:
		{
			break;
		}
		case ORCMSG_EXIT:
		{
			break;
		}
#pragma region OCR_PROC
		case ORCMSG_PROC_START:
		{
			///pre_work
			ALGO_CFG_OCR * pCfg = (ALGO_CFG_OCR *)wParamIn;
			memcpy(mptr_ocr_cfg, pCfg, sizeof(ALGO_CFG_OCR));
			///work
			long long l1 = (long long)mptr_ocr_cfg;
			result = ocr_algo_msg_proc(LSMSG_OCR_START, 0, (long long)mptr_ocr_cfg);
			//
			mptr_ocr_in->ptr_bits = new BYTE[mptr_ocr_in_imgdata_len];
			break;
		}
		case ORCMSG_PROC_STOP:
		{
			result = ocr_algo_msg_proc(LSMSG_OCR_END, NULL, NULL);
			//
			if(mptr_ocr_in->ptr_bits!=0)
			{
				delete[] mptr_ocr_in->ptr_bits;
				mptr_ocr_in->ptr_bits = 0;
			}
			break;
		}
		case ORCMSG_PROC_RUNRECOGNIZE:
		{
			///pre_work
			ALGO_IN_OCR * ptr_ocr_in = (ALGO_IN_OCR *)wParamIn;
			std::string _img_path = ptr_ocr_in->img_path;
			//put ptr_ocr_in to mptr_ocr_in
			mptr_ocr_in->b_char_location = ptr_ocr_in->b_char_location;
			mptr_ocr_in->b_char_recognize = ptr_ocr_in->b_char_recognize;
			memcpy(mptr_ocr_in->b_rsv, ptr_ocr_in->b_rsv, sizeof(ptr_ocr_in->b_rsv));
			mptr_ocr_in->img_ori = ptr_ocr_in->img_ori;
			memcpy(mptr_ocr_in->img_path, ptr_ocr_in->img_path, sizeof(ptr_ocr_in->img_path));
			mptr_ocr_in->img_type = ptr_ocr_in->img_type;
#pragma region test			
			if (read_bmpheader((char*)(_img_path.c_str()), &header))
			{
				int len = header.bmiHeader.biSizeImage;
				if (mptr_ocr_in_imgdata_len < len)
				{
					delete[]mptr_ocr_in->ptr_bits;
					mptr_ocr_in->ptr_bits = new BYTE[len];
					mptr_ocr_in_imgdata_len = len;
				}
				//read_bmp((char*)(_img_path.c_str()), ptr_ocr_in->ptr_bits, &header);
				read_bmp_lefttop((char*)(_img_path.c_str()), mptr_ocr_in->ptr_bits, &header);
				mptr_ocr_in->img_wh.cx = header.bmiHeader.biWidth;
				mptr_ocr_in->img_wh.cy = header.bmiHeader.biHeight;
				mptr_ocr_in->img_widthstep = cal_bmp_widthstep(header);
				switch (header.bmiHeader.biBitCount)
				{
				case 8:
					mptr_ocr_in->img_type = GRAY;
					break;
				case 24:
					mptr_ocr_in->img_type = RGB24;
					break;
				case 32:
					mptr_ocr_in->img_type = RGB32;
					break;
				default:
					return -1;
				}
			}
			else
			{
				return -1;
			}
#pragma endregion
			///work
			int t1 = sizeof(mptr_ocr_res);
			int t2 = sizeof(ALGO_RES_OCR);
			memset(mptr_ocr_res, 0, sizeof(ALGO_RES_OCR));
			ALGO_TASK task = { 0 };
			task.id_task = LSMSG_OCR_RECOGNIZE;
			task.ptr_in = (BYTE*)mptr_ocr_in;
			task.ptr_out = (BYTE*)mptr_ocr_res;
			/*{
				std::thread t1(ocr_algo_data_proc, &task);
				std::thread t2(ocr_algo_data_proc, &task);
				std::thread t3(ocr_algo_data_proc, &task);
				std::thread t4(ocr_algo_data_proc, &task);
				std::thread t5(ocr_algo_data_proc, &task);
				std::thread t6(ocr_algo_data_proc, &task);
				std::thread t7(ocr_algo_data_proc, &task);
				std::thread t8(ocr_algo_data_proc, &task);
				std::thread t9(ocr_algo_data_proc, &task);
				t1.join();
				t2.join();
				t3.join();
				t4.join();
				t5.join();
				t6.join();
				t7.join();
				t8.join();
				t9.join();
			}*/
			result = ocr_algo_data_proc(&task);
			ALGO_RES_OCR *p1 = (ALGO_RES_OCR*)wParamOut;
			memcpy(p1, task.ptr_out, sizeof(ALGO_RES_OCR));
			break;
		}
#pragma endregion
		default:
			return -1;
		}
		return result;
	}

}