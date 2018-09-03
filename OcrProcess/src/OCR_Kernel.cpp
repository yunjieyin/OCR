#include "stdafx.h"
#include "OCR_Kernel.h"
#include "..\OcrProcess\inc\ChannelProcess.h"

#define channelmaxcount 2

int Random(int start, int end) {
	int dis = end - start;
	return rand() % dis + start;
}
char  arr[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
char RandomChar()
{	
	int len = sizeof(arr)-1;
	int index = Random(0, len);
	return arr[index];
}


//OCRProcess::CSegmentProcess *mp_proc_segment;
//OCRProcess::CSegmentProcess *mp_proc_array_segment;

OCRProcess::CChannelProcess *mp_procchannel_array[channelmaxcount];

long ocr_algo_msg_proc(UINT msg, long long wParam, long long lParam)
{

	switch (msg)
	{
	case LSMSG_OCR_START:
	{
		ALGO_CFG_OCR * pCfg = (ALGO_CFG_OCR *)lParam;
		for (int i = 0; i < channelmaxcount; i++)
		{
			mp_procchannel_array[i] = new OCRProcess::CChannelProcess();
			mp_procchannel_array[i]->SetBaseInfo(pCfg);
		}
		return 0;
	}
	case LSMSG_OCR_END:
	{

		for (int i = 0; i < channelmaxcount; i++)
		{
			if (mp_procchannel_array[i] != 0)
			{
				delete mp_procchannel_array[i]; 
			}
		}
		return 0;
	}
	default:
		break;
	}
	return -1;
}

/// <summary>
/// </summary>
/// <returns>0:Õý³£¡£</returns>
long ocr_algo_data_proc(ALGO_TASK * sp_task)
{
	int res = -1;
	int channelnum = sp_task->index;
	switch (sp_task->id_task)
	{
		case LSMSG_OCR_RECOGNIZE:
		{
			if (true)
			{
				ALGO_IN_OCR * ptr_ocr_in = (ALGO_IN_OCR *)sp_task->ptr_in;
				//std::vector< ALGO_RES_OCR_OBJECT_SEGMENT>  ocr_out;
				//
				ALGO_RES_OCR* ptr_ocr_out = (ALGO_RES_OCR *)sp_task->ptr_out;
				memset(ptr_ocr_out, 0, sizeof(ALGO_RES_OCR));
				//

				res = mp_procchannel_array[channelnum]->RunProcess(ptr_ocr_in, mp_procchannel_array[channelnum]->ocr_out);

				if (res != 0) 
				{
					ptr_ocr_out->b_ok = false;
					return res;
				}
				if (mp_procchannel_array[channelnum]->ocr_out.size() == 0) 
				{
					ptr_ocr_out->b_ok = false;
					return res; 
				}
				ptr_ocr_out->b_ok = true;
				ptr_ocr_out->nbr_object = mp_procchannel_array[channelnum]->ocr_out.size();
				for (int index_obj = 0; index_obj < mp_procchannel_array[channelnum]->ocr_out.size(); index_obj++)
				{
					ptr_ocr_out->tbl_object[index_obj].b_ok = true;
					ptr_ocr_out->tbl_object[index_obj].nbr_roi = mp_procchannel_array[channelnum]->ocr_out[index_obj].nbr_roi;
					for (int index_roi = 0; index_roi < mp_procchannel_array[channelnum]->ocr_out[index_obj].tbl_roi.size(); index_roi++)
					{
						ptr_ocr_out->tbl_object[index_obj].tbl_roi[index_roi].nbr_recognize_char =
							mp_procchannel_array[channelnum]->ocr_out[index_obj].tbl_roi[index_roi].nbr_char;
						for (int index_char = 0; index_char < mp_procchannel_array[channelnum]->ocr_out[index_obj].tbl_roi[index_roi].nbr_char; index_char++)
						{
							ptr_ocr_out->tbl_object[index_obj].tbl_roi[index_roi].tbl_recognize_char_rect[index_char] =
								mp_procchannel_array[channelnum]->ocr_out[index_obj].tbl_roi[index_roi].rect[index_char];
							ptr_ocr_out->tbl_object[index_obj].tbl_roi[index_roi].tbl_recognize_char_text[index_char][0] = RandomChar();
							//
						}
					}
					ptr_ocr_out->tbl_object[index_obj].nbr_roi = mp_procchannel_array[channelnum]->ocr_out[index_obj].nbr_roi;
				}
				///

				mp_procchannel_array[channelnum]->ocr_out.clear();
				std::vector< ALGO_RES_OCR_OBJECT_SEGMENT>().swap(mp_procchannel_array[channelnum]->ocr_out);

			}

 			break;
		}
		default:
			break;
	}
	return res;
}