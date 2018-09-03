#pragma once
#ifdef OCRPROCESS_EXPORTS
#define OCRPROCESS_DLL __declspec(dllexport)
#else
#define OCRPROCESS_DLL __declspec(dllimport)
#endif


#include "OCR_Center.h"
#include "OCR_Kernel.h"


namespace OcrProcess
{
	 enum OCRPROCESS_DLL MsgEnum
	{
		 LoadINI = ORCMSG_LOAD,
		 Exit = ORCMSG_EXIT,
		 ///<summary>
		 ///<summary>
		 ///<remarks>
		 // wParamIn = &(ALGO_CFG_OCR)
		 // wParamOut = 0
		 ///</remarks>
		 OcrStart = ORCMSG_PROC_START,
		 OcrStop = ORCMSG_PROC_STOP,
		 ///<summary>
		 ///<summary>
		 ///<remarks>
		 // wParamIn =&(ALGO_IN_OCR)
		 // wParamOut =&(ALGO_RES_OCR)
		 ///</remarks>
		 OcrRun_Recognize = ORCMSG_PROC_RUNRECOGNIZE
	};
	class OCRPROCESS_DLL COcrProcessCenter
	{
	public:
		COcrProcessCenter();
		~COcrProcessCenter();
	public:
		long msg_proc(UINT message,long long wParamIn, long long wParamOut);


	private:
		ALGO_CFG_OCR *mptr_ocr_cfg;
		ALGO_RES_OCR *mptr_ocr_res;
		ALGO_IN_OCR *mptr_ocr_in;
		int mptr_ocr_in_imgdata_len=1;
	};
}

