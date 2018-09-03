#ifndef OCR_CENTER_MSGTABLE_
#define OCR_CENTER_MSGTABLE_
///////////////////////////////////////////////////////////////////////////////////
//消息表
///////////////////////////////////////////////////////////////////////////////////
//消息表
#define _WM_USER			0x0400
#define MSG_IDST			(_WM_USER + 0x0000)
#define MSG_IDED			(_WM_USER + 0xFFFF)
//----------------------------------------------------------------------------------
#define		ORCMSG_LOAD					(MSG_IDST + 0x0001)	//开启服务
#define		ORCMSG_EXIT					(MSG_IDST + 0x0002)	//关闭服务


#define		ORCMSG_PROC_START			(MSG_IDST + 0x0101)	//识别算法的开启
//		wParamIn = &(ALGO_CFG_OCR)
//		wParamOut = 0
#define		ORCMSG_PROC_STOP			(MSG_IDST + 0x0102)	//识别算法的关闭
//		wParamIn = 0
//		wParamOut = 0
#define		ORCMSG_PROC_RUNRECOGNIZE	(MSG_IDST + 0x0103)	//识别算法的执行
//		wParamIn = &(ALGO_IN_OCR)
//		wParamOut = &(ALGO_RES_OCR)



///////////////////////////////////////////////////////////////////////////////////		
///////////////////////////////////////////////////////////////////////////////////
//消息表
//#define  CSMSG_START			(MSG_IDST + 0x2001) //与主控建立连接
//#define  CSMSG_STOP			(MSG_IDST + 0x2002) //与主控断开连接
//#define  CSMSG_SEND			(MSG_IDST + 0x2003) //向主控发送数据

#define	 CSCMD_HEARTBEAT		0x11	//	客户端心跳	= 0x11;	【客户端 —> 总控系统】心跳包
#define	 CSCMD_LABLER			0x12	//	打标事件	= 0x12;	【客户端 —> 总控系统】打标事件
#define  CSCMD_LABLER_ALARM		0x13	//	贴标机报警	= 0x13;	【客户端 —> 总控系统】客户端告诉总控系统贴标机有报警
#define  CSCMD_START			0x14	//	启动事件	= 0x14;	【客户端 -> 总控系统】客户端告诉总控系统设备是否准备好启动
#define  CSCMD_STOP			0x15	//	停机事件	= 0x15;	【客户端 -> 总控系统】客户端告诉总控系统设备是否准备好停机
#define  CSCMD_TRI_ALARM		0x16	//	连续三条报警	= 0x16;	【C —> S】客户端告诉总控系统出现连续三条报警及报警信息
#define	 CSCMD_STOP_LISTEN		0xF1	//	停止监听服务	= 0xF1;	发送此类型，监听线程安全停止


#endif