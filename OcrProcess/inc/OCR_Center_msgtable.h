#ifndef OCR_CENTER_MSGTABLE_
#define OCR_CENTER_MSGTABLE_
///////////////////////////////////////////////////////////////////////////////////
//��Ϣ��
///////////////////////////////////////////////////////////////////////////////////
//��Ϣ��
#define _WM_USER			0x0400
#define MSG_IDST			(_WM_USER + 0x0000)
#define MSG_IDED			(_WM_USER + 0xFFFF)
//----------------------------------------------------------------------------------
#define		ORCMSG_LOAD					(MSG_IDST + 0x0001)	//��������
#define		ORCMSG_EXIT					(MSG_IDST + 0x0002)	//�رշ���


#define		ORCMSG_PROC_START			(MSG_IDST + 0x0101)	//ʶ���㷨�Ŀ���
//		wParamIn = &(ALGO_CFG_OCR)
//		wParamOut = 0
#define		ORCMSG_PROC_STOP			(MSG_IDST + 0x0102)	//ʶ���㷨�Ĺر�
//		wParamIn = 0
//		wParamOut = 0
#define		ORCMSG_PROC_RUNRECOGNIZE	(MSG_IDST + 0x0103)	//ʶ���㷨��ִ��
//		wParamIn = &(ALGO_IN_OCR)
//		wParamOut = &(ALGO_RES_OCR)



///////////////////////////////////////////////////////////////////////////////////		
///////////////////////////////////////////////////////////////////////////////////
//��Ϣ��
//#define  CSMSG_START			(MSG_IDST + 0x2001) //�����ؽ�������
//#define  CSMSG_STOP			(MSG_IDST + 0x2002) //�����ضϿ�����
//#define  CSMSG_SEND			(MSG_IDST + 0x2003) //�����ط�������

#define	 CSCMD_HEARTBEAT		0x11	//	�ͻ�������	= 0x11;	���ͻ��� ��> �ܿ�ϵͳ��������
#define	 CSCMD_LABLER			0x12	//	����¼�	= 0x12;	���ͻ��� ��> �ܿ�ϵͳ������¼�
#define  CSCMD_LABLER_ALARM		0x13	//	���������	= 0x13;	���ͻ��� ��> �ܿ�ϵͳ���ͻ��˸����ܿ�ϵͳ������б���
#define  CSCMD_START			0x14	//	�����¼�	= 0x14;	���ͻ��� -> �ܿ�ϵͳ���ͻ��˸����ܿ�ϵͳ�豸�Ƿ�׼��������
#define  CSCMD_STOP			0x15	//	ͣ���¼�	= 0x15;	���ͻ��� -> �ܿ�ϵͳ���ͻ��˸����ܿ�ϵͳ�豸�Ƿ�׼����ͣ��
#define  CSCMD_TRI_ALARM		0x16	//	������������	= 0x16;	��C ��> S���ͻ��˸����ܿ�ϵͳ������������������������Ϣ
#define	 CSCMD_STOP_LISTEN		0xF1	//	ֹͣ��������	= 0xF1;	���ʹ����ͣ������̰߳�ȫֹͣ


#endif