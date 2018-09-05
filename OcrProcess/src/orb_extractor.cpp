#include "stdafx.h"
#include <orb_extractor.h>

namespace orb
{
	const int PATCH_SIZE = 31;//用于计算BIREF描述子的特征点邻域大小
	const int HALF_PATCH_SIZE = 15;//用于计算BIREF描述子的特征点邻域大小的一半
	const int EDGE_THRESHOLD = 19;//边缘阈值,靠近边缘阈值以内的像素是不检测特征点的。

	// 灰度质心法计算特征点方向
	static float Intensity_Centroid_Angle(const cv::Mat& image, cv::Point2f pt, const std::vector<int>& u_max)
	{
		int m_01 = 0, m_10 = 0;
		// 得到中心位置
		const uchar* pCenter = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));

		// 对 v=0 这一行单独计算
		for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
			m_10 += u * pCenter[u];

		// 这边要注意图像的step不一定是图像的宽度
		int step = (int)image.step1();

		for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
		{
			// 上下和左右两条线同时计算
			int v_sum = 0;
			int d = u_max[v];
			for (int u = -d; u <= d; ++u)
			{
				int val_plus = pCenter[u + v*step], val_minus = pCenter[u - v*step];
				v_sum += (val_plus - val_minus);//计算上下的时候是有符号的，所以这边是减
				m_10 += u * (val_plus + val_minus);//这边加是由于u已经确定好了符号
			}
			m_01 += v * v_sum;
		}

		return cv::fastAtan2((float)m_01, (float)m_10);
	}


	const float factor_pi = (float)(CV_PI / 180.f);
	static void computeOrbDescriptor(const cv::KeyPoint& kpt,
		const cv::Mat& img, const cv::Point* pattern,
		uchar* desc)
	{
		float angle = (float)kpt.angle*factor_pi;
		float a = (float)cos(angle), b = (float)sin(angle);

		const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
		const int step = (int)img.step;
#define GET_VALUE(idx) \
    center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
            cvRound(pattern[idx].x*a - pattern[idx].y*b)]

		int t0, t1, val;
		for (int i = 0; i < 32; ++i, pattern += 16)
		{
			t0 = GET_VALUE(0); t1 = GET_VALUE(1);
			val = t0 < t1;
			t0 = GET_VALUE(2); t1 = GET_VALUE(3);
			val |= (t0 < t1) << 1;
			t0 = GET_VALUE(4); t1 = GET_VALUE(5);
			val |= (t0 < t1) << 2;
			t0 = GET_VALUE(6); t1 = GET_VALUE(7);
			val |= (t0 < t1) << 3;
			t0 = GET_VALUE(8); t1 = GET_VALUE(9);
			val |= (t0 < t1) << 4;
			t0 = GET_VALUE(10); t1 = GET_VALUE(11);
			val |= (t0 < t1) << 5;
			t0 = GET_VALUE(12); t1 = GET_VALUE(13);
			val |= (t0 < t1) << 6;
			t0 = GET_VALUE(14); t1 = GET_VALUE(15);
			val |= (t0 < t1) << 7;

			desc[i] = (uchar)val;
		}

#undef GET_VALUE
	}


	static int bit_pattern_31_[256 * 4] =
	{
		8, -3, 9, 5/*mean (0), correlation (0)*/,
		4, 2, 7, -12/*mean (1.12461e-05), correlation (0.0437584)*/,
		-11, 9, -8, 2/*mean (3.37382e-05), correlation (0.0617409)*/,
		7, -12, 12, -13/*mean (5.62303e-05), correlation (0.0636977)*/,
		2, -13, 2, 12/*mean (0.000134953), correlation (0.085099)*/,
		1, -7, 1, 6/*mean (0.000528565), correlation (0.0857175)*/,
		-2, -10, -2, -4/*mean (0.0188821), correlation (0.0985774)*/,
		-13, -13, -11, -8/*mean (0.0363135), correlation (0.0899616)*/,
		-13, -3, -12, -9/*mean (0.121806), correlation (0.099849)*/,
		10, 4, 11, 9/*mean (0.122065), correlation (0.093285)*/,
		-13, -8, -8, -9/*mean (0.162787), correlation (0.0942748)*/,
		-11, 7, -9, 12/*mean (0.21561), correlation (0.0974438)*/,
		7, 7, 12, 6/*mean (0.160583), correlation (0.130064)*/,
		-4, -5, -3, 0/*mean (0.228171), correlation (0.132998)*/,
		-13, 2, -12, -3/*mean (0.00997526), correlation (0.145926)*/,
		-9, 0, -7, 5/*mean (0.198234), correlation (0.143636)*/,
		12, -6, 12, -1/*mean (0.0676226), correlation (0.16689)*/,
		-3, 6, -2, 12/*mean (0.166847), correlation (0.171682)*/,
		-6, -13, -4, -8/*mean (0.101215), correlation (0.179716)*/,
		11, -13, 12, -8/*mean (0.200641), correlation (0.192279)*/,
		4, 7, 5, 1/*mean (0.205106), correlation (0.186848)*/,
		5, -3, 10, -3/*mean (0.234908), correlation (0.192319)*/,
		3, -7, 6, 12/*mean (0.0709964), correlation (0.210872)*/,
		-8, -7, -6, -2/*mean (0.0939834), correlation (0.212589)*/,
		-2, 11, -1, -10/*mean (0.127778), correlation (0.20866)*/,
		-13, 12, -8, 10/*mean (0.14783), correlation (0.206356)*/,
		-7, 3, -5, -3/*mean (0.182141), correlation (0.198942)*/,
		-4, 2, -3, 7/*mean (0.188237), correlation (0.21384)*/,
		-10, -12, -6, 11/*mean (0.14865), correlation (0.23571)*/,
		5, -12, 6, -7/*mean (0.222312), correlation (0.23324)*/,
		5, -6, 7, -1/*mean (0.229082), correlation (0.23389)*/,
		1, 0, 4, -5/*mean (0.241577), correlation (0.215286)*/,
		9, 11, 11, -13/*mean (0.00338507), correlation (0.251373)*/,
		4, 7, 4, 12/*mean (0.131005), correlation (0.257622)*/,
		2, -1, 4, 4/*mean (0.152755), correlation (0.255205)*/,
		-4, -12, -2, 7/*mean (0.182771), correlation (0.244867)*/,
		-8, -5, -7, -10/*mean (0.186898), correlation (0.23901)*/,
		4, 11, 9, 12/*mean (0.226226), correlation (0.258255)*/,
		0, -8, 1, -13/*mean (0.0897886), correlation (0.274827)*/,
		-13, -2, -8, 2/*mean (0.148774), correlation (0.28065)*/,
		-3, -2, -2, 3/*mean (0.153048), correlation (0.283063)*/,
		-6, 9, -4, -9/*mean (0.169523), correlation (0.278248)*/,
		8, 12, 10, 7/*mean (0.225337), correlation (0.282851)*/,
		0, 9, 1, 3/*mean (0.226687), correlation (0.278734)*/,
		7, -5, 11, -10/*mean (0.00693882), correlation (0.305161)*/,
		-13, -6, -11, 0/*mean (0.0227283), correlation (0.300181)*/,
		10, 7, 12, 1/*mean (0.125517), correlation (0.31089)*/,
		-6, -3, -6, 12/*mean (0.131748), correlation (0.312779)*/,
		10, -9, 12, -4/*mean (0.144827), correlation (0.292797)*/,
		-13, 8, -8, -12/*mean (0.149202), correlation (0.308918)*/,
		-13, 0, -8, -4/*mean (0.160909), correlation (0.310013)*/,
		3, 3, 7, 8/*mean (0.177755), correlation (0.309394)*/,
		5, 7, 10, -7/*mean (0.212337), correlation (0.310315)*/,
		-1, 7, 1, -12/*mean (0.214429), correlation (0.311933)*/,
		3, -10, 5, 6/*mean (0.235807), correlation (0.313104)*/,
		2, -4, 3, -10/*mean (0.00494827), correlation (0.344948)*/,
		-13, 0, -13, 5/*mean (0.0549145), correlation (0.344675)*/,
		-13, -7, -12, 12/*mean (0.103385), correlation (0.342715)*/,
		-13, 3, -11, 8/*mean (0.134222), correlation (0.322922)*/,
		-7, 12, -4, 7/*mean (0.153284), correlation (0.337061)*/,
		6, -10, 12, 8/*mean (0.154881), correlation (0.329257)*/,
		-9, -1, -7, -6/*mean (0.200967), correlation (0.33312)*/,
		-2, -5, 0, 12/*mean (0.201518), correlation (0.340635)*/,
		-12, 5, -7, 5/*mean (0.207805), correlation (0.335631)*/,
		3, -10, 8, -13/*mean (0.224438), correlation (0.34504)*/,
		-7, -7, -4, 5/*mean (0.239361), correlation (0.338053)*/,
		-3, -2, -1, -7/*mean (0.240744), correlation (0.344322)*/,
		2, 9, 5, -11/*mean (0.242949), correlation (0.34145)*/,
		-11, -13, -5, -13/*mean (0.244028), correlation (0.336861)*/,
		-1, 6, 0, -1/*mean (0.247571), correlation (0.343684)*/,
		5, -3, 5, 2/*mean (0.000697256), correlation (0.357265)*/,
		-4, -13, -4, 12/*mean (0.00213675), correlation (0.373827)*/,
		-9, -6, -9, 6/*mean (0.0126856), correlation (0.373938)*/,
		-12, -10, -8, -4/*mean (0.0152497), correlation (0.364237)*/,
		10, 2, 12, -3/*mean (0.0299933), correlation (0.345292)*/,
		7, 12, 12, 12/*mean (0.0307242), correlation (0.366299)*/,
		-7, -13, -6, 5/*mean (0.0534975), correlation (0.368357)*/,
		-4, 9, -3, 4/*mean (0.099865), correlation (0.372276)*/,
		7, -1, 12, 2/*mean (0.117083), correlation (0.364529)*/,
		-7, 6, -5, 1/*mean (0.126125), correlation (0.369606)*/,
		-13, 11, -12, 5/*mean (0.130364), correlation (0.358502)*/,
		-3, 7, -2, -6/*mean (0.131691), correlation (0.375531)*/,
		7, -8, 12, -7/*mean (0.160166), correlation (0.379508)*/,
		-13, -7, -11, -12/*mean (0.167848), correlation (0.353343)*/,
		1, -3, 12, 12/*mean (0.183378), correlation (0.371916)*/,
		2, -6, 3, 0/*mean (0.228711), correlation (0.371761)*/,
		-4, 3, -2, -13/*mean (0.247211), correlation (0.364063)*/,
		-1, -13, 1, 9/*mean (0.249325), correlation (0.378139)*/,
		7, 1, 8, -6/*mean (0.000652272), correlation (0.411682)*/,
		1, -1, 3, 12/*mean (0.00248538), correlation (0.392988)*/,
		9, 1, 12, 6/*mean (0.0206815), correlation (0.386106)*/,
		-1, -9, -1, 3/*mean (0.0364485), correlation (0.410752)*/,
		-13, -13, -10, 5/*mean (0.0376068), correlation (0.398374)*/,
		7, 7, 10, 12/*mean (0.0424202), correlation (0.405663)*/,
		12, -5, 12, 9/*mean (0.0942645), correlation (0.410422)*/,
		6, 3, 7, 11/*mean (0.1074), correlation (0.413224)*/,
		5, -13, 6, 10/*mean (0.109256), correlation (0.408646)*/,
		2, -12, 2, 3/*mean (0.131691), correlation (0.416076)*/,
		3, 8, 4, -6/*mean (0.165081), correlation (0.417569)*/,
		2, 6, 12, -13/*mean (0.171874), correlation (0.408471)*/,
		9, -12, 10, 3/*mean (0.175146), correlation (0.41296)*/,
		-8, 4, -7, 9/*mean (0.183682), correlation (0.402956)*/,
		-11, 12, -4, -6/*mean (0.184672), correlation (0.416125)*/,
		1, 12, 2, -8/*mean (0.191487), correlation (0.386696)*/,
		6, -9, 7, -4/*mean (0.192668), correlation (0.394771)*/,
		2, 3, 3, -2/*mean (0.200157), correlation (0.408303)*/,
		6, 3, 11, 0/*mean (0.204588), correlation (0.411762)*/,
		3, -3, 8, -8/*mean (0.205904), correlation (0.416294)*/,
		7, 8, 9, 3/*mean (0.213237), correlation (0.409306)*/,
		-11, -5, -6, -4/*mean (0.243444), correlation (0.395069)*/,
		-10, 11, -5, 10/*mean (0.247672), correlation (0.413392)*/,
		-5, -8, -3, 12/*mean (0.24774), correlation (0.411416)*/,
		-10, 5, -9, 0/*mean (0.00213675), correlation (0.454003)*/,
		8, -1, 12, -6/*mean (0.0293635), correlation (0.455368)*/,
		4, -6, 6, -11/*mean (0.0404971), correlation (0.457393)*/,
		-10, 12, -8, 7/*mean (0.0481107), correlation (0.448364)*/,
		4, -2, 6, 7/*mean (0.050641), correlation (0.455019)*/,
		-2, 0, -2, 12/*mean (0.0525978), correlation (0.44338)*/,
		-5, -8, -5, 2/*mean (0.0629667), correlation (0.457096)*/,
		7, -6, 10, 12/*mean (0.0653846), correlation (0.445623)*/,
		-9, -13, -8, -8/*mean (0.0858749), correlation (0.449789)*/,
		-5, -13, -5, -2/*mean (0.122402), correlation (0.450201)*/,
		8, -8, 9, -13/*mean (0.125416), correlation (0.453224)*/,
		-9, -11, -9, 0/*mean (0.130128), correlation (0.458724)*/,
		1, -8, 1, -2/*mean (0.132467), correlation (0.440133)*/,
		7, -4, 9, 1/*mean (0.132692), correlation (0.454)*/,
		-2, 1, -1, -4/*mean (0.135695), correlation (0.455739)*/,
		11, -6, 12, -11/*mean (0.142904), correlation (0.446114)*/,
		-12, -9, -6, 4/*mean (0.146165), correlation (0.451473)*/,
		3, 7, 7, 12/*mean (0.147627), correlation (0.456643)*/,
		5, 5, 10, 8/*mean (0.152901), correlation (0.455036)*/,
		0, -4, 2, 8/*mean (0.167083), correlation (0.459315)*/,
		-9, 12, -5, -13/*mean (0.173234), correlation (0.454706)*/,
		0, 7, 2, 12/*mean (0.18312), correlation (0.433855)*/,
		-1, 2, 1, 7/*mean (0.185504), correlation (0.443838)*/,
		5, 11, 7, -9/*mean (0.185706), correlation (0.451123)*/,
		3, 5, 6, -8/*mean (0.188968), correlation (0.455808)*/,
		-13, -4, -8, 9/*mean (0.191667), correlation (0.459128)*/,
		-5, 9, -3, -3/*mean (0.193196), correlation (0.458364)*/,
		-4, -7, -3, -12/*mean (0.196536), correlation (0.455782)*/,
		6, 5, 8, 0/*mean (0.1972), correlation (0.450481)*/,
		-7, 6, -6, 12/*mean (0.199438), correlation (0.458156)*/,
		-13, 6, -5, -2/*mean (0.211224), correlation (0.449548)*/,
		1, -10, 3, 10/*mean (0.211718), correlation (0.440606)*/,
		4, 1, 8, -4/*mean (0.213034), correlation (0.443177)*/,
		-2, -2, 2, -13/*mean (0.234334), correlation (0.455304)*/,
		2, -12, 12, 12/*mean (0.235684), correlation (0.443436)*/,
		-2, -13, 0, -6/*mean (0.237674), correlation (0.452525)*/,
		4, 1, 9, 3/*mean (0.23962), correlation (0.444824)*/,
		-6, -10, -3, -5/*mean (0.248459), correlation (0.439621)*/,
		-3, -13, -1, 1/*mean (0.249505), correlation (0.456666)*/,
		7, 5, 12, -11/*mean (0.00119208), correlation (0.495466)*/,
		4, -2, 5, -7/*mean (0.00372245), correlation (0.484214)*/,
		-13, 9, -9, -5/*mean (0.00741116), correlation (0.499854)*/,
		7, 1, 8, 6/*mean (0.0208952), correlation (0.499773)*/,
		7, -8, 7, 6/*mean (0.0220085), correlation (0.501609)*/,
		-7, -4, -7, 1/*mean (0.0233806), correlation (0.496568)*/,
		-8, 11, -7, -8/*mean (0.0236505), correlation (0.489719)*/,
		-13, 6, -12, -8/*mean (0.0268781), correlation (0.503487)*/,
		2, 4, 3, 9/*mean (0.0323324), correlation (0.501938)*/,
		10, -5, 12, 3/*mean (0.0399235), correlation (0.494029)*/,
		-6, -5, -6, 7/*mean (0.0420153), correlation (0.486579)*/,
		8, -3, 9, -8/*mean (0.0548021), correlation (0.484237)*/,
		2, -12, 2, 8/*mean (0.0616622), correlation (0.496642)*/,
		-11, -2, -10, 3/*mean (0.0627755), correlation (0.498563)*/,
		-12, -13, -7, -9/*mean (0.0829622), correlation (0.495491)*/,
		-11, 0, -10, -5/*mean (0.0843342), correlation (0.487146)*/,
		5, -3, 11, 8/*mean (0.0929937), correlation (0.502315)*/,
		-2, -13, -1, 12/*mean (0.113327), correlation (0.48941)*/,
		-1, -8, 0, 9/*mean (0.132119), correlation (0.467268)*/,
		-13, -11, -12, -5/*mean (0.136269), correlation (0.498771)*/,
		-10, -2, -10, 11/*mean (0.142173), correlation (0.498714)*/,
		-3, 9, -2, -13/*mean (0.144141), correlation (0.491973)*/,
		2, -3, 3, 2/*mean (0.14892), correlation (0.500782)*/,
		-9, -13, -4, 0/*mean (0.150371), correlation (0.498211)*/,
		-4, 6, -3, -10/*mean (0.152159), correlation (0.495547)*/,
		-4, 12, -2, -7/*mean (0.156152), correlation (0.496925)*/,
		-6, -11, -4, 9/*mean (0.15749), correlation (0.499222)*/,
		6, -3, 6, 11/*mean (0.159211), correlation (0.503821)*/,
		-13, 11, -5, 5/*mean (0.162427), correlation (0.501907)*/,
		11, 11, 12, 6/*mean (0.16652), correlation (0.497632)*/,
		7, -5, 12, -2/*mean (0.169141), correlation (0.484474)*/,
		-1, 12, 0, 7/*mean (0.169456), correlation (0.495339)*/,
		-4, -8, -3, -2/*mean (0.171457), correlation (0.487251)*/,
		-7, 1, -6, 7/*mean (0.175), correlation (0.500024)*/,
		-13, -12, -8, -13/*mean (0.175866), correlation (0.497523)*/,
		-7, -2, -6, -8/*mean (0.178273), correlation (0.501854)*/,
		-8, 5, -6, -9/*mean (0.181107), correlation (0.494888)*/,
		-5, -1, -4, 5/*mean (0.190227), correlation (0.482557)*/,
		-13, 7, -8, 10/*mean (0.196739), correlation (0.496503)*/,
		1, 5, 5, -13/*mean (0.19973), correlation (0.499759)*/,
		1, 0, 10, -13/*mean (0.204465), correlation (0.49873)*/,
		9, 12, 10, -1/*mean (0.209334), correlation (0.49063)*/,
		5, -8, 10, -9/*mean (0.211134), correlation (0.503011)*/,
		-1, 11, 1, -13/*mean (0.212), correlation (0.499414)*/,
		-9, -3, -6, 2/*mean (0.212168), correlation (0.480739)*/,
		-1, -10, 1, 12/*mean (0.212731), correlation (0.502523)*/,
		-13, 1, -8, -10/*mean (0.21327), correlation (0.489786)*/,
		8, -11, 10, -6/*mean (0.214159), correlation (0.488246)*/,
		2, -13, 3, -6/*mean (0.216993), correlation (0.50287)*/,
		7, -13, 12, -9/*mean (0.223639), correlation (0.470502)*/,
		-10, -10, -5, -7/*mean (0.224089), correlation (0.500852)*/,
		-10, -8, -8, -13/*mean (0.228666), correlation (0.502629)*/,
		4, -6, 8, 5/*mean (0.22906), correlation (0.498305)*/,
		3, 12, 8, -13/*mean (0.233378), correlation (0.503825)*/,
		-4, 2, -3, -3/*mean (0.234323), correlation (0.476692)*/,
		5, -13, 10, -12/*mean (0.236392), correlation (0.475462)*/,
		4, -13, 5, -1/*mean (0.236842), correlation (0.504132)*/,
		-9, 9, -4, 3/*mean (0.236977), correlation (0.497739)*/,
		0, 3, 3, -9/*mean (0.24314), correlation (0.499398)*/,
		-12, 1, -6, 1/*mean (0.243297), correlation (0.489447)*/,
		3, 2, 4, -8/*mean (0.00155196), correlation (0.553496)*/,
		-10, -10, -10, 9/*mean (0.00239541), correlation (0.54297)*/,
		8, -13, 12, 12/*mean (0.0034413), correlation (0.544361)*/,
		-8, -12, -6, -5/*mean (0.003565), correlation (0.551225)*/,
		2, 2, 3, 7/*mean (0.00835583), correlation (0.55285)*/,
		10, 6, 11, -8/*mean (0.00885065), correlation (0.540913)*/,
		6, 8, 8, -12/*mean (0.0101552), correlation (0.551085)*/,
		-7, 10, -6, 5/*mean (0.0102227), correlation (0.533635)*/,
		-3, -9, -3, 9/*mean (0.0110211), correlation (0.543121)*/,
		-1, -13, -1, 5/*mean (0.0113473), correlation (0.550173)*/,
		-3, -7, -3, 4/*mean (0.0140913), correlation (0.554774)*/,
		-8, -2, -8, 3/*mean (0.017049), correlation (0.55461)*/,
		4, 2, 12, 12/*mean (0.01778), correlation (0.546921)*/,
		2, -5, 3, 11/*mean (0.0224022), correlation (0.549667)*/,
		6, -9, 11, -13/*mean (0.029161), correlation (0.546295)*/,
		3, -1, 7, 12/*mean (0.0303081), correlation (0.548599)*/,
		11, -1, 12, 4/*mean (0.0355151), correlation (0.523943)*/,
		-3, 0, -3, 6/*mean (0.0417904), correlation (0.543395)*/,
		4, -11, 4, 12/*mean (0.0487292), correlation (0.542818)*/,
		2, -4, 2, 1/*mean (0.0575124), correlation (0.554888)*/,
		-10, -6, -8, 1/*mean (0.0594242), correlation (0.544026)*/,
		-13, 7, -11, 1/*mean (0.0597391), correlation (0.550524)*/,
		-13, 12, -11, -13/*mean (0.0608974), correlation (0.55383)*/,
		6, 0, 11, -13/*mean (0.065126), correlation (0.552006)*/,
		0, -1, 1, 4/*mean (0.074224), correlation (0.546372)*/,
		-13, 3, -9, -2/*mean (0.0808592), correlation (0.554875)*/,
		-9, 8, -6, -3/*mean (0.0883378), correlation (0.551178)*/,
		-13, -6, -8, -2/*mean (0.0901035), correlation (0.548446)*/,
		5, -9, 8, 10/*mean (0.0949843), correlation (0.554694)*/,
		2, 7, 3, -9/*mean (0.0994152), correlation (0.550979)*/,
		-1, -6, -1, -1/*mean (0.10045), correlation (0.552714)*/,
		9, 5, 11, -2/*mean (0.100686), correlation (0.552594)*/,
		11, -3, 12, -8/*mean (0.101091), correlation (0.532394)*/,
		3, 0, 3, 5/*mean (0.101147), correlation (0.525576)*/,
		-1, 4, 0, 10/*mean (0.105263), correlation (0.531498)*/,
		3, -6, 4, 5/*mean (0.110785), correlation (0.540491)*/,
		-13, 0, -10, 5/*mean (0.112798), correlation (0.536582)*/,
		5, 8, 12, 11/*mean (0.114181), correlation (0.555793)*/,
		8, 9, 9, -6/*mean (0.117431), correlation (0.553763)*/,
		7, -4, 8, -12/*mean (0.118522), correlation (0.553452)*/,
		-10, 4, -10, 9/*mean (0.12094), correlation (0.554785)*/,
		7, 3, 12, 4/*mean (0.122582), correlation (0.555825)*/,
		9, -7, 10, -2/*mean (0.124978), correlation (0.549846)*/,
		7, 0, 12, -2/*mean (0.127002), correlation (0.537452)*/,
		-1, -6, 0, -11/*mean (0.127148), correlation (0.547401)*/
	};

	ORBextractor::ORBextractor(int numFeatures, float scaleFactor, int numLevels,
		int defaultFastThreshold, int minFastThreshold) :
		m_nMaxFeatureNum(numFeatures), m_fScalFactor(scaleFactor), m_nNumLevels(numLevels),
		m_nDefaultFastThreshold(defaultFastThreshold), m_nMinFastThreshold(minFastThreshold)
	{
		// 这边将每层金字塔对应的尺度因子给出
		m_fScalFactorVec.resize(m_nNumLevels);
		m_fScalFactorVec[0] = 1.0f;
		for (int i = 1; i < m_nNumLevels; i++)
		{
			m_fScalFactorVec[i] = m_fScalFactorVec[i - 1] * m_fScalFactor;
		}

		m_pyramidImageVec.resize(m_nNumLevels);

		m_numFeatureVec.resize(m_nNumLevels);
		float fFactor = 1.0f / m_fScalFactor;
		const float EPSINON = 0.000001;
		float x = 1 - (float)pow((double)fFactor, (double)m_nNumLevels);
		float nNumFeaturesPerScal = m_nMaxFeatureNum / m_nNumLevels;
		if (abs(x) > EPSINON)//x不为0的时候执行，防止用户给出的尺度因子为1
		{
			nNumFeaturesPerScal = m_nMaxFeatureNum * (1 - fFactor) / x;
		}

		int nSumFeatures = 0;
		for (int level = 0; level < m_nNumLevels - 1; level++)
		{
			m_numFeatureVec[level] = cvRound(nNumFeaturesPerScal);
			nSumFeatures += m_numFeatureVec[level];
			nNumFeaturesPerScal *= fFactor;
		}
		m_numFeatureVec[m_nNumLevels - 1] = std::max(m_nMaxFeatureNum - nSumFeatures, 0);
		// 以上，主要目的就是金字塔图像每层检测的特征数进行均匀控制

		// 复制训练的模板
		const int nNumPoints = 512;
		const cv::Point* pattern0 = (const cv::Point*)bit_pattern_31_;
		std::copy(pattern0, pattern0 + nNumPoints, std::back_inserter(m_patternVec));

		//用于计算特征方向时，每个v坐标对应最大的u坐标
		m_uMaxVec.resize(HALF_PATCH_SIZE + 1);
		// 将v坐标划分为两部分进行计算，主要为了确保计算特征主方向的时候，x,y方向对称
		int v, v0, vMax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
		int vMin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
		// 通过勾股定理计算
		const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
		for (v = 0; v <= vMax; ++v)
			m_uMaxVec[v] = cvRound(sqrt(hp2 - v * v));

		// 确保对称，即保证是一个圆
		for (v = HALF_PATCH_SIZE, v0 = 0; v >= vMin; --v)
		{
			while (m_uMaxVec[v0] == m_uMaxVec[v0 + 1])
				++v0;
			m_uMaxVec[v] = v0;
			++v0;
		}
	}

	static void computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& umax)
	{
		for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
			keypoint_end = keypoints.end(); keypoint != keypoint_end; ++keypoint)
		{
			keypoint->angle = Intensity_Centroid_Angle(image, keypoint->pt, umax);
		}
	}

	void ExtractorNode::divideNode(ExtractorNode &node1, ExtractorNode &node2, ExtractorNode &node3, ExtractorNode &node4)
	{
		const int half_x = ceil(static_cast<float>(m_pointUR.x - m_pointUL.x) / 2);
		const int half_y = ceil(static_cast<float>(m_pointBR.y - m_pointUL.y) / 2);

		//定义节点边界
		node1.m_pointUL = m_pointUL;
		node1.m_pointUR = cv::Point2i(m_pointUL.x + half_x, m_pointUL.y);
		node1.m_pointBL = cv::Point2i(m_pointUL.x, m_pointUL.y + half_y);
		node1.m_pointBR = cv::Point2i(m_pointUL.x + half_x, m_pointUL.y + half_y);
		node1.m_keypointsVec.reserve(m_keypointsVec.size());

		node2.m_pointUL = node1.m_pointUR;
		node2.m_pointUR = m_pointUR;
		node2.m_pointBL = node1.m_pointBR;
		node2.m_pointBR = cv::Point2i(m_pointUR.x, m_pointUL.y + half_y);
		node2.m_keypointsVec.reserve(m_keypointsVec.size());

		node3.m_pointUL = node1.m_pointBL;
		node3.m_pointUR = node1.m_pointUR;
		node3.m_pointBL = m_pointBL;
		node3.m_pointBR = cv::Point2i(node1.m_pointBR.x, m_pointBL.y);
		node3.m_keypointsVec.reserve(m_keypointsVec.size());

		node4.m_pointUL = node3.m_pointUR;
		node4.m_pointUR = node2.m_pointBR;
		node4.m_pointBL = node3.m_pointBR;
		node4.m_pointBR = m_pointBR;
		node4.m_keypointsVec.reserve(m_keypointsVec.size());

		//将特征点按象限进行划分
		for (size_t i = 0; i < m_keypointsVec.size(); i++)
		{
			const cv::KeyPoint &kp = m_keypointsVec[i];
			if (kp.pt.x < node1.m_pointUR.x)
			{
				if (kp.pt.y < node1.m_pointBR.y)
					node1.m_keypointsVec.push_back(kp);
				else
					node3.m_keypointsVec.push_back(kp);
			}
			else if (kp.pt.y < node1.m_pointBR.y)
				node2.m_keypointsVec.push_back(kp);
			else
				node4.m_keypointsVec.push_back(kp);
		}

		if (node1.m_keypointsVec.size() == 1)
			node1.m_bNoMore = true;
		if (node2.m_keypointsVec.size() == 1)
			node2.m_bNoMore = true;
		if (node3.m_keypointsVec.size() == 1)
			node3.m_bNoMore = true;
		if (node4.m_keypointsVec.size() == 1)
			node4.m_bNoMore = true;

	}

	std::vector<cv::KeyPoint> ORBextractor::distributeQuadTree(const std::vector<cv::KeyPoint>& rawKeyPoints,
		const int &xMin,const int &xMax, const int &yMin, const int &yMax, const int &numFeatures, const int &level)
	{
		// 计算初始时有几个节点
		const int init_node_num = round(static_cast<float>(xMax - xMin) / (yMax - yMin));
		// 得到节点之间的间隔
		const float interval_x = static_cast<float>(xMax - xMin) / init_node_num;

		std::vector<ExtractorNode*> initNodesVec;
		initNodesVec.resize(init_node_num);
		// 划分之后包含的节点
		std::list<ExtractorNode> listNodesVec;
		for (int i = 0; i < init_node_num; i++)
		{
			ExtractorNode ni;
			ni.m_pointUL = cv::Point2i(interval_x*static_cast<float>(i), 0);
			ni.m_pointUR = cv::Point2i(interval_x*static_cast<float>(i + 1), 0);
			ni.m_pointBL = cv::Point2i(ni.m_pointUL.x, yMax - yMin);
			ni.m_pointBR = cv::Point2i(ni.m_pointUR.x, yMax - yMin);
			ni.m_keypointsVec.reserve(rawKeyPoints.size());

			listNodesVec.push_back(ni);
			initNodesVec[i] = &listNodesVec.back();
		}

		//将点分配给子节点
		for (size_t i = 0; i < rawKeyPoints.size(); i++)
		{
			const cv::KeyPoint &kp = rawKeyPoints[i];
			initNodesVec[kp.pt.x / interval_x]->m_keypointsVec.push_back(kp);
		}

		std::list<ExtractorNode>::iterator iter = listNodesVec.begin();

		while (iter != listNodesVec.end())
		{
			// 如果只含一个特征点的时候，则不再划分
			if (iter->m_keypointsVec.size() == 1)
			{
				iter->m_bNoMore = true;
				iter++;
			}
			else if (iter->m_keypointsVec.empty())
				iter = listNodesVec.erase(iter);
			else
				iter++;
		}

		bool bFinish = false;

		int iteration = 0;

		std::vector<std::pair<int, ExtractorNode*> > numberAndNodes;//节点及对应包含的特征数
		numberAndNodes.reserve(listNodesVec.size() * 4);

		while (!bFinish)
		{
			iteration++;
			// 初始节点个数，用于判断是否节点再一次进行了划分
			int nPreSize = listNodesVec.size();

			iter = listNodesVec.begin();
			// 表示节点分解次数
			int nExpandNumber = 0;

			numberAndNodes.clear();

			while (iter != listNodesVec.end())
			{
				if (iter->m_bNoMore)
				{
					// 表面只有一个特征点，则不再划分
					iter++;
					continue;
				}
				else
				{
					// 如果超过一个特征点，则继续划分
					ExtractorNode n1, n2, n3, n4;
					iter->divideNode(n1, n2, n3, n4);

					// 对划分之后的节点进行判断，是否含有特征点，含有特征点则添加节点
					if (n1.m_keypointsVec.size() > 0)
					{
						listNodesVec.push_front(n1);
						if (n1.m_keypointsVec.size() > 1)
						{
							nExpandNumber++;
							numberAndNodes.push_back(std::make_pair(n1.m_keypointsVec.size(), &listNodesVec.front()));
							listNodesVec.front().m_iterNode = listNodesVec.begin();
						}
					}
					if (n2.m_keypointsVec.size() > 0)
					{
						listNodesVec.push_front(n2);
						if (n2.m_keypointsVec.size() > 1)
						{
							nExpandNumber++;
							numberAndNodes.push_back(std::make_pair(n2.m_keypointsVec.size(), &listNodesVec.front()));
							listNodesVec.front().m_iterNode = listNodesVec.begin();
						}
					}
					if (n3.m_keypointsVec.size() > 0)
					{
						listNodesVec.push_front(n3);
						if (n3.m_keypointsVec.size() > 1)
						{
							nExpandNumber++;
							numberAndNodes.push_back(std::make_pair(n3.m_keypointsVec.size(), &listNodesVec.front()));
							listNodesVec.front().m_iterNode = listNodesVec.begin();
						}
					}
					if (n4.m_keypointsVec.size() > 0)
					{
						listNodesVec.push_front(n4);
						if (n4.m_keypointsVec.size() > 1)
						{
							nExpandNumber++;
							numberAndNodes.push_back(std::make_pair(n4.m_keypointsVec.size(), &listNodesVec.front()));
							listNodesVec.front().m_iterNode = listNodesVec.begin();
						}
					}

					iter = listNodesVec.erase(iter);
					continue;
				}
			}

			// 当节点个数大于需分配的特征数或者所有的节点只有一个特征点（节点不能划分）的时候，则结束。
			if ((int)listNodesVec.size() >= numFeatures || (int)listNodesVec.size() == nPreSize)
			{
				bFinish = true;
			}
			else if (((int)listNodesVec.size() + nExpandNumber * 3) > numFeatures)//节点展开次数乘以3用于表明下一次的节点分解可能超过特征数，即为最后一次分解
			{
				while (!bFinish)
				{
					nPreSize = listNodesVec.size();

					std::vector<std::pair<int, ExtractorNode*> > previousNumberAndNods = numberAndNodes;
					numberAndNodes.clear();

					sort(previousNumberAndNods.begin(), previousNumberAndNods.end());
					for (int j = previousNumberAndNods.size() - 1; j >= 0; j--)
					{
						ExtractorNode n1, n2, n3, n4;
						previousNumberAndNods[j].second->divideNode(n1, n2, n3, n4);

						// 划分之后进一步的判断
						if (n1.m_keypointsVec.size() > 0)
						{
							listNodesVec.push_front(n1);
							if (n1.m_keypointsVec.size() > 1)
							{
								numberAndNodes.push_back(std::make_pair(n1.m_keypointsVec.size(), &listNodesVec.front()));
								listNodesVec.front().m_iterNode = listNodesVec.begin();
							}
						}
						if (n2.m_keypointsVec.size() > 0)
						{
							listNodesVec.push_front(n2);
							if (n2.m_keypointsVec.size() > 1)
							{
								numberAndNodes.push_back(std::make_pair(n2.m_keypointsVec.size(), &listNodesVec.front()));
								listNodesVec.front().m_iterNode = listNodesVec.begin();
							}
						}
						if (n3.m_keypointsVec.size() > 0)
						{
							listNodesVec.push_front(n3);
							if (n3.m_keypointsVec.size() > 1)
							{
								numberAndNodes.push_back(std::make_pair(n3.m_keypointsVec.size(), &listNodesVec.front()));
								listNodesVec.front().m_iterNode = listNodesVec.begin();
							}
						}
						if (n4.m_keypointsVec.size() > 0)
						{
							listNodesVec.push_front(n4);
							if (n4.m_keypointsVec.size() > 1)
							{
								numberAndNodes.push_back(std::make_pair(n4.m_keypointsVec.size(), &listNodesVec.front()));
								listNodesVec.front().m_iterNode = listNodesVec.begin();
							}
						}

						listNodesVec.erase(previousNumberAndNods[j].second->m_iterNode);

						if ((int)listNodesVec.size() >= numFeatures)
							break;
					}

					if ((int)listNodesVec.size() >= numFeatures || (int)listNodesVec.size() == nPreSize)
						bFinish = true;

				}
			}
		}

		// 保留每个节点下最好的特征点
		std::vector<cv::KeyPoint> bestKeypointsVec;
		bestKeypointsVec.reserve(m_nMaxFeatureNum);
		for (std::list<ExtractorNode>::iterator lit = listNodesVec.begin(); lit != listNodesVec.end(); lit++)
		{
			std::vector<cv::KeyPoint> &node_keys = lit->m_keypointsVec;
			cv::KeyPoint* keypoint = &node_keys[0];
			float max_response = keypoint->response;

			for (size_t k = 1; k < node_keys.size(); k++)
			{
				if (node_keys[k].response > max_response)
				{
					keypoint = &node_keys[k];
					max_response = node_keys[k].response;
				}
			}

			bestKeypointsVec.push_back(*keypoint);
		}

		return bestKeypointsVec;
	}

	void ORBextractor::computeKeyPointsQuadTree(std::vector<std::vector<cv::KeyPoint> >& allkeypoints)
	{
		allkeypoints.resize(m_nNumLevels);
		// 设置格子大小
		const float fBorderWidth = 30;

		for (int level = 0; level < m_nNumLevels; ++level)
		{
			// 得到每一层图像进行特征检测区域上下两个坐标
			const int minBorder_x = EDGE_THRESHOLD - 3;
			const int minBorder_y = minBorder_x;
			const int maxBorder_x = m_pyramidImageVec[level].cols - EDGE_THRESHOLD + 3;
			const int maxBorder_y = m_pyramidImageVec[level].rows - EDGE_THRESHOLD + 3;
			// 用于分配的关键点
			std::vector<cv::KeyPoint> toDistributeKeysVec;
			toDistributeKeysVec.reserve(m_nMaxFeatureNum * 10);

			const float width = (maxBorder_x - minBorder_x);
			const float height = (maxBorder_y - minBorder_y);
			// 将待检测区域划分为格子的行列数
			const int cols = width / fBorderWidth;
			const int rows = height / fBorderWidth;
			// 重新计算每个格子的大小
			const int width_cell = ceil(width / cols);
			const int height_cell = ceil(height / rows);
			// 在每个格子内进行fast特征检测
			for (int i = 0; i < rows; i++)
			{
				const float ini_y = minBorder_y + i*height_cell;
				float max_y = ini_y + height_cell + 6;

				if (ini_y >= maxBorder_y - 3)
					continue;
				if (max_y > maxBorder_y)
					max_y = maxBorder_y;

				for (int j = 0; j < cols; j++)
				{
					const float ini_x = minBorder_x + j*width_cell;
					float max_x = ini_x + width_cell + 6;
					if (ini_x >= maxBorder_x - 6)
						continue;
					if (max_x > maxBorder_x)
						max_x = maxBorder_x;

					std::vector<cv::KeyPoint> keysVec;
					cv::FAST(m_pyramidImageVec[level].rowRange(ini_y, max_y).colRange(ini_x, max_x),
						keysVec, m_nDefaultFastThreshold, true);
					// 如果检测到的fast特征为空，则降低阈值再进行检测
					if (keysVec.empty())
					{
						cv::FAST(m_pyramidImageVec[level].rowRange(ini_y, max_y).colRange(ini_x, max_x),
							keysVec, m_nMinFastThreshold, true);
					}
					// 计算实际特征点的位置
					if (!keysVec.empty())
					{
						for (std::vector<cv::KeyPoint>::iterator vit = keysVec.begin(); vit != keysVec.end(); vit++)
						{
							(*vit).pt.x += j*width_cell;
							(*vit).pt.y += i*height_cell;
							toDistributeKeysVec.push_back(*vit);
						}
					}

				}
			}

			std::vector<cv::KeyPoint> & keypoints = allkeypoints[level];
			keypoints.reserve(m_nMaxFeatureNum);
			// 将特征点进行四叉树划分
			keypoints = distributeQuadTree(toDistributeKeysVec, minBorder_x, maxBorder_x,
				minBorder_y, maxBorder_y, m_numFeatureVec[level], level);

			const int scaled_patch_size = PATCH_SIZE * m_fScalFactorVec[level];

			// 换算特征点真实位置（添加边界值），添加特征点的尺度信息
			const int nkps = keypoints.size();
			for (int i = 0; i < nkps; i++)
			{
				keypoints[i].pt.x += minBorder_x;
				keypoints[i].pt.y += minBorder_y;
				keypoints[i].octave = level;
				keypoints[i].size = scaled_patch_size;
			}
		}

		// 计算特征点的方向
		for (int level = 0; level < m_nNumLevels; ++level)
			computeOrientation(m_pyramidImageVec[level], allkeypoints[level], m_uMaxVec);
	}


	static void computeDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
		const std::vector<cv::Point>& pattern)
	{
		descriptors = cv::Mat::zeros((int)keypoints.size(), 32, CV_8UC1);

		for (size_t i = 0; i < keypoints.size(); i++)
			computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
	}

	void ORBextractor::operator()(cv::InputArray image_array, cv::InputArray mask_array, std::vector<cv::KeyPoint>& output_keypoints,
		cv::OutputArray descriptors_array)
	{
		if (image_array.empty())
			return;

		cv::Mat image = image_array.getMat();
		//assert(image.type() == CV_8UC1);
		if (image.type() != CV_8UC1)
			cvtColor(image, image, cv::COLOR_BGR2GRAY);

		// 构建金字塔图像
		computePyramid(image);

		std::vector < std::vector<cv::KeyPoint> > all_keypoints;
		computeKeyPointsQuadTree(all_keypoints);
		//computeKeyPointsOld(all_keypoints);

		cv::Mat descriptors;

		int keypoints_num = 0;
		for (int level = 0; level < m_nNumLevels; ++level)
			keypoints_num += (int)all_keypoints[level].size();
		if (keypoints_num == 0)
			descriptors_array.release();
		else
		{
			descriptors_array.create(keypoints_num, 32, CV_8U);
			descriptors = descriptors_array.getMat();
		}

		output_keypoints.clear();
		output_keypoints.reserve(keypoints_num);

		int offset = 0;//存储描述子的偏移量，用于分割不同尺度层
		for (int level = 0; level < m_nNumLevels; ++level)
		{
			std::vector<cv::KeyPoint>& keypoints = all_keypoints[level];
			int numKeypointsLevel = (int)keypoints.size();//当前尺度下图像特征点的数目

			if (numKeypointsLevel == 0)
				continue;

			// 处理当前尺度下的图像
			cv::Mat working_mat = m_pyramidImageVec[level].clone();
			GaussianBlur(working_mat, working_mat, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);

			// 计算描述子
			cv::Mat desc = descriptors.rowRange(offset, offset + numKeypointsLevel);
			computeDescriptors(working_mat, keypoints, desc, m_patternVec);

			offset += numKeypointsLevel;

			// 特征点坐标进行尺度处理，换算到当前图像中
			if (level != 0)
			{
				float scale = m_fScalFactorVec[level]; //getScale(level, firstLevel, scale_factor_);
				for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
					keypoint_end = keypoints.end(); keypoint != keypoint_end; ++keypoint)
					keypoint->pt *= scale;
			}
			// 将完成好的特征点添加到输出vector中
			output_keypoints.insert(output_keypoints.end(), keypoints.begin(), keypoints.end());
		}
	}

	void ORBextractor::computePyramid(cv::Mat image)
	{
		for (int level = 0; level < m_nNumLevels; ++level)
		{
			float scale = 1.0f / m_fScalFactorVec[level];
			cv::Size sz(cvRound((float)image.cols * scale), cvRound((float)image.rows * scale));

			if (level != 0)
			{
				resize(m_pyramidImageVec[level - 1], m_pyramidImageVec[level], sz, 0, 0, cv::INTER_LINEAR);
			}
			else
			{
				m_pyramidImageVec[level] = image;
			}
		}

	}
}

