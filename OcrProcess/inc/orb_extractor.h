#include <vector>
#include <list>
#include <opencv/cv.h>
#include <iterator>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <highgui.hpp>



namespace orb
{
	/* 一个图像区域的象限划分：:
	UL_(1)   |    UR_(0)
	---------|-----------
	BL_(2)   |    BR_(3)
	*/
	class  ExtractorNode
	{
	public:
		ExtractorNode() :m_bNoMore(false){}

		/** \brief 划分四叉树
		*/
		void divideNode(ExtractorNode &node1, ExtractorNode &node2, ExtractorNode &node3, ExtractorNode &node4);

	public:
		std::vector<cv::KeyPoint> m_keypointsVec;//vec_keys_;
		cv::Point2i m_pointUL, m_pointUR, m_pointBL, m_pointBR;//UL_, UR_, BL_, BR_;
		std::list<ExtractorNode>::iterator m_iterNode;
		bool m_bNoMore;
	};

	class  ORBextractor
	{
	public:

		ORBextractor(int numFeatures = 1000, float scaleFactor = 1.2f, int numLevels = 4,
			int defaultFastThreshold = 20, int minFastThreshold = 7);

		~ORBextractor(){}

		/** \brief 计算图像的orb特征及描述，将orb特征分配到一个四叉树当中
		*  目前mask参数是被忽略的，没有实现
		*/
		void operator()(cv::InputArray image, cv::InputArray mask,
			std::vector<cv::KeyPoint>& keypoints,
			cv::OutputArray descriptors);

		/** \brief 得到高斯金字塔的层数
		*/
		inline int getLevels()
		{
			return m_nMaxFeatureNum;
		}
		/** \brief 得到金字塔图像之间的尺度参数
		*/
		inline float getScaleFactor()
		{
			return m_fScalFactor;
		}
		/** \brief 得到金字塔图像每层的尺度因子
		*/
		inline std::vector<float> getScaleFactors()
		{
			return m_fScalFactorVec;
		}

	protected:
		/** \brief 计算图像金字塔
		*/
		void computePyramid(cv::Mat image);
		/** \brief 通过四叉树的方式计算特征点
		*/
		void computeKeyPointsQuadTree(std::vector<std::vector<cv::KeyPoint> >& allkeypoints);
		/** \brief 通过四叉树的方式分配特征点
		*/
		std::vector<cv::KeyPoint> distributeQuadTree(const std::vector<cv::KeyPoint>& rawKeyPoints,
			const int &xMin, const int &xMax, const int &yMin, const int &yMax, const int &numFeatures, const int &level);

		//void computeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& all_keypoints);

	public:
		std::vector<cv::Mat> m_pyramidImageVec;//图像金字塔

	protected:
		std::vector<cv::Point> m_patternVec;//用于存放训练的模板

		int m_nMaxFeatureNum;//最多提取的特征点的数量
		int m_nNumLevels;//高斯金字塔的层数
		float m_fScalFactor;//金字塔图像之间的尺度参数
		std::vector<float> m_fScalFactorVec;//用于存储每层的尺度因子
			
		int m_nDefaultFastThreshold;//默认设置fast角点阈值20
		int m_nMinFastThreshold;//设置fast角点阈值为9

		std::vector<int> m_numFeatureVec;//每层特征的个数

		std::vector<int> m_uMaxVec;// 用于存储计算特征方向时，图像每个v坐标对应最大的u坐标
			
	};
}


