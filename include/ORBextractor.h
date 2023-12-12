/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>


namespace ORB_SLAM3
{

class ExtractorNode//提取节点类
{
public:
    ExtractorNode():bNoMore(false){}//默认构造函数声明，使bNoMore为false

 /**
 * @brief 将提取器节点分成4个子节点，同时也完成图像区域的划分、特征点归属的划分，以及相关标志位的置位
 * 
 * @param[in & out] n1  提取器节点1：左上
 * @param[in & out] n2  提取器节点1：右上
 * @param[in & out] n3  提取器节点1：左下
 * @param[in & out] n4  提取器节点1：右下
 */
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;//存储关键点的容器
    cv::Point2i UL, UR, BL, BR;//像素点坐标（x，y）
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;//判断这个图像是否还可以继续分割
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };//匿名枚举，哈里斯角点则为0，fast角点则为1

//int _nfeatures,		//指定要提取的特征点数目
//float _scaleFactor,	//指定图像金字塔的缩放系数
//int _nlevels,		    //指定图像金字塔的层数
//int _iniThFAST,		//指定初始的FAST特征点提取参数，可以提取出最明显的角点
//int _minThFAST):	    //如果因为图像纹理不丰富提取出的特征点不多，为了达到想要的特征点数目，
							//就使用这个参数提取出不是那么明显的角点
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);//声明 ORBextractor函数

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
/**
 * @brief 用仿函数（重载括号运算符）方法来计算图像特征点
 * 
 * @param[in] _image                    输入原始图的图像
 * @param[in] _mask                     掩码mask
 * @param[in & out] _keypoints          存储特征点关键点的向量
 * @param[in & out] _descriptors        存储特征点描述子的矩阵
 */
    int operator()( cv::InputArray _image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

