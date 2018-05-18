/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

namespace ORB_SLAM2
{

class Viewer;  //绘图视图类
class FrameDrawer;  //绘制帧类
class Map;  //图操作类
class Tracking;  //追踪类
class LocalMapping;  //局部建图类
class LoopClosing;  //闭环检测类

class System  //系统类
{
public:
    // Input sensor
    //选择传感器类型：单目、双目、深度相机
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    //初始化slam系统，它包含局部建图、闭环检测、显示地图三个线程
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    //处理单目帧（彩色或灰度）追踪相机
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    // This stops local mapping thread (map building) and performs only camera tracking.
    /// 暂停建图，单独执行相机追踪
    void ActivateLocalizationMode();
    // This resumes local mappig thread and performs SLAM again.
    /// 重新开启局部地图的线程
    /// 在这里使用的是mutex信号量的多线程编程
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    /// 从上次调用函数开始，如果回环检测和全局BA有大的改变则返回true
    bool MapChanged();

    // Reset the system (clear map)
    /// 复位清晰图
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    /// 等到所有线程结束任务的时候关闭每个线程
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    /// kitti数据集的保存位姿的方法
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    /// 从最近处理帧的信息追踪后调用该函数。
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

private:

    // Input sensor
    ///传感器
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ///对于场景识别和特征匹配的ORB字典
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    ///场景识别中的关键帧的数据
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    ///map结构存储所有关键帧和map点云
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    /// 跟踪器
    /// 通过接受帧去计算相关相机的位姿。判断何时加入新的关键帧时，创建新的点云。
    /// 当追踪失败进行重定位。
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    /// 局部建图器
    /// 进行局部图建和局部的BA
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    /// 闭环检测器
    /// 每插入一个关键帧就计算是否有闭环并且进行全局的BA
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    /// 用Pangolin去建图和显示相机位姿。
    Viewer* mpViewer;

    /// 关键帧和建图
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    /// 系统线程 ：局部建图、闭环检测、视图、追踪（在主函数中）
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    /// 复位标志
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    /// 更改模式标志
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;  // 激活
    bool mbDeactivateLocalizationMode;  //停用

    // Tracking state
    /// 追踪状态
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H

//-----------------------#学习到的一些知识点

/** # namespace
 命名空间是ANSIC++引入的可以由用户命名的作用域，用来处理程序中 常见的同名冲突

 <问题提出>
 c语言中的3层作用域：文件（编译单元）、函数、复合语句；
 c++中的类作用域，类是出现在文件内。
 在不同的作用域中可以定义相同名字的变量，互不于扰，系统能够区别它们。
 全局变量的作用域是整个程序，在同一作用域中不应有两个或多个同名的实体(enuty)，包括变量、函数和类等。

 <问题解决>
 namespace nsl
 { const int RATE=0.08； //常量
 doublepay； //变量
 doubletax() //函数
 {return a*RATE；}
 namespacens2 //嵌套的命名空间
 {int age；}
 }
  现在命名空间成员包括变量doublepay，注意doublepay仍然是全局变量，仅仅是把它们隐藏在指定的命名空间中而已。
  如果在程序中要使用变量a和b，必须加上命名空间名和作用域分辨符“::”，如nsl::a，nsl::b。
  这种用法称为命名空间限定(qualified)，这些名字(如nsl::a)称为被限定名 (qualified name)。

 <使用指南>
 using namespace std;
 #include
 #include
 namespace ns1 //声明命名空间ns1
 {
 class Student //在命名空间nsl内声明Student类
 { public:
      tudent(int n,string nam,int a)
      { num=n;name=nam;age=a;}
      void get_data();
   private:
      int num;
      string name;
      int age;
  };
 void Student::get_data() //定义成员函数
 }

 <使用成员函数u>
 1.使用命名空间别名
 namespace Television //声明命名空间，名为Television
 { … }可以用一个较短而易记的别名代替它。如：
 namespace TV=Television； //别名TV与原名Television等价

 2.使用using命名空间成员名
 sing后面的命名空间成员名必须是由命名空间限定的名字。例如：
 using nsl::Studen；
 */