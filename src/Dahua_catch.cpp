//#include "camset.h"
#include <Eigen/QR>
#include <Eigen/Dense>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/ppf.h>
#include <pcl/common/transforms.h>
// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/aruco.hpp>

#include <vector>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include "camset.h"


using namespace cv;
using namespace std;

template <typename T>
double getDistance(T pointO, T pointA)
{
    double distance;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
    distance = sqrtf(distance);
    return distance;
}

template <typename T>
double getDistance(vector<T> pointO, vector<T> pointA)
{
    double distance;
    distance = powf((pointO[0] - pointA[0]), 2) + powf((pointO[1] - pointA[1]), 2) + powf((pointO[2] - pointA[2]), 2);
    distance = sqrtf(distance);
    return distance;
}

int main(int argc, char *argv[])
{
    ICameraPtr cameraSptr;

    /* 发现设备 */
    CSystem &systemObj = CSystem::getInstance();
    TVector<ICameraPtr> vCameraPtrList;
    bool isDiscoverySuccess = systemObj.discovery(vCameraPtrList);
    if (!isDiscoverySuccess)
    {
        printf("discovery device fail.\n");
        return 0;
    }

    if (vCameraPtrList.size() == 0)
    {
        printf("no devices.\n");
        getchar();
    }

    /* 2、打开相机 */
    if (!vCameraPtrList[0]->connect())
    {
        printf("connect cameral failed.\n");
        return 0;
    }

    /* 1、枚举所有相机信息（序列号、版本、相机厂家） */
    for (int i = 0; i < vCameraPtrList.size(); i++)
    {
        cameraSptr = vCameraPtrList[i];

        CStringNode paramDeviceVersion(cameraSptr, "DeviceVersion");
        CString strDeviceVersion;
        paramDeviceVersion.getValue(strDeviceVersion);

        printf("Camera[%d] Info :\n", i);
        printf("    key           = [%s]\n", cameraSptr->getKey());
        printf("    vendor name   = [%s]\n", cameraSptr->getVendorName());
        printf("    model         = [%s]\n", cameraSptr->getModelName());
        printf("    serial number = [%s]\n", cameraSptr->getSerialNumber());
        printf("    DeviceVersion = [%s]\n", strDeviceVersion.c_str());
    }

    cameraSptr = vCameraPtrList[0];

#if 0
            //设置相机参数值
            setGrabMode(cameraSptr, true);
            //setGrabMode(cameraSptr, false);
            //设置是否连续拍照
            bool bContious = true;
            getGrabMode(cameraSptr, bContious);
            //triggerSoftware(cameraSptr);
            //设置分辨率
            int64_t nWidth, nHeight;
            //得到最大分辨率
            getMaxResolution(cameraSptr, nWidth, nHeight);
            cout<<"最大分辨率宽:"<<nWidth<<"高:"<<(int)nHeight<<endl;

            setResolution(cameraSptr, 4000, 3000);
            //getResolution(cameraSptr, nWidth, nHeight);
            //设置偏移值
            setBinning(cameraSptr);

            //设置ROI:region of intersting
            int64_t nX, nY, nROIWidth, nROIHeight;
            setROI(cameraSptr, 120, 120, 4000, 3000);
            //getROI(cameraSptr, nX, nY, nROIWidth, nROIHeight);
            //获取采集数据的宽度和高
            getWidth(cameraSptr, nWidth);
            getHeight(cameraSptr, nHeight);
            //设置曝光时间
            double dExposureTime = 0;
            //setExposureTime(cameraSptr, 100, true);
            setExposureTime(cameraSptr, 1350, false);
            getExposureTime(cameraSptr, dExposureTime);
            //获取最小和最大曝光时间
            double dMinExposure, dMaxExposure;
            getExposureTimeMinMaxValue(cameraSptr, dMinExposure, dMaxExposure);
            //设置图像增益值
            double dGainRaw = 0;
            double dGainRawMin = 0;
            double dGainRawMax = 0;
            setGainRaw(cameraSptr, 28);
            getGainRaw(cameraSptr, dGainRaw);
            getGainRawMinMaxValue(cameraSptr, dGainRawMin, dGainRawMax);
            cout<<"最大增益:"<<dGainRawMax<<"最小增益"<<dGainRawMin<<endl;
            //设置图像伽马值
            double dGamma = 0;
            double dGammaMin = 0;
            double dGammaMax = 0;
            setGamma(cameraSptr, 0.8);
            getGamma(cameraSptr, dGamma);
            getGammaMinMaxValue(cameraSptr, dGammaMin, dGammaMax);
            //设置白平衡值
            double dRedBalanceRatio = 0;
            double dGreenBalanceRatio = 0;
            double dBlueBalanceRatio = 0;
            double dMinBalanceRatio = 0;
            double dMaxBalanceRatio = 0;
            setBalanceRatio(cameraSptr, 1.5, 1.5, 1.5);
            getBalanceRatio(cameraSptr, dRedBalanceRatio, dGreenBalanceRatio, dBlueBalanceRatio);
            getBalanceRatioMinMaxValue(cameraSptr, dMinBalanceRatio, dMaxBalanceRatio);
            //设置帧速率
            double dFrameRate = 9;
            setAcquisitionFrameRate(cameraSptr, 9);
            getAcquisitionFrameRate(cameraSptr, dFrameRate);
            //保存和加载配置
            userSetSave(cameraSptr);
            loadUserSet(cameraSptr);
            //设置外触发时间
            double dDelayTime = 0;
            setTriggerDelay(cameraSptr, 20);
            getTriggerDelay(cameraSptr, dDelayTime);
            //设置外触发模式
            bool bRisingEdge = true;
            setLineTriggerMode(cameraSptr, bRisingEdge);
            getLineTriggerMode(cameraSptr, bRisingEdge);
            //设置外触发信号滤波时间
            double dLineDebouncerTimeAbs = 0;
            setLineDebouncerTimeAbs(cameraSptr, 20);
            getLineDebouncerTimeAbs(cameraSptr, dLineDebouncerTimeAbs);
            //设置外部光源曝光时间（设置输出值为TRUE的时间）
            setOutputTime(cameraSptr, 1000);
            //设置采图模式是否连续
            setGrabMode(cameraSptr, true);
            //是否X翻转
            setReverseX(cameraSptr, false);
            //是否Y翻转
            setReverseY(cameraSptr, false);
#endif

    /* 7、开始采图 */
    IStreamSourcePtr streamPtr = systemObj.createStreamSource(cameraSptr);
    if (NULL == streamPtr)
    {
        printf("create stream obj  fail.\r\n");
        return 0;
    }
    streamPtr->setBufferCount(2);
    bool isStartGrabbingSuccess = streamPtr->startGrabbing();
    if (!isStartGrabbingSuccess)
    {
        printf("StartGrabbing  fail.\n");
    }

    /* 拉流 */
    Dahua::Memory::TSharedPtr<StreamRetrieve> streamThreadSptr(new StreamRetrieve(streamPtr));
    if (NULL == streamThreadSptr)
    {
        printf("create thread obj failed.\n");
        return 0;
    }

    streamThreadSptr->start();
    //从采图程序中转化

    while (streamPtr->isGrabbing())
    {
        CFrame frame_now;
        clock_t startTime_whole, endTime_whole;
        startTime_whole = clock(); //开始计耗时
        if (streamPtr->getFrame(frame_now))
        {

            //streamPtr->stopGrabbing();
            clock_t startTime, endTime;
            startTime = clock(); //开始计耗时
            const void *date = frame_now.getImage();
            size_t size_num = frame_now.getImageSize();
            //cout << "image size = " << size_num << endl;
            void *new_date = (void *)malloc(sizeof(void) * size_num); //分配size_num个void型存储单元，并将首地址存储到指针变量pd中

            memcpy(new_date, date, size_num);
            Mat source_image = Mat(frame_now.getImageHeight(), frame_now.getImageWidth(), CV_8UC3, new_date);
            Mat outImg;
            cv::resize(source_image, outImg, cv::Size(source_image.cols * 0.5,source_image.rows * 0.5));
            imshow("see",source_image);
            //Mat source_image = imread("../test.jpeg");

            // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

            // std::vector<int> ids;
            // std::vector<std::vector<cv::Point2f>> corners;

            // cv::aruco::detectMarkers(source_image, dictionary, corners, ids);

            // for (int i = 0; i < ids.size(); i++)
            //     cout << ids[i] << " ";
            // Mat imageCopy;
            // source_image.copyTo(imageCopy);

            // if (ids.size() > 0)
            // {
            //     cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
            // }

            // for (int i = 0; i < corners.size(); i++)
            // {
            //     circle(imageCopy, Point(corners[i][0].x, corners[i][0].y), 10, (255, 0, 0), -1);
            // }
            // circle(imageCopy, Point(source_image.cols / 2, source_image.rows / 2), 30, (0, 255, 0), -1);

            // int num_point;
            // cout << ids.size() << " " << corners.size() << endl;

            // double scale = 0.3;
            // Size ResImgSiz = Size(imageCopy.cols * scale, imageCopy.rows * scale);
            // Mat ResImg = Mat(ResImgSiz, imageCopy.type());
            // resize(imageCopy, ResImg, ResImgSiz, CV_INTER_CUBIC);

            // imshow("point_in", ResImg);
            // waitKey();
            // int key;
            // //key = waitKey(1000 / 30);
            // // if (27 == (char)key)
            // // {
            // std::ifstream infile_feat("../point_robot"); //加载数据文件
            // cout << "read end" << endl;
            // std::string feature;                    //存储读取的每行数据
            // double feat_onePoint;                   //存储每行按空格分开的每一个float数据
            // std::vector<double> lines;              //存储每行数据
            // std::vector<vector<double>> lines_feat; //存储所有数据
            // lines.clear();
            // lines_feat.clear();
            // cout << "readed" << std::endl;
            // int count_time = 0;
            // std::cout << count_time;

            // while (!infile_feat.eof())
            // {

            //     if (count_time > 23)
            //         break;
            //     count_time++;

            //     getline(infile_feat, feature);  //一次读取一行数据
            //     stringstream stringin(feature); //使用串流实现对string的输入输出操作
            //     while (stringin >> feat_onePoint)
            //     {                                   //按空格一次读取一个数据存入feat_onePoint
            //         lines.push_back(feat_onePoint); //存储每行按空格分开的数据
            //         std::cout << feat_onePoint << " ";
            //     }
            //     cout << feature << endl;
            //     lines_feat.push_back(lines); //存储所有数据
            //     lines.clear();
            // }

            // cout << lines_feat.size() << endl;
            // infile_feat.close();
            // std::cout << "good";
            // double maxdis_in_robot = getDistance(lines_feat[0], lines_feat[23]);

            // int min_num = 0;
            // int max_num = ids.size();
            // for (int i = 0; i < ids.size(); i++)
            // {
            //     if (ids[i] == 0)
            //         min_num = i;
            //     if (ids[i] == 23)
            //         max_num = i;
            // }

            // vector<double> point0;
            // point0.push_back(corners[min_num][0].x);
            // point0.push_back(corners[min_num][0].y);
            // point0.push_back(0);

            // vector<double> point23;
            // point23.push_back(corners[max_num][0].x);
            // point23.push_back(corners[max_num][0].y);
            // point23.push_back(0);

            // double maxdis_in_image = getDistance(point0, point23);

            // double ratio_image2robot = maxdis_in_robot / maxdis_in_image;
            // ////////////////////////////////

            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

            // cloud_in->width = corners.size();
            // cloud_in->height = 1;
            // cloud_in->is_dense = false;
            // cloud_in->resize(cloud_in->width * cloud_in->height);

            // cloud_out->width = lines_feat.size();
            // cloud_out->height = 1;
            // cloud_out->is_dense = false;
            // cloud_out->resize(cloud_out->width * cloud_out->height);

            // for (int j = 0; j < cloud_in->points.size(); j++)
            // {
            //     cloud_in->points[j].x = corners[j][0].x * ratio_image2robot;
            //     cloud_in->points[j].y = corners[j][0].y * ratio_image2robot;
            //     cloud_in->points[j].z = 0;
            // }

            // for (int j = 0; j < cloud_out->points.size(); j++)
            // {
            //     cloud_out->points[j].x = lines_feat[ids[j]][0];
            //     cloud_out->points[j].y = lines_feat[ids[j]][1];
            //     cloud_out->points[j].z = lines_feat[ids[j]][2];
            // }

            // //利用SVD方法求解变换矩阵
            // pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
            // pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
            // TESVD.estimateRigidTransformation(*cloud_in, *cloud_out, transformation2);
            // Eigen::Matrix<double, 4, 4> transform;
            // transform << (double)transformation2(0, 0), (double)transformation2(0, 1), (double)transformation2(0, 2), (double)transformation2(0, 3),
            //     (double)transformation2(1, 0), (double)transformation2(1, 1), (double)transformation2(1, 2), (double)transformation2(1, 3),
            //     (double)transformation2(2, 0), (double)transformation2(2, 1), (double)transformation2(2, 2), (double)transformation2(2, 3),
            //     0.0, 0.0, 0, 1;
            // Eigen::Matrix<double, 4, 4> scalr_change;
            // scalr_change << ratio_image2robot, 0.0, 0.0, 0.0,
            //     0.0, ratio_image2robot, 0.0, 0.0,
            //     0.0, 0.0, ratio_image2robot, 0.0,
            //     0.0, 0.0, 0, 1;
            // Eigen::Matrix<double, 4, 4> final_transform = transform * scalr_change;

            // cout << final_transform << endl;

            // Eigen::Matrix<double, 4, 1> test_point;
            // test_point << source_image.cols / 2, source_image.rows / 2, 0, 1;

            // Eigen::Matrix<double, 4, 1> result_point = final_transform * test_point;

            // ofstream outfile;
            // outfile.open("test_cor.txt", ios::binary | ios::app | ios::in | ios::out);
            // outfile << result_point[0] << " " << result_point[1] << " " << result_point[2] << "\n";
            // outfile.close(); //关闭文件，保存文件。
            // ////////////////////////

            // std::cout << "good" << maxdis_in_robot << " " << maxdis_in_image;
        }
    }

    /* 停止拉流线程 */
    streamThreadSptr->stop();

    /* 8、停止采图 */
    streamPtr->stopGrabbing();

    /* 修改相机曝光时间 */
    modifyCamralExposureTime(systemObj, cameraSptr);

    /* 3、关闭相机 */
    if (!cameraSptr->disConnect())
    {
        printf("disConnect camera failed\n");
        return 0;
    }

    printf("disConnect successfully thread ID :%d\n", CThread::getCurrentThreadID());

    return 1;
    //}
}
