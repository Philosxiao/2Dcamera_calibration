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

#include "frameCapture.h"

using namespace cv;
using namespace std;

Eigen::Matrix<double, 4, 4> final_transform;

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

void onMouse(int event, int x, int y, int flags, void *param)
{
    cv::Mat *im = reinterpret_cast<cv::Mat *>(param);
    Eigen::Matrix<double, 4, 1> test_point;
    test_point << x, y, 0, 1;
    Eigen::Matrix<double, 4, 1> result_point = final_transform * test_point;
    switch (event)
    {
    case CV_EVENT_LBUTTONDOWN:
        cout << "at(" << x << "," << y << "), robot system coordinates are:" << result_point[0] << " " << result_point[1] << " " << result_point[2]
             << endl;

        break;
    }
}

int main(int argc, char *argv[])
{
    //启动相机
    FrameCapture kinect_cam;
    cv::namedWindow("point_in", WINDOW_AUTOSIZE);

    while (1)
    {
        kinect_cam.capture();

        Mat frame_now = kinect_cam.getRgbMat();
        //Mat aligned_depth_image = kinect_cam.getDepthMat();

        //cout<<frame_now<<endl;
        if (frame_now.empty())
        {
            cout << "no image capture!" << endl;
        }
        Mat source_image = frame_now;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(source_image, dictionary, corners, ids);
        Mat imageCopy;
        source_image.copyTo(imageCopy);
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }

        for (int i = 0; i < corners.size(); i++)
        {
            circle(imageCopy, Point(corners[i][0].x, corners[i][0].y), 2, (0, 0, 255), -1);
        }

        int num_point;
        // double scale = 1;
        // Size ResImgSiz = Size(imageCopy.cols * scale, imageCopy.rows * scale);
        // Mat ResImg = Mat(ResImgSiz, imageCopy.type());
        // resize(imageCopy, ResImg, ResImgSiz, CV_INTER_CUBIC);
        cv::setMouseCallback("point_in", onMouse, reinterpret_cast<void *>(&imageCopy));
        imshow("point_in", imageCopy);
        int key;
        key = waitKey(1000 / 30);
        if (27 == (char)key)
        {
            if (ids.size() != 24)
            {
                cout << " points are not enough!"<<endl;
                continue;
            }
            cout << "updating transformation...";
            std::ifstream infile_feat("../point_robot"); //加载数据文件
            cout << "read end" << endl;
            std::string feature;                    //存储读取的每行数据
            double feat_onePoint;                   //存储每行按空格分开的每一个float数据
            std::vector<double> lines;              //存储每行数据
            std::vector<vector<double>> lines_feat; //存储所有数据
            lines.clear();
            lines_feat.clear();
            cout << "readed" << std::endl;
            int count_time = 0;
            std::cout << count_time;

            while (!infile_feat.eof())
            {

                if (count_time > 23)
                    break;
                count_time++;

                getline(infile_feat, feature);  //一次读取一行数据
                stringstream stringin(feature); //使用串流实现对string的输入输出操作
                while (stringin >> feat_onePoint)
                {                                   //按空格一次读取一个数据存入feat_onePoint
                    lines.push_back(feat_onePoint); //存储每行按空格分开的数据
                    std::cout << feat_onePoint << " ";
                }
                cout << feature << endl;
                lines_feat.push_back(lines); //存储所有数据
                lines.clear();
            }

            cout << lines_feat.size() << endl;
            infile_feat.close();
            double maxdis_in_robot = getDistance(lines_feat[0], lines_feat[23]);

            int min_num = 0;
            int max_num = ids.size();
            for (int i = 0; i < ids.size(); i++)
            {
                if (ids[i] == 0)
                    min_num = i;
                if (ids[i] == 23)
                    max_num = i;
            }

            vector<double> point0;
            point0.push_back(corners[min_num][0].x);
            point0.push_back(corners[min_num][0].y);
            point0.push_back(0);

            vector<double> point23;
            point23.push_back(corners[max_num][0].x);
            point23.push_back(corners[max_num][0].y);
            point23.push_back(0);

            double maxdis_in_image = getDistance(point0, point23);

            double ratio_image2robot = maxdis_in_robot / maxdis_in_image;
            ////////////////////////////////

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

            cloud_in->width = corners.size();
            cloud_in->height = 1;
            cloud_in->is_dense = false;
            cloud_in->resize(cloud_in->width * cloud_in->height);

            cloud_out->width = lines_feat.size();
            cloud_out->height = 1;
            cloud_out->is_dense = false;
            cloud_out->resize(cloud_out->width * cloud_out->height);

            for (int j = 0; j < cloud_in->points.size(); j++)
            {
                cloud_in->points[j].x = corners[j][0].x * ratio_image2robot;
                cloud_in->points[j].y = corners[j][0].y * ratio_image2robot;
                cloud_in->points[j].z = 0;
            }

            for (int j = 0; j < cloud_out->points.size(); j++)
            {
                cloud_out->points[j].x = lines_feat[ids[j]][0];
                cloud_out->points[j].y = lines_feat[ids[j]][1];
                cloud_out->points[j].z = lines_feat[ids[j]][2];
            }

            //利用SVD方法求解变换矩阵
            pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
            pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
            TESVD.estimateRigidTransformation(*cloud_in, *cloud_out, transformation2);
            Eigen::Matrix<double, 4, 4> transform;
            transform << (double)transformation2(0, 0), (double)transformation2(0, 1), (double)transformation2(0, 2), (double)transformation2(0, 3),
                (double)transformation2(1, 0), (double)transformation2(1, 1), (double)transformation2(1, 2), (double)transformation2(1, 3),
                (double)transformation2(2, 0), (double)transformation2(2, 1), (double)transformation2(2, 2), (double)transformation2(2, 3),
                0.0, 0.0, 0, 1;
            Eigen::Matrix<double, 4, 4> scalr_change;
            scalr_change << ratio_image2robot, 0.0, 0.0, 0.0,
                0.0, ratio_image2robot, 0.0, 0.0,
                0.0, 0.0, ratio_image2robot, 0.0,
                0.0, 0.0, 0, 1;
            final_transform = transform * scalr_change;

            cout << "new final transform:" << endl
                 << final_transform << endl;
        }

        // Eigen::Matrix<double, 4, 1> test_point;
        // test_point << source_image.cols / 2, source_image.rows / 2, 0, 1;

        // Eigen::Matrix<double, 4, 1> result_point = final_transform * test_point;

        // ofstream outfile;
        // outfile.open("test_cor.txt", ios::binary | ios::app | ios::in | ios::out);
        // outfile << result_point[0] << " " << result_point[1] << " " << result_point[2] << "\n";
        // outfile.close(); //关闭文件，保存文件。
        ////////////////////////

        //std::cout << "good" << maxdis_in_robot << " " << maxdis_in_image;
    }
    return 1;
    //}
}
