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

cv::Mat source_image,dst,img,tmp;
void onMouse(int event, int x, int y, int flags, void *param)
{
    Eigen::Matrix<double, 4, 1> test_point;
    test_point << x, y, 0, 1;

	static Point pre_pt(-1,-1);//初始坐标
	static Point cur_pt(-1,-1);//实时坐标
	char temp[16];
	
    if (event == CV_EVENT_LBUTTONDOWN && CV_EVENT_FLAG_CTRLKEY)//按住ctrl健，鼠标左键按下，读取初始坐标，并在图像上该点处划圆
	{
		source_image.copyTo(img);//将原始图片复制到img中
		sprintf(temp,"(%d,%d)",x,y);
		pre_pt = Point(x,y);
		putText(img,temp,pre_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255),1,8);//在窗口上显示坐标
		circle(img,pre_pt,2,Scalar(255,0,0,0),CV_FILLED,CV_AA,0);//划圆
		imshow("img",img);
	}
    else if (event == CV_EVENT_RBUTTONDOWN)//不按住ctrl健，鼠标左键按下，读取初始坐标，并在图像上该点处划圆
	{
        Eigen::Matrix<double, 4, 1> result_point = final_transform * test_point;

		source_image.copyTo(img);//将原始图片复制到img中
        cout << "at(" << x << "," << y << "), robot system coordinates are:" << result_point[0] << " " << result_point[1] << " " << result_point[2]
             << endl;
		imshow("img",img);
	}
	else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))//左键没有按下的情况下鼠标移动的处理函数
	{
        source_image.copyTo(img);//将原始图片复制到img中
		img.copyTo(tmp);//将img复制到临时图像tmp上，用于显示实时坐标
		sprintf(temp,"(%d,%d)",x,y);
		cur_pt = Point(x,y);
		putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255));//只是实时显示鼠标移动的坐标
		imshow("img",tmp);
	}
    else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON) && CV_EVENT_FLAG_CTRLKEY)//按住ctrl健，鼠标左键按下时，鼠标移动，则在图像上划矩形
	{
        source_image.copyTo(img);//将原始图片复制到img中
		img.copyTo(tmp);
		sprintf(temp,"(%d,%d)",x,y);
		cur_pt = Point(x,y);
		putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255));
		rectangle(tmp,pre_pt,cur_pt,Scalar(0,255,0,0),1,8,0);//在临时图像上实时显示鼠标拖动时形成的矩形
		imshow("img",tmp);
	}
	else if (event == CV_EVENT_LBUTTONUP  && CV_EVENT_FLAG_CTRLKEY)//按住ctrl健，鼠标左键松开，将在图像上划矩形
	{
		source_image.copyTo(img);
		sprintf(temp,"(%d,%d)",x,y);
		cur_pt = Point(x,y);
		putText(img,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255));
		circle(img,pre_pt,2,Scalar(255,0,0,0),CV_FILLED,CV_AA,0);
		rectangle(img,pre_pt,cur_pt,Scalar(0,255,0,0),1,8,0);//根据初始点和结束点，将矩形画到img上
		imshow("img",img);
		img.copyTo(tmp);
		//截取矩形包围的图像，并保存到dst中
		int width = abs(pre_pt.x - cur_pt.x);
		int height = abs(pre_pt.y - cur_pt.y);
		if (width == 0 || height == 0)
		{
			printf("width == 0 || height == 0");
			return;
		}
		dst = source_image(Rect(min(cur_pt.x,pre_pt.x),min(cur_pt.y,pre_pt.y),width,height));
		imwrite("../dst_cut.jpg",dst);//保存到本地路径下
	}
}




int main(int argc, char *argv[])
{
    //启动相机
    FrameCapture kinect_cam;
    cv::namedWindow("img", WINDOW_AUTOSIZE);
    int i=1;

    while (1)
    {
        kinect_cam.capture();

        Mat frame_now = kinect_cam.getRgbMat();
        //Mat aligned_depth_image = kinect_cam.getDepthMat();

        //cout<<frame_now<<endl;
        if (frame_now.empty())
        {
            cout << "no image capture!" << endl;
            continue;
        }
        source_image = frame_now.clone();
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

        source_image=imageCopy;
        int num_point;

        cv::setMouseCallback("img", onMouse,0);
        imshow("img", imageCopy);
        
        int key;
        key = waitKey(1000 / 30);
        if (27 == (char)key){
           imwrite("../image/image_"+to_string(i)+".jpg",frame_now);
           i++;
        }

        else if ('c' == (char)key)
        {
            if (ids.size() != 24)
            {
                cout << "points are not enough!"<<endl;
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
        //loading the old matrix
        else if ('l' == (char)key){
            final_transform<< 0.000102009, 0.0008186, 5.42583e-06, -0.287844,
                0.000818614, -0.00010199, -3.21257e-06, 0.177021,
                -2.51704e-06, 5.78141e-06, -0.000824925, 0.168851,
                0.0, 0.0, 0, 1;
                cout<<"loading successfully"<<endl;
        }
    }
    return 1;
    //}
}
