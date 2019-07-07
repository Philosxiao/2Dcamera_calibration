#pragma once

#ifndef DETECT_CAL_H
#define DETECT_CAL_H
// c++
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netdb.h> 


// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <detect_cal.h>
//#include <TCPClient.h>

// Darknet.
#ifdef GPU
#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"
#endif

//Realsense
// Include OpenCV API
#include <opencv2/opencv.hpp> // Include OpenCV API
                              // Include RealSense Cross Platform API


extern "C"
{
#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include <sys/time.h>
#include <image.h>
#include <darknet.h>
}




using namespace std;

class TCPClient
{
  private:
    int sock;
    std::string address;
    int port;
    struct sockaddr_in server;
    struct timeval timeout = {7, 0};

  public:
    TCPClient();
    bool setup(string address, int port);
    bool Send(string data);
    string receive(int size = 4096);
    string receive_pic(int size);
    string read();
    void exit();
};



extern "C" void ipl_into_image(IplImage *src, image im);
extern "C" image ipl_to_image(IplImage *src);
extern "C" void show_image_cv(image p, const char *name, IplImage *disp);


using namespace cv;
using namespace std;






struct DetectedObject
{
    int object_class;
    float prob;
    box bounding_box;        //比例
    cv::Rect bounding_point; //在图像上的实际像素位置
    string num_code;

    DetectedObject(int object_class, float prob, box bb, cv::Rect bounding_point) : object_class(object_class), prob(prob), bounding_box(bb), bounding_point(bounding_point) {}
};


struct num_exist
{
    //code的寄存器
    string num;
    int count_times;
    num_exist(string code_num, int count_times = 4) : num(code_num), count_times(count_times) {}
    //检查,发送出去
    bool check_if_exist()
    {
        if (count_times > 0)
        {
            return true;
        }
        else
            return false;
    }
};


struct BP
{
    //有位置,判断如果在中心的话
    string name;
    Point center_position;
    vector<int> bounding;
    Mat arean;
    int link_code = -1;

    BP(string name, Point center_position, Mat arean, vector<int> bounding) : name(name), center_position(center_position), arean(arean), bounding(bounding) {}
};

struct code
{
    //有效存在的code
    Point center_position;
    Mat arean;
    bool num_exist; //是否在其中存在数字
    string num_type;
    string num_code;
    int link_packegebox_num = -1;

    bool is_in_BP(vector<int> BPbounding)
    {
        if (center_position.x > BPbounding[0] && center_position.x < BPbounding[1] && center_position.y > BPbounding[2] && center_position.y < BPbounding[3])
            return true;
        else
            return false;
    }

    code(Point center,Mat image, string num_type, string num_code) : center_position(center), num_exist(true),arean(image), num_type(num_type), num_code(num_code) {}
    code(Point center,Mat image) : center_position(center), num_exist(false),arean(image){}
};
// #ifdef GPU

// extern "C"  image get_image_from_stream(CvCapture *cap);
// extern "C"  std::vector<DetectedObject> detect_bounding_box(network *net, rs2::frame color, float thresh, Mat &video_img);
// extern "C"  void show_result(Mat &video_img, std::vector<DetectedObject> detect_result, char **classes_name);
// extern "C"  std::vector<Mat> detect_forground(frameset data, std::vector<DetectedObject> detect_result, int fro_need_show);

// #else
string Int_to_String(int n);
bool is_num_in_exist(vector<num_exist> v, num_exist element);
image get_image_from_stream(CvCapture *cap);
std::vector<DetectedObject> detect_bounding_box(network *net, cv::Mat video_img, float thresh);
void show_result(Mat &video_img, std::vector<DetectedObject> detect_result, char **classes_name,vector<num_exist>& codenum_all);
cv::Mat find_fit_tar(Mat image,int num);
std::vector<Mat> get_likly_arean(std::vector<Mat> object_front, network *net, char **names, cv::Mat color_mat);
cv::Mat get_image_from_mask(std::vector<Mat> fro_mask,cv::Mat color_mat);


cv::Mat matRotateClockWise180(cv::Mat src);//返回顺时针180旋转后的矩阵
std::vector<std::string>  get_code(cv::Mat resource_image);
vector<string>  get_unroate_code(cv::Mat resource_image);

cv::Mat send_return(Mat rgb_img);

//#endif


#endif