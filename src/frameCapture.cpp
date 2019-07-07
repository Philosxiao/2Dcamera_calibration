#include "frameCapture.h"
#include <string>
#include <vector>
#include "filter.h"

FrameCapture::FrameCapture():sizeColor(1920, 1080), sizeIr(512, 424), depthShift(0), running(false)
{
    kinect_id=0;
    capture_frame_A=0;
    capture_frame_B=0;
    config_path="../config/config.yml";
    calib_path="../config/";
    use_default_params=true;
    start();
}

FrameCapture::~FrameCapture() {
    stop();
}

bool FrameCapture::start()
{
    if(running)
    {
        std::cout<<"kinect2_bridge is already running!"<<std::endl;
        return false;
    }
    if(!initialize())
    {
        std::cerr<<"Initialization failed!"<<std::endl;
        return false;
    }
    running = true;
    return true;
}

void FrameCapture::stop()
{
    if(!running)
    {
        std::cerr<<"kinect2_bridge is not running!"<<std::endl;
        return;
    }
    running = false;
    if(!device->stop())
    {
        std::cerr<<"could not stop device!"<<std::endl;
    }

    if(!device->close())
    {
        std::cerr<<"could not close device!"<<std::endl;
    }
    delete listener;
    delete registration;
    delete packetPipeline;
//    delete device;
    delete depthRegHighRes;
}

bool FrameCapture::initialize()
{
    libfreenect2::setGlobalLogger(NULL);
    FileStorage fs(config_path, FileStorage::READ);
    if(!fs.isOpened()){
        std::string info="can not open config file! Please check the config path: "+config_path;
        std::cerr<<info<<std::endl;
    }
    int32_t  depth_dev=0;
    std::string depth_method = "default";
    double maxDepth, minDepth;
    bool bilateral_filter=true;
    bool edge_aware_filter=true;
    minDepth=0.5;
    maxDepth=4.5;
//    fs["minDepth"] >> minDepth;
//    fs["maxDepth"] >> maxDepth;
    if(kinect_id==0)
        fs["camera_serial_A"] >>sensor;
    else
        fs["camera_serial_B"] >>sensor;
 
    if(calib_path.empty() || calib_path.back() != '/')
    {
        calib_path += '/';
    }

    if(!initPipeline(depth_method, depth_dev))
    {
        return false;
    }

    if(!initDevice(sensor))
    {
        return false;
    }

    initConfig(bilateral_filter, edge_aware_filter, minDepth, maxDepth);

    initCalibration(calib_path, sensor);

    if(!initRegistration(maxDepth))
    {
        if(!device->close())
        {
          std::cerr<<"could not close device!"<<std::endl;
        }
        delete listener;
        return false;
    }
    return true;
}

bool FrameCapture::initRegistration(const double maxDepth){
    depthRegHighRes = new DepthRegistration();

    if(!depthRegHighRes->init(cameraMatrixColor, sizeColor, cameraMatrixDepth, sizeIr, distortionDepth, rotation, translation, 0.5f, maxDepth))
    {
      delete depthRegHighRes;
      return false;
    }

    registration = new libfreenect2::Registration(irParams, colorParams);

    return true;
}

void FrameCapture::initConfig(const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth)
{
  libfreenect2::Freenect2Device::Config config;
  config.EnableBilateralFilter = bilateral_filter;
  config.EnableEdgeAwareFilter = edge_aware_filter;
  config.MinDepth = minDepth;
  config.MaxDepth = maxDepth;
  device->setConfiguration(config);
}

bool FrameCapture::initPipeline(const std::string &method, const int32_t device)
{
    if(method == "default")
    {
        #ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
            packetPipeline = new libfreenect2::CudaPacketPipeline(device);
        #elif defined(LIBFREENECT2_WITH_OPENCL_SUPPORT)
            packetPipeline = new libfreenect2::OpenCLPacketPipeline(device);
        #elif defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
            packetPipeline = new libfreenect2::OpenGLPacketPipeline();
        #else
            packetPipeline = new libfreenect2::CpuPacketPipeline();
        #endif
    }
    else if(method == "cpu")
    {
        packetPipeline = new libfreenect2::CpuPacketPipeline();
    }
    else if(method == "cuda")
    {
        #ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
            packetPipeline = new libfreenect2::CudaPacketPipeline(device);
        #else
            std::cerr<<"Cuda depth processing is not available!"<<std::endl;
            return false;
        #endif
    }
    else if(method == "opencl")
    {
        #ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
            packetPipeline = new libfreenect2::OpenCLPacketPipeline(device);
        #else
            std::cerr<<"OpenCL depth processing is not available!"<<std::endl;
        return false;
    #endif
    }
    else if(method == "opengl")
    {
        #ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
            packetPipeline = new libfreenect2::OpenGLPacketPipeline();
        #else
            std::cerr<<"OpenGL depth processing is not available!"<<std::endl;
            return false;
        #endif
    }
    else if(method == "clkde")
    {
        #ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
            packetPipeline = new libfreenect2::OpenCLKdePacketPipeline(device);
        #else
            std::cerr<<"OpenCL depth processing is not available!"<<std::endl;
            return false;
        #endif
    }
    else if(method == "cudakde")
    {
        #ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
            packetPipeline = new libfreenect2::CudaKdePacketPipeline(device);
        #else
            std::cerr<<"Cuda depth processing is not available!"<<std::endl;
            return false;
        #endif
    }
    else
    {
        std::cerr<<"Unknown depth processing method: " << method<<std::endl;
        return false;
    }

    return true;
}

bool FrameCapture::initDevice(std::string &sensor){
    bool deviceFound = false;
    const int numOfDevs = freenect2.enumerateDevices();

    if(numOfDevs <= 0)
    {
      std::cerr<<"no Kinect2 devices found!"<<std::endl;
      delete packetPipeline;
      return false;
    }

    if(sensor.empty())
    {
      sensor = freenect2.getDefaultDeviceSerialNumber();
    }

    std::cout<<"Kinect2 devices found: "<<std::endl;
    for(int i = 0; i < numOfDevs; ++i)
    {
      const std::string &s = freenect2.getDeviceSerialNumber(i);
      deviceFound = deviceFound || s == sensor;
      std::cout<<"  " << i << ": "<< s << (s == sensor ?" (selected)" : "")<<std::endl;
    }

    if(!deviceFound)
    {
      std::cerr<<"Device with serial '" << sensor << "' not found!"<<std::endl;
      delete packetPipeline;
      return false;
    }

    //device = freenect2.openDevice(sensor);
    device = freenect2.openDevice(sensor, packetPipeline);

    if(device == 0)
    {
      std::cout<<"no device connected or failure opening the default one!"<<std::endl;
      return false;
    }

    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir);

    device->setColorFrameListener(listener);
    device->setIrAndDepthFrameListener(listener);

    std::cout<<"starting kinect2"<<std::endl;
    if(!device->start())
    {
      std::cerr<<"could not start device!"<<std::endl;
      delete listener;
      return false;
    }

    std::cout<<"device serial: "<< sensor<<std::endl;
    std::cout<<"device firmware: "<< device->getFirmwareVersion()<<std::endl;

    colorParams = device->getColorCameraParams();
    irParams = device->getIrCameraParams();

    cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
    distortionColor = cv::Mat::zeros(1, 5, CV_64F);

    cameraMatrixColor.at<double>(0, 0) = colorParams.fx;
    cameraMatrixColor.at<double>(1, 1) = colorParams.fy;
    cameraMatrixColor.at<double>(0, 2) = colorParams.cx;
    cameraMatrixColor.at<double>(1, 2) = colorParams.cy;
    cameraMatrixColor.at<double>(2, 2) = 1;

    cameraMatrixIr = cv::Mat::eye(3, 3, CV_64F);
    distortionIr = cv::Mat::zeros(1, 5, CV_64F);

    cameraMatrixIr.at<double>(0, 0) = irParams.fx;
    cameraMatrixIr.at<double>(1, 1) = irParams.fy;
    cameraMatrixIr.at<double>(0, 2) = irParams.cx;
    cameraMatrixIr.at<double>(1, 2) = irParams.cy;
    cameraMatrixIr.at<double>(2, 2) = 1;

    distortionIr.at<double>(0, 0) = irParams.k1;
    distortionIr.at<double>(0, 1) = irParams.k2;
    distortionIr.at<double>(0, 2) = irParams.p1;
    distortionIr.at<double>(0, 3) = irParams.p2;
    distortionIr.at<double>(0, 4) = irParams.k3;

    cameraMatrixDepth = cameraMatrixIr.clone();
    distortionDepth = distortionIr.clone();

    rotation = cv::Mat::eye(3, 3, CV_64F);
    translation = cv::Mat::zeros(3, 1, CV_64F);
    return true;
}

void FrameCapture::initCalibration(const std::string &calib_path, const std::string &sensor)
{
  std::string calibPath = calib_path + sensor + '/';

  struct stat fileStat;
  bool calibDirNotFound = stat(calibPath.c_str(), &fileStat) != 0 || !S_ISDIR(fileStat.st_mode);
  if(!calibDirNotFound)
      use_default_params=false;
  if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor))
  {
    std::cout<<"using sensor defaults for color intrinsic parameters."<<std::endl;
  }

  if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_IR, cameraMatrixDepth, distortionDepth))
  {
    std::cout<<"using sensor defaults for ir intrinsic parameters."<<std::endl;
  }

  if(calibDirNotFound || !loadCalibrationPoseFile(calibPath + K2_CALIB_POSE, rotation, translation))
  {
    std::cout<<"using defaults for rotation and translation."<<std::endl;
  }

  if(calibDirNotFound || !loadCalibrationDepthFile(calibPath + K2_CALIB_DEPTH, depthShift))
  {
    std::cout<<"using defaults for depth shift."<<std::endl;
    depthShift = 0.0;
  }

  const int mapType = CV_16SC2;
  cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, sizeColor, mapType, map1Color, map2Color);
  cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);

  std::cout<<"camera parameters used:"<<std::endl;
  std::cout<<"camera matrix color:"<< std::endl << cameraMatrixColor << std::endl;
  std::cout<<"distortion coefficients color:" << std::endl << distortionColor << std::endl;
  std::cout<<"camera matrix ir:"<< std::endl << cameraMatrixIr << std::endl;
  std::cout<<"distortion coefficients ir:"<< std::endl << distortionIr << std::endl;
  std::cout<<"camera matrix depth:"  << std::endl << cameraMatrixDepth << std::endl;
  std::cout<<"distortion coefficients depth:"  << std::endl << distortionDepth << std::endl;
  std::cout<<"rotation:"<< std::endl << rotation << std::endl;
  std::cout<<"translation:"<< std::endl << translation << std::endl;
  std::cout<<"depth shift:"<< std::endl << depthShift << std::endl;
}

bool FrameCapture::loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const
{
  cv::FileStorage fs;
  if(fs.open(filename, cv::FileStorage::READ))
  {
    fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
    fs[K2_CALIB_DISTORTION] >> distortion;
    fs.release();
  }
  else
  {
    std::cerr<<"can't open calibration file: " << filename<<std::endl;
    return false;
  }
  return true;
}

bool FrameCapture::loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const
{
  cv::FileStorage fs;
  if(fs.open(filename, cv::FileStorage::READ))
  {
    fs[K2_CALIB_ROTATION] >> rotation;
    fs[K2_CALIB_TRANSLATION] >> translation;
    fs.release();
  }
  else
  {
    std::cerr<<"can't open calibration pose file: " << filename<<std::endl;
    return false;
  }
  return true;
}

bool FrameCapture::loadCalibrationDepthFile(const std::string &filename, double &depthShift) const
{
  cv::FileStorage fs;
  if(fs.open(filename, cv::FileStorage::READ))
  {
    fs[K2_CALIB_DEPTH_SHIFT] >> depthShift;
    fs.release();
  }
  else
  {
    std::cerr<<"can't open calibration depth file: " << filename<<std::endl;
    return false;
  }
  return true;
}

bool FrameCapture::receiveFrames(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::FrameMap &frames)
{
  bool newFrames = false;
  for(; !newFrames;)
  {
    newFrames = true;
    listener->waitForNewFrame(frames);
    if(!running)
    {
      if(newFrames)
      {
        listener->release(frames);
      }
      return false;
    }
  }
  return newFrames;
}

void FrameCapture::receiveRGBD(){
    // if(!device->start())
    // {
    //   std::cerr<<"could not start device!"<<std::endl;
    //   return;
    // }
    std::vector<cv::Mat> depths;
    cv::Mat depth_median,color;
    int sample_num=5;
    //for(int i=0;i<sample_num;i++){
        libfreenect2::FrameMap frames;
        if(!receiveFrames(listener, frames))
        {
          return;
        }
        libfreenect2::Frame *depthFrame = frames[libfreenect2::Frame::Depth];
        if(depthFrame->status != 0)
        {
          listener->release(frames);
          running = false;
          std::cerr<<"failure in depth packet processor from libfreenect2"<<std::endl;
          return;
        }
        if(depthFrame->format != libfreenect2::Frame::Float)
        {
          listener->release(frames);
          running = false;
          std::cerr<<"received invalid frame format"<<std::endl;
          return;
        }
        cv::Mat depth;
        cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data).copyTo(depth);
        //depths.push_back(depth);
        //if(i==sample_num-1){
            libfreenect2::Frame *colorFrame = frames[libfreenect2::Frame::Color];
            if(colorFrame->status != 0)
            {
                listener->release(frames);
                running = false;
                std::cerr<<"failure in rgb packet processor from libfreenect2"<<std::endl;
                return;
            }
            if(colorFrame->format != libfreenect2::Frame::BGRX && colorFrame->format != libfreenect2::Frame::RGBX)
            {
                listener->release(frames);
                running = false;
                std::cerr<<"received invalid frame format"<<std::endl;
                return;
            }
            cv::Mat(colorFrame->height, colorFrame->width, CV_8UC4, colorFrame->data).copyTo(color);
        //}
        listener->release(frames);
   // }
//    depth_median.create( depths[0].size(), depths[0].type());
//    depth_median=Scalar::all(0);
//    #pragma omp parallel for
//    for(int i=0;i<depths[0].size().height;i++){
//        for(int j=0;j<depths[0].size().width;j++)
//        {
//            std::vector<float> depth_vec;
//            for(int k=0;k<sample_num;k++) depth_vec.push_back(depths[k].at<float>(i,j));
//            std::nth_element(depth_vec.begin(),depth_vec.begin()+(sample_num-1)/2,depth_vec.end());
//            depth_median.at<float>(i,j)=depth_vec[(sample_num-1)/2];
//        }
//    }

    //processIrDepth(depth_median);
    processIrDepth(depth);
    processColor(color);
    // if(!device->stop())
    // {
    //   std::cout<<"could not stop device!"<<std::endl;
    //   return;
    // }
}

void FrameCapture::processIrDepth(const cv::Mat &depth){
    // DEPTH
    cv::Mat depthShifted;
    depth.convertTo(depthShifted, CV_16U, 1, depthShift);
    cv::flip(depthShifted, depthShifted, 1);
    depthRegHighRes->registerDepth(depthShifted, depth_raw);
    depth_raw.convertTo(depth_raw,CV_16UC1);
}

void FrameCapture::processColor(const cv::Mat &color)
{
    cv::Mat tmp,rgb_img;
    cv::flip(color, tmp, 1);
    cv::cvtColor(tmp, rgb_img, CV_BGRA2BGR);
  // COLOR
    cv::remap(rgb_img, rgbmat, map1Color, map2Color, cv::INTER_AREA);
    rgbmat.convertTo(rgbmat,CV_8UC3);
}

void FrameCapture::setKinectId(int idx){
    kinect_id=idx;
}

void FrameCapture::transToIdKinect(int idx){
    setKinectId(idx);
    stop();
    sleep(3);
    start();
}

cv::Mat FrameCapture::depthRefinement(cv::Mat &depth, cv::Mat &rgb)
{

    cv::Mat srcImageGray;
    cvtColor(rgb, srcImageGray, CV_RGB2GRAY);
    cv::Mat srcImagef;
    srcImageGray.convertTo(srcImagef, CV_32F);
    int r = 3;
    int sigs = 30;
    int sigc = 50;
    int sigc2 = 50;
    int pr = 2;
    double ss = sigs / 10.0;
    double sc = sigc / 10.0;
    double sc2 = sigc2;
    int d = 2 * r + 1;

    depth.setTo(0,depth>4500);
    cv::Mat filledDepth = depth.clone();
    fillOcclusionDepth(filledDepth, 0);
    cv::Mat tp;
    transpose(filledDepth, tp);
    fillOcclusionDepth(tp, 0);
    transpose(tp, filledDepth);
    cv::Mat filledDepthf;
    filledDepth.convertTo(filledDepthf, CV_32F);

    double minv, maxv;
    minMaxLoc(filledDepth, &minv, &maxv);

    cv::Mat depthout;
    cv::Mat weight;
    cv::Mat filteredDepthf = cv::Mat::ones(depth.size(), CV_32F);
    trilateralWeightMap(srcImagef, filledDepthf, weight, Size(d, d), sc, sc2, ss);
    weightedJointBilateralFilter(filledDepthf, weight, srcImagef, filteredDepthf, Size(d, d), sc, ss, 0);
    filteredDepthf.convertTo(depthout, CV_16U);
    jointNearestFilter(depthout, filledDepth, Size(2 * pr + 1, 2 * pr + 1), depthout);
    return depthout;
}

void FrameCapture::default_receiveRGBD(){
    libfreenect2::FrameMap frames;
    libfreenect2::Frame* undistorted = new libfreenect2::Frame(512,424,4);
    libfreenect2::Frame* registered = new libfreenect2::Frame(512,424,4);
    libfreenect2::Frame* depth2rgb = new libfreenect2::Frame(1920,1080 + 2,4);
    int sample_num=3;
    for(int i=0;i<sample_num;i++){
        listener->waitForNewFrame(frames);
        if(i==sample_num-1)
            break;
        listener->release(frames);
    }
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];


    cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);

    cv::flip(rgbmat, rgbmat, 1);

    registration->apply(rgb, depth, undistorted, registered, true, depth2rgb);

    cv::Mat bigdepth;
    cv::Mat(depth2rgb->height, depth2rgb->width, CV_32FC1, depth2rgb->data).copyTo(bigdepth);

    cv::flip(bigdepth, bigdepth, 1);

    cv::cvtColor(rgbmat,rgbmat,cv::COLOR_BGRA2BGR);

    bigdepth.convertTo(bigdepth, CV_16UC1);
    bigdepth.setTo(0, bigdepth > 4500.0f);
    bigdepth(cv::Rect(0, 1, 1920, 1080)).copyTo(depth_raw);
    listener->release(frames);
    delete undistorted;
    delete registered;
    delete depth2rgb;
}

void FrameCapture::capture() {
    if(use_default_params)
        default_receiveRGBD();
    else
        receiveRGBD();
    depthmat=depthRefinement(depth_raw,rgbmat);
}



