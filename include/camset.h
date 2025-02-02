/************************************************************************/
/* 没有附加修改相机IP的代码，在运行之前，
请使用相机客户端修改相机IP地址的网段与主机的网段一致                 */
/************************************************************************/

#ifndef __CAMSET_H__
#define __CAMSET_H__
#ifndef __unix__
#include <Winsock2.h>
#else
#include <arpa/inet.h>
#endif
#include "GenICam/System.h"
#include "GenICam/Camera.h"
#include "GenICam/StreamSource.h"
#include "GenICam/GigE/GigECamera.h"
#include "GenICam/GigE/GigEInterface.h"
#include "Infra/PrintLog.h"
#include "StreamRetrieve.h"
#include "Memory/SharedPtr.h"



// using namespace Dahua::GenICam;
// using namespace Dahua::Infra;


/* 4、设置相机采图模式（连续采图、触发采图） */
static int32_t setGrabMode(ICameraPtr& cameraSptr, bool bContious)
{
    int32_t bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptrAcquisitionControl->triggerSelector();
    bRet = enumNode.setValueBySymbol("FrameStart");
    if (false == bRet)
    {
        printf("set TriggerSelector fail.\n");
        return -1;
    }

    if (true == bContious)
    {
        enumNode = sptrAcquisitionControl->triggerMode();
        bRet = enumNode.setValueBySymbol("Off");
        if (false == bRet)
        {
            printf("set triggerMode fail.\n");
            return -1;
        }
    }
    else
    {
        enumNode = sptrAcquisitionControl->triggerMode();
        bRet = enumNode.setValueBySymbol("On");
        if (false == bRet)
        {
            printf("set triggerMode fail.\n");
            return -1;
        }

        /* 设置触发源为软触发（硬触发为Line1） */
        enumNode = sptrAcquisitionControl->triggerSource();
        bRet = enumNode.setValueBySymbol("Software");
        if (false == bRet)
        {
            printf("set triggerSource fail.\n");
            return -1;
        }
    }
    return 0;
}

/* 5、获取相机采图模式 */
static int32_t getGrabMode(ICameraPtr& cameraSptr, bool &bContious)
{
    int32_t bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptrAcquisitionControl->triggerSelector();
    bRet = enumNode.setValueBySymbol("FrameStart");
    if (false == bRet)
    {
        printf("set TriggerSelector fail.\n");
        return -1;
    }

    CString strValue;
    enumNode = sptrAcquisitionControl->triggerMode();
    bRet = enumNode.getValueSymbol(strValue);
    if (false == bRet)
    {
        printf("get triggerMode fail.\n");
        return -1;
    }

    if (strValue == "Off")
    {
        bContious = true;
    }
    else if (strValue == "On")
    {
        bContious = false;
    }
    else
    {
        printf("get triggerMode fail.\n");
        return -1;
    }
    return 0;
}

/* 6、软件触发 */
static int32_t triggerSoftware(ICameraPtr& cameraSptr)
{
    int32_t bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    CCmdNode cmdNode = sptrAcquisitionControl->triggerSoftware();
    bRet = cmdNode.execute();
    if (false == bRet)
    {
        printf("triggerSoftware execute fail.\n");
        return -1;
    }
    return 0;
}

/* 9、设置传感器采样率（采集分辨率） */
static int32_t setResolution(ICameraPtr& cameraSptr, int nWidth, int nHeight)
{
    int32_t bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.setValue(nWidth);
    if (false == bRet)
    {
        printf("set width fail.\n");
        return -1;
    }

    intNode = sptrImageFormatControl->height();
    bRet = intNode.setValue(nHeight);
    if (false == bRet)
    {
        printf("set height fail.\n");
        return -1;
    }
    return 0;
}

/* 10、获取传感器采样率 */
static int32_t getResolution(ICameraPtr& cameraSptr, int64_t &nWidth, int64_t &nHeight)
{
    int32_t bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.getValue(nWidth);
    if (false == bRet)
    {
        printf("get width fail.\n");
        return -1;
    }

    intNode = sptrImageFormatControl->height();
    bRet = intNode.getValue(nHeight);
    if (false == bRet)
    {
        printf("get height fail.\n");
        return -1;
    }
    return 0;
}

/* 设置binning (Off X Y XY) */
static int32_t setBinning(ICameraPtr& cameraSptr)
{
    CEnumNodePtr ptrParam(new CEnumNode(cameraSptr, "Binning"));
    if (ptrParam)
    {
        if (false == ptrParam->isReadable())
        {
            printf("binning not support.\n");
            return -1;
        }

        if (false == ptrParam->setValueBySymbol("XY"))
        {
            printf("set Binning XY fail.\n");
            return -1;
        }

        if (false == ptrParam->setValueBySymbol("Off"))
        {
            printf("set Binning Off fail.\n");
            return -1;
        }
    }
    return 0;
}

/* 11、获取传感器最大分辩率 */
static int32_t getMaxResolution(ICameraPtr& cameraSptr, int64_t &nWidthMax, int64_t &nHeightMax)
{
    /* width */
    {
        CIntNodePtr ptrParam(new CIntNode(cameraSptr, "SensorWidth"));
        if (ptrParam)
        {
            if (false == ptrParam->getValue(nWidthMax))
            {
                printf("get WidthMax fail.\n");
                return -1;
            }
        }
    }

    /* height */
    {
        CIntNodePtr ptrParam(new CIntNode(cameraSptr, "SensorHeight"));
        if (ptrParam)
        {
            if (false == ptrParam->getValue(nWidthMax))
            {
                printf("get WidthMax fail.\n");
                return -1;
            }
        }
    }
    return 0;
}

/* 12、设置图像ROI */
static int32_t setROI(ICameraPtr& cameraSptr, int64_t nX, int64_t nY, int64_t nWidth, int64_t nHeight)
{
    bool bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    /* width */
    CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.setValue(nWidth);
    if (!bRet)
    {
        printf("set width fail.\n");
	return -1;
    }

    /* height */
    intNode = sptrImageFormatControl->height();
    bRet = intNode.setValue(nHeight);
    if (!bRet)
    {
        printf("set height fail.\n");
	return -1;
    }

    /* OffsetX */
    intNode = sptrImageFormatControl->offsetX();
    bRet = intNode.setValue(nX);
    if (!bRet)
    {
        printf("set offsetX fail.\n");
	return -1;
    }

    /* OffsetY */
    intNode = sptrImageFormatControl->offsetY();
    bRet = intNode.setValue(nY);
    if (!bRet)
    {
        printf("set offsetY fail.\n");
	return -1;
    }

    return 0;
}

/* 13、获取图像ROI */
static int32_t getROI(ICameraPtr& cameraSptr, int64_t &nX, int64_t &nY, int64_t &nWidth, int64_t &nHeight)
{
    bool bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    /* width */
    CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.getValue(nWidth);
    if (!bRet)
    {
        printf("get width fail.\n");
    }

    /* height */
    intNode = sptrImageFormatControl->height();
    bRet = intNode.getValue(nHeight);
    if (!bRet)
    {
        printf("get height fail.\n");
    }

    /* OffsetX */
    intNode = sptrImageFormatControl->offsetX();
    bRet = intNode.getValue(nX);
    if (!bRet)
    {
        printf("get offsetX fail.\n");
    }

    /* OffsetY */
    intNode = sptrImageFormatControl->offsetY();
    bRet = intNode.getValue(nY);
    if (!bRet)
    {
        printf("get offsetY fail.\n");
    }
    return 0;
}

/* 14、获取采图图像宽度 */
static int32_t getWidth(ICameraPtr& cameraSptr, int64_t &nWidth)
{
    bool bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.getValue(nWidth);
    if (!bRet)
    {
        printf("get width fail.\n");
    }
    return 0;
}

/* 15、获取采图图像高度 */
static int32_t getHeight(ICameraPtr& cameraSptr, int64_t &nHeight)
{
    bool bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    CIntNode intNode = sptrImageFormatControl->height();
    bRet = intNode.getValue(nHeight);
    if (!bRet)
    {
        printf("get height fail.\n");
        return -1;
    }
    return 0;
}

/* 17、设置曝光值(曝光、自动曝光/手动曝光) */
static int32_t setExposureTime(ICameraPtr& cameraSptr, double dExposureTime, bool bAutoExposure = false)
{
    bool bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    if (bAutoExposure)
    {
        CEnumNode enumNode = sptrAcquisitionControl->exposureAuto();
        bRet = enumNode.setValueBySymbol("Continuous");
        if (false == bRet)
        {
            printf("set exposureAuto fail.\n");
            return -1;
        }
    }
    else
    {
        CEnumNode enumNode = sptrAcquisitionControl->exposureAuto();
        bRet = enumNode.setValueBySymbol("Off");
        if (false == bRet)
        {
            printf("set exposureAuto fail.\n");
            return -1;
        }

        CDoubleNode doubleNode = sptrAcquisitionControl->exposureTime();
        bRet = doubleNode.setValue(dExposureTime);
        if (false == bRet)
        {
            printf("set exposureTime fail.\n");
            return -1;
        }
    }
    return 0;
}

/* 18、获取曝光时间 */
static int32_t getExposureTime(ICameraPtr& cameraSptr, double &dExposureTime)
{
    bool bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAcquisitionControl->exposureTime();
    bRet = doubleNode.getValue(dExposureTime);
    if (false == bRet)
    {
        printf("get exposureTime fail.\n");
        return -1;
    }
    return 0;
}

/* 19、获取曝光范围 */
static int32_t getExposureTimeMinMaxValue(ICameraPtr& cameraSptr, double &dMinValue, double &dMaxValue)
{
    bool bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAcquisitionControl->exposureTime();
    bRet = doubleNode.getMinVal(dMinValue);
    if (false == bRet)
    {
        printf("get exposureTime minValue fail.\n");
        return -1;
    }

    bRet = doubleNode.getMaxVal(dMaxValue);
    if (false == bRet)
    {
        printf("get exposureTime maxValue fail.\n");
        return -1;
    }
    return 0;
}

/* 20、设置增益值 */
static int32_t setGainRaw(ICameraPtr& cameraSptr, double dGainRaw)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
    bRet = doubleNode.setValue(dGainRaw);
    if (false == bRet)
    {
        printf("set gainRaw fail.\n");
        return -1;
    }
    return 0;
}

/* 21、获取增益值 */
static int32_t getGainRaw(ICameraPtr& cameraSptr, double &dGainRaw)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
    bRet = doubleNode.getValue(dGainRaw);
    if (false == bRet)
    {
        printf("get gainRaw fail.\n");
        return -1;
    }
    return 0;
}

/* 22、获取增益值范围 */
static int32_t getGainRawMinMaxValue(ICameraPtr& cameraSptr, double &dMinValue, double &dMaxValue)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
    bRet = doubleNode.getMinVal(dMinValue);
    if (false == bRet)
    {
        printf("get gainRaw minValue fail.\n");
        return -1;
    }

    bRet = doubleNode.getMaxVal(dMaxValue);
    if (false == bRet)
    {
        printf("get gainRaw maxValue fail.\n");
        return -1;
    }
    return 0;
}

/* 23、设置伽马值 */
static int32_t setGamma(ICameraPtr& cameraSptr, double dGamma)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gamma();
    bRet = doubleNode.setValue(dGamma);
    if (false == bRet)
    {
        printf("set gamma fail.\n");
        return -1;
    }
    return 0;
}

/* 24、获取伽马值 */
static int32_t getGamma(ICameraPtr& cameraSptr, double &dGamma)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gamma();
    bRet = doubleNode.getValue(dGamma);
    if (false == bRet)
    {
        printf("get gamma fail.\n");
        return -1;
    }
    return 0;
}

/* 25、获取伽马值范围 */
static int32_t getGammaMinMaxValue(ICameraPtr& cameraSptr, double &dMinValue, double &dMaxValue)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gamma();
    bRet = doubleNode.getMinVal(dMinValue);
    if (false == bRet)
    {
        printf("get gamma minValue fail.\n");
        return -1;
    }

    bRet = doubleNode.getMaxVal(dMaxValue);
    if (false == bRet)
    {
        printf("get gamma maxValue fail.\n");
        return -1;
    }
    return 0;
}

/* 26、设置白平衡值（有三个白平衡值） */
static int32_t setBalanceRatio(ICameraPtr& cameraSptr, double dRedBalanceRatio, double dGreenBalanceRatio, double dBlueBalanceRatio)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    /* 关闭自动白平衡 */
    CEnumNode enumNode = sptrAnalogControl->balanceWhiteAuto();
    if (false == enumNode.isReadable())
    {
        printf("balanceRatio not support.\n");
        return -1;
    }

    bRet = enumNode.setValueBySymbol("Off");
    if (false == bRet)
    {
        printf("set balanceWhiteAuto Off fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Red");
    if (false == bRet)
    {
        printf("set red balanceRatioSelector fail.\n");
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.setValue(dRedBalanceRatio);
    if (false == bRet)
    {
        printf("set red balanceRatio fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Green");
    if (false == bRet)
    {
        printf("set green balanceRatioSelector fail.\n");
        return -1;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.setValue(dGreenBalanceRatio);
    if (false == bRet)
    {
        printf("set green balanceRatio fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Blue");
    if (false == bRet)
    {
        printf("set blue balanceRatioSelector fail.\n");
        return -1;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.setValue(dBlueBalanceRatio);
    if (false == bRet)
    {
        printf("set blue balanceRatio fail.\n");
        return -1;
    }
    return 0;
}

/* 27、获取白平衡值（有三个白平衡值） */
static int32_t getBalanceRatio(ICameraPtr& cameraSptr, double &dRedBalanceRatio, double &dGreenBalanceRatio, double &dBlueBalanceRatio)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptrAnalogControl->balanceRatioSelector();
    if (false == enumNode.isReadable())
    {
        printf("balanceRatio not support.\n");
        return -1;
    }

    bRet = enumNode.setValueBySymbol("Red");
    if (false == bRet)
    {
        printf("set red balanceRatioSelector fail.\n");
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.getValue(dRedBalanceRatio);
    if (false == bRet)
    {
        printf("get red balanceRatio fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Green");
    if (false == bRet)
    {
        printf("set green balanceRatioSelector fail.\n");
        return -1;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.getValue(dGreenBalanceRatio);
    if (false == bRet)
    {
        printf("get green balanceRatio fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Blue");
    if (false == bRet)
    {
        printf("set blue balanceRatioSelector fail.\n");
        return -1;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.getValue(dBlueBalanceRatio);
    if (false == bRet)
    {
        printf("get blue balanceRatio fail.\n");
        return -1;
    }
    return 0;
}

/* 28、获取白平衡值范围 */
static int32_t getBalanceRatioMinMaxValue(ICameraPtr& cameraSptr, double &dMinValue, double &dMaxValue)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
    if (false == doubleNode.isReadable())
    {
        printf("balanceRatio not support.\n");
        return -1;
    }

    bRet = doubleNode.getMinVal(dMinValue);
    if (false == bRet)
    {
        printf("get balanceRatio min value fail.\n");
        return -1;
    }

    bRet = doubleNode.getMaxVal(dMaxValue);
    if (false == bRet)
    {
        printf("get balanceRatio max value fail.\n");
        return -1;
    }

    return 0;
}

/* 29、设置采图速度（秒帧数） */
static int32_t setAcquisitionFrameRate(ICameraPtr& cameraSptr, double dFrameRate)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CBoolNode booleanNode = sptAcquisitionControl->acquisitionFrameRateEnable();
    bRet = booleanNode.setValue(true);
    if (false == bRet)
    {
        printf("set acquisitionFrameRateEnable fail.\n");
        return -1;
    }

    CDoubleNode doubleNode = sptAcquisitionControl->acquisitionFrameRate();
    bRet = doubleNode.setValue(dFrameRate);
    if (false == bRet)
    {
        printf("set acquisitionFrameRate fail.\n");
        return -1;
    }
    return 0;
}

/* 30、获取采图速度（秒帧数） */
static int32_t getAcquisitionFrameRate(ICameraPtr& cameraSptr, double &dFrameRate)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptAcquisitionControl->acquisitionFrameRate();
    bRet = doubleNode.getValue(dFrameRate);
    if (false == bRet)
    {
        printf("get acquisitionFrameRate fail.\n");
        return -1;
    }
    return 0;
}

/* 31、保存参数 */
static int32_t userSetSave(ICameraPtr& cameraSptr)
{
    bool bRet;
    IUserSetControlPtr sptUserSetControl = CSystem::getInstance().createUserSetControl(cameraSptr);
    if (NULL == sptUserSetControl)
    {
        return -1;
    }

    bRet = sptUserSetControl->saveUserSet(IUserSetControl::userSet1);
    if (false == bRet)
    {
        printf("saveUserSet fail.\n");
        return -1;
    }
    return 0;
}

/* 32、加载参数 */
static int32_t loadUserSet(ICameraPtr& cameraSptr)
{
    bool bRet;
    IUserSetControlPtr sptUserSetControl = CSystem::getInstance().createUserSetControl(cameraSptr);
    if (NULL == sptUserSetControl)
    {
        return -1;
    }

    bRet = sptUserSetControl->setCurrentUserSet(IUserSetControl::userSet1);
    if (false == bRet)
    {
        printf("saveUserSet fail.\n");
        return -1;
    }
    return 0;
}

/* 33、设置外触发延时时间 */
static int32_t setTriggerDelay(ICameraPtr& cameraSptr, double dDelayTime)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptAcquisitionControl->triggerDelay();
    bRet = doubleNode.setValue(dDelayTime);
    if (false == bRet)
    {
        printf("set triggerDelay fail.\n");
        return -1;
    }
    return 0;
}

/* 34、获取外触发延时时间 */
static int32_t getTriggerDelay(ICameraPtr& cameraSptr, double &dDelayTime)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptAcquisitionControl->triggerDelay();
    bRet = doubleNode.getValue(dDelayTime);
    if (false == bRet)
    {
        printf("set triggerDelay fail.\n");
        return -1;
    }
    return 0;
}

/* 35、设置外触发模式（上升沿触发、下降沿触发） */
static int32_t setLineTriggerMode(ICameraPtr& cameraSptr, bool bRisingEdge)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptAcquisitionControl->triggerSelector();
    if (false == enumNode.setValueBySymbol("FrameStart"))
    {
        printf("set triggerSelector fail.\n");
        return -1;
    }

    enumNode = sptAcquisitionControl->triggerMode();
    if (false == enumNode.setValueBySymbol("On"))
    {
        printf("set triggerMode fail.\n");
        return -1;
    }

    enumNode = sptAcquisitionControl->triggerSource();
    if (false == enumNode.setValueBySymbol("Line1"))
    {
        printf("set triggerSource fail.\n");
        return -1;
    }

    enumNode = sptAcquisitionControl->triggerActivation();
    if (true == bRisingEdge)
    {
        bRet = enumNode.setValueBySymbol("RisingEdge");
    }
    else
    {
        bRet = enumNode.setValueBySymbol("FallingEdge");
    }
    return 0;
}

/* 36、获取外触发模式（上升沿触发、下降沿触发） */
static int32_t getLineTriggerMode(ICameraPtr& cameraSptr, bool &bRisingEdge)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptAcquisitionControl->triggerSelector();
    if (false == enumNode.setValueBySymbol("FrameStart"))
    {
        printf("set triggerSelector fail.\n");
        return -1;
    }

    CString strValue;
    enumNode = sptAcquisitionControl->triggerActivation();
    if (true == bRisingEdge)
    {
        bRet = enumNode.getValueSymbol(strValue);
    }
    else
    {
        bRet = enumNode.getValueSymbol(strValue);
    }

    if (false == bRet)
    {
        printf("get triggerActivation fail.\n");
        return -1;
    }

    if (strValue == "RisingEdge")
    {
        bRisingEdge = true;
    }
    else if (strValue == "FallingEdge")
    {
        bRisingEdge = false;
    }
    else
    {
        printf("get triggerActivation fail.\n");
        return -1;
    }
    return 0;
}

/* 37、设置外触发信号滤波时间 */
static int32_t setLineDebouncerTimeAbs(ICameraPtr& cameraSptr, double dLineDebouncerTimeAbs)
{
    IDigitalIOControlPtr sptDigitalIOControl = CSystem::getInstance().createDigitalIOControl(cameraSptr);
    if (NULL == sptDigitalIOControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptDigitalIOControl->lineSelector();
    if (false == enumNode.setValueBySymbol("Line1"))
    {
        printf("set lineSelector fail.\n");
        return -1;
    }

    CDoubleNode doubleNode = sptDigitalIOControl->lineDebouncerTimeAbs();
    if (false == doubleNode.setValue(dLineDebouncerTimeAbs))
    {
        printf("set lineDebouncerTimeAbs fail.\n");
        return -1;
    }
    return 0;
}

/* 38、获取外触发信号滤波时间 */
static int32_t getLineDebouncerTimeAbs(ICameraPtr& cameraSptr, double &dLineDebouncerTimeAbs)
{
    IDigitalIOControlPtr sptDigitalIOControl = CSystem::getInstance().createDigitalIOControl(cameraSptr);
    if (NULL == sptDigitalIOControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptDigitalIOControl->lineSelector();
    if (false == enumNode.setValueBySymbol("Line1"))
    {
        printf("set lineSelector fail.\n");
        return -1;
    }

    CDoubleNode doubleNode = sptDigitalIOControl->lineDebouncerTimeAbs();
    if (false == doubleNode.getValue(dLineDebouncerTimeAbs))
    {
        printf("get lineDebouncerTimeAbs fail.\n");
        return -1;
    }
    return 0;
}

/* 39、设置外触发脉冲宽度（不支持） */
/* 40、获取外触发脉冲宽度（不支持） */
/* 41、设置输出信号线（控制光源用）（面阵相机是Line0） */
/* 42、获取输出信号线（面阵相机是Line0） */

/* 43、设置外部光源曝光时间（设置输出值为TRUE的时间） */
static int32_t setOutputTime(ICameraPtr& cameraSptr, int nTimeMS)
{
    IDigitalIOControlPtr sptDigitalIOControl = CSystem::getInstance().createDigitalIOControl(cameraSptr);
    if (NULL == sptDigitalIOControl)
    {
        return -1;
    }

    CEnumNode paramLineSource(cameraSptr, "LineSource");
    if (false == paramLineSource.setValueBySymbol("UserOutput1"))
    {
        printf("set LineSource fail.");
        return -1;
    }

    /* 将输出信号拉高然后拉低 */
    CBoolNode booleanNode = sptDigitalIOControl->userOutputValue();
    if (false == booleanNode.setValue(true))
    {
        printf("set userOutputValue fail.\n");
        return -1;
    }

    CThread::sleep(nTimeMS);

    if (false == booleanNode.setValue(false))
    {
        printf("set userOutputValue fail.\n");
        return -1;
    }
}

/* 44、获取外部光源曝光时间（输出信号的时间由软件侧控制） */

/* 45、设置X轴翻转 */
static int32_t setReverseX(ICameraPtr& cameraSptr, bool flag)
{
	IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);	

	CBoolNode boolNodeReverseX = sptrImageFormatControl->reverseX();
	if(!boolNodeReverseX.setValue(flag))
	{
		printf("set reverseX fail.\n");
		return -1;
	}

	return 0;
}

/* 46、设置Y轴翻转 */
static int32_t setReverseY(ICameraPtr& cameraSptr, bool flag)
{
	IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);	

	CBoolNode boolNodeReverseY = sptrImageFormatControl->reverseY();
	if(!boolNodeReverseY.setValue(flag))
	{
		printf("set reverseY fail.\n");
		return -1;
	}

	return 0;
}

/* 47、当相机与网卡处于不同网段时，自动设置相机IP与网卡处于同一网段 （与相机连接之前调用）*/
static int32_t autoSetCameraIP(ICameraPtr& cameraSptr)
{
	IGigECameraPtr gigeCameraPtr = IGigECamera::getInstance(cameraSptr);
	if(NULL == gigeCameraPtr)
	{
		return -1;
	}

	//获取Gige相机相关信息
	CString ip = gigeCameraPtr->getIpAddress();
	CString subnetMask = gigeCameraPtr->getSubnetMask();
	CString gateway = gigeCameraPtr->getGateway();
	CString macAddress = gigeCameraPtr->getMacAddress();
	printf("ip address is %s.\r\n", ip.c_str());
	printf("subnetMask is %s.\r\n", subnetMask.c_str());
	printf("gateway is %s.\r\n", gateway.c_str());
	printf("macAddress is %s.\r\n", macAddress.c_str());
	printf("\n");

	unsigned long devIpValue = ntohl(inet_addr(gigeCameraPtr->getIpAddress().c_str()));
	unsigned long devSubMaskValue = ntohl(inet_addr(gigeCameraPtr->getSubnetMask().c_str()));

	//获取对应接口的网卡信息
	IGigEInterfacePtr gigeInterfaceSPtr = IGigEInterface::getInstance(cameraSptr);
	if(NULL == gigeInterfaceSPtr)
	{
		return -1;
	}

	CString interfaceIp = gigeInterfaceSPtr->getIpAddress();
	CString interfaceSubnetMask = gigeInterfaceSPtr->getSubnetMask();
	CString interfaceGateway = gigeInterfaceSPtr->getGateway();
	CString interfaceMacAddress = gigeInterfaceSPtr->getMacAddress();
	printf("ip address of interface  is %s.\r\n", interfaceIp.c_str());
	printf("subnetMask of interface is %s.\r\n", interfaceSubnetMask.c_str());
	printf("gateway of interface is %s.\r\n", interfaceGateway.c_str());
	printf("macAddress of interface is %s.\r\n", interfaceMacAddress.c_str());
	printf("\n");

	unsigned long InterfaceIpValue = ntohl(inet_addr(gigeInterfaceSPtr->getIpAddress().c_str()));
	unsigned long InterfaceSubMaskValue = ntohl(inet_addr(gigeInterfaceSPtr->getSubnetMask().c_str()));

	if( (devIpValue & devSubMaskValue) != (InterfaceIpValue & InterfaceSubMaskValue) )
	{
		//设备与网卡不在同一网段，强制设置设备与网卡在同一网段
		unsigned char newIPStr[20] = {0};

		while(1)
		{
			unsigned long newIpValue = rand() % 254 + 1; //1~254
			if(newIpValue != (InterfaceIpValue & 0xff))
			{
				newIpValue = (InterfaceIpValue & 0xffffff00) + newIpValue;
				struct in_addr   stInAddr;
				stInAddr.s_addr	= htonl(newIpValue);
				memcpy(newIPStr, inet_ntoa(stInAddr), strlen(inet_ntoa(stInAddr)));
				break;
			}
		}

		if(!gigeCameraPtr->forceIpAddress((const char*)newIPStr, gigeInterfaceSPtr->getSubnetMask().c_str(), gigeInterfaceSPtr->getGateway().c_str()))
		{
			printf("Set device ip failed.\n");
			return -1;
		}
	}

	return 0;
}

/* 48、设置相机IP （与相机连接之前调用）*/
static int32_t setCameraIp(ICameraPtr& cameraSptr, char* ipAddress, char* subnetMask, char* gateway)
{
	IGigECameraPtr gigeCameraPtr = IGigECamera::getInstance(cameraSptr);
	if(NULL == gigeCameraPtr)
	{
		return -1;
	}

	if(!gigeCameraPtr->forceIpAddress(ipAddress, subnetMask, gateway))
	{
		printf("Set device ip failed.\n");
		return -1;
	}

	return 0;
}

/* 49、设置相机静态IP （与相机连接之后调用）*/
static int32_t setCameraPersistentIP(ICameraPtr& cameraSptr)
{
	IGigECameraPtr gigeCameraPtr = IGigECamera::getInstance(cameraSptr);
	if(NULL == gigeCameraPtr)
	{
		printf("gigeCameraPtr is null.\n");
		return -1;
	}

	ITransportLayerControlPtr transportLayerControlPtr= CSystem::getInstance().createTransportLayerControl(cameraSptr);

	if(NULL == transportLayerControlPtr)
	{
		printf("transportLayerControlPtr is null.\n");
		return -1;
	}

	transportLayerControlPtr->gevCurrentIPConfigurationPersistentIP().setValue(true);
	transportLayerControlPtr->gevPersistentDefaultGateway().setValue(gigeCameraPtr->getGateway().c_str());
	transportLayerControlPtr->gevPersistentIPAddress().setValue(gigeCameraPtr->getIpAddress().c_str());
	transportLayerControlPtr->gevPersistentSubnetMask().setValue(gigeCameraPtr->getSubnetMask().c_str());

	return 0;
}

static void modifyCamralExposureTime(CSystem &systemObj, ICameraPtr& cameraSptr)
{
    IAcquisitionControlPtr sptrAcquisitionControl = systemObj.createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return;
    }

    double exposureTimeValue = 0.0;
    CDoubleNode exposureTime = sptrAcquisitionControl->exposureTime();

    exposureTime.getValue(exposureTimeValue);
    printf("before change ,exposureTime is %f. thread ID :%d\n", exposureTimeValue, CThread::getCurrentThreadID());

    exposureTime.setValue(exposureTimeValue + 2);
    exposureTime.getValue(exposureTimeValue);
    printf("after change ,exposureTime is %f. thread ID :%d\n", exposureTimeValue, CThread::getCurrentThreadID());
}

void LogPrinterFunc(const char* log)
{
	return;
}
#endif
