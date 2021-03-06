#include "rm_videocapture.h"
#include "configure.h"

RM_VideoCapture::RM_VideoCapture(int cameramode)
/**
  @brief: 相机类构造函数
  ----------------------------------
  @param: cameramode 选择相机型号
  */
{
    if(cameramode == 0)
    {
        cameraSet();
        iscamera0_open = true;
        cout<<"Set camera Industrial camera"<<endl;
    }
    else
    {
        iscamera0_open = false;
        cout<<"set camera USB camera"<<endl;
    }
}

RM_VideoCapture::~RM_VideoCapture()
{
    if(iscamera0_open)
    {
        CameraUnInit(hCamera);
        //注意，现反初始化后再free
        free(g_pRgbBuffer);
        cout<<"release Industry camera success......"<<endl;
    }
    else
    {
        cout<<"release USB camera success......"<<endl;
    }
}

bool RM_VideoCapture::isindustryimgInput()
/**
 * @brief: 工业相机的图像转到指针中，可转换为 @typedef: cv::Mat
 * ------------------------------------------------------------
 * @param: iscamera0_open 是否启用工业相机
 * ------------------------------------------------------------
 * @return: isindustry_camera_open 是否启用成功
 */
{
    bool isindustry_camera_open = false;
    if(iscamera0_open == 1)
    {
        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
        {
            //----------读取原图----------//
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
            if (iplImage)
            {
                cvReleaseImageHeader(&iplImage);
            }
            iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,channel);
            cvSetData(iplImage,g_pRgbBuffer,sFrameInfo.iWidth*channel);//此处只是设置指针，无图像块数据拷贝，不需担心转换效率
        }
        isindustry_camera_open = true;
    }
    else
    {
        isindustry_camera_open = false;
    }
    return isindustry_camera_open;
}

int RM_VideoCapture::cameraSet()
/**
 *@brief: 相机参数的初始化
 *--------------------------------------------------------
 *@return: 1 Success
 *        -1 fail
 *--------------------------------------------------------
**/
{
    CameraSdkInit(1);
    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
    printf("state = %d\n", iStatus);
    printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if(iCameraCounts==0)
    {
        return -1;
    }
    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);
    //初始化失败
    printf("state = %d\n", iStatus);
    if(iStatus!=CAMERA_STATUS_SUCCESS)
    {
        return -1;
    }
    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    cout<<CameraGetAeState(hCamera,&AEstate);
    cout<<CameraSetAeState(hCamera,FALSE);
    if(armor_color == 0)
    {
        CameraSetExposureTime(hCamera,350);
    }
    else
    {
        CameraSetExposureTime(hCamera,350);
    }
    /*让SDK进入工作模式，开始接收来自相机发送的图像数据。
     *如果当前相机是触发模式，则需要接收到触发帧以后才会更新图像*/
    CameraPlay(hCamera);
    /*
    其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */
    if(tCapability.sIspCapacity.bMonoSensor)
    {
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else
    {
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }
    return 1;
}

void RM_VideoCapture::cameraReleasebuff()
/**
 *@brief: 释放相机缓存数据
 *------------------------------------
 * @param: iscamera0_open 是否启用工业相机
*/
{
    if(iscamera0_open)
    {
        //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
        //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
        CameraReleaseImageBuffer(hCamera,pbyBuffer);
    }
}
