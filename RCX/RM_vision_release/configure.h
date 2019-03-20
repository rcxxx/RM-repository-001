#ifndef CONFIGURE_H
#define CONFIGURE_H

#include "CameraApi.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <math.h>
#include <iostream>
#include <fcntl.h>  //文件控制定义
#include <termios.h>   //POSIX终端控制定义
#include <unistd.h>    //UNIX标准定义
#include <errno.h>     //ERROR数字定义
#include <sys/select.h>

using namespace std;
using namespace cv;

#define capture_defult 0    //相机的默认值
#define armor_color 0       //装甲颜色 red=0  blue=1
#define serialisopen 1      //是否启用串口

/** ------------------------------------------------------**/
/**
  @brief:　轮廓匹配部分参数
  @param: AREA_RATIO_MIN  面积之比的最小阈值
  @param: AREA_RATIO_MAX  面积之比的最大阈值
  @param: LOW_MULTIPLE  中点距离最小阈值的倍率参数
  @param: HIGH_MULTIPLE  中点距离最大阈值的倍率参数
  @param: DIFF_RATIO  宽/高 之差与最大 宽/高　之间的比值
*/
#define AREA_RATIO_MIN 0.4
#define AREA_RATIO_MAX 1.1
#define LOW_MULTIPLE 2.110
#define HIGH_MULTIPLE 28.888
#define CENTER_SLPOE_MAX 0.45
/** ------------------------------------------------------**/

#endif // CONFIGURE_H
