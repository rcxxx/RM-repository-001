#ifndef RM_ARMORFITTED_H
#define RM_ARMORFITTED_H
#include "configure.h"
#include "serialport.h"
#include "rm_kalmanfitted.h"

class RM_ArmorFitted
{
private:
    /** param initial **/
    Mat src_img;    //原图
    Mat gray_img;   //灰度图
    Mat bin_img;    //二值图
    Mat dst_img;    //输出图
    int threshold_value_blue = 80;
    int threshold_Value_red = 80;
    /** param initial **/
    RM_klamanFilter kf;

public:
    /**
     * @brief 执行函数部分
     */
    RM_ArmorFitted() :kf()
    {
        cout<<"Armor is ready"<<endl;
    }
    void imageProcessing(Mat frame);
    void armorFitted();
    SerialPort port;

public:
    /**
     *@brief: 功能函数部分
     */
    float centerDistance(Point p1,Point p2);
    float distancetoCamera(const RotatedRect &rect1, const RotatedRect &rect2);
    bool iscentral_region(Mat src, Point point, const float ranging_results);
    bool isarmoredColorroi(const RotatedRect &rect);
    bool catchlightState(const RotatedRect &rect);
    bool can_contour_match(const RotatedRect &rect1, const RotatedRect &rect2);
};

#endif // RM_ARMORFITTED_H
