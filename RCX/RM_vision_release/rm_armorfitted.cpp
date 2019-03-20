#include "rm_armorfitted.h"

float RM_ArmorFitted::centerDistance(Point p1, Point p2)
/**
  @brief: get distance between two Point
  -------------------------------
  @param: Point1,Point2
  -------------------------------
  @return: distance of Point @typedef:float
**/
{
    float D = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
    return D;
}

float RM_ArmorFitted::distancetoCamera(const RotatedRect &rect1, const RotatedRect &rect2)
/**
 * @brief: 模糊单目测距函数
 * -------------------------------------------------
 * @param: rect1,rect2 输入的两个 minAreaRect
 * -------------------------------------------------
 * @return: Distance_to_camera 距离　@typedef: float
 * -------------------------------------------------
 * @author: jiajia
 * -------------------------------------------------
 * @note: 该函数为模糊测距，仅为参考值，误差在　0-20cm左右
 */
{
    float  Distance_to_camera;
    float rects_distance = centerDistance(rect1.center,rect2.center);
    unsigned int half_Perimeter = rect1.size.width+rect1.size.height;
    unsigned int true_length;
    float distance_Perimeter_ratio = rects_distance / (float)half_Perimeter;
    if(distance_Perimeter_ratio  >3.6550)
    {
        true_length =2300 ;     /** big armor **/
    }
    else
    {
         true_length =1350;     /** little armor **/
    }
     //float  focal_length  = 7.00;   //普通相机焦距 6mm
     float  focal_length  = 10.3;   //工业相机焦距 12mm
     Distance_to_camera = (true_length * focal_length)/rects_distance;
     return Distance_to_camera;
}

bool RM_ArmorFitted::isarmoredColorroi(const RotatedRect &rect)
/**
  @brief: HSV颜色空间检测 ROI区域
  --------------------------------
  @param: rect:目标区域 minAreaRect
  --------------------------------
  @return: is_color  @typedef: bool
  --------------------------------
  @author: Rcxxx
**/
{
    bool is_color = false;
    Mat roi_img;
    Mat hsv_roi_img;
    Point2f verices[4];
    Point2f verdst[4];
    unsigned int roi_w;
    unsigned int roi_h;
    rect.points(verices);
    if(rect.size.width > rect.size.height)
    {
        roi_w = rect.size.height;
        roi_h = rect.size.width;
        verdst[0] = Point2f(0,roi_h);
        verdst[1] = Point2f(0,0);
        verdst[2] = Point2f(roi_w,0);
        verdst[3] = Point2f(roi_w,roi_h);
    }
    else
    {
        roi_w = rect.size.width;
        roi_h = rect.size.height;
        verdst[0] = Point2f(roi_w,roi_h);
        verdst[1] = Point2f(0,roi_h);
        verdst[2] = Point2f(0,0);
        verdst[3] = Point2f(roi_w,0);
    }

    roi_img = Mat(roi_h,roi_w,CV_8UC1);
    Mat warpMatrix = getPerspectiveTransform(verices,verdst);
    warpPerspective(src_img,roi_img,warpMatrix,roi_img.size(),INTER_LINEAR, BORDER_CONSTANT);

    cvtColor(roi_img,hsv_roi_img,COLOR_BGR2HSV);
    float H=0.0,S=0.0,V=0.0;
    unsigned int flag = 0;
    for(int x = 0;x < hsv_roi_img.cols; ++x)
    {
        for(int y = 0;y < hsv_roi_img.rows; ++y)
        {
            H = hsv_roi_img.at<Vec3b> (y,x)[0];
            S = hsv_roi_img.at<Vec3b> (y,x)[1];
            V = hsv_roi_img.at<Vec3b> (y,x)[2];
            //red
            if(armor_color == 0)
            {
                if((H>=145 && H<180)||(H>=0 && H<=13))
                {   if(S >= 135 && S <= 255)
                    {   if(V > 148 && V <= 255)
                        {
                            flag += 1;
                        }
                    }
                }
            }
            //blue
            else
            {
                if(H>=75 && H<=130)
                {   if(S >= 195 && S <= 255)
                    {   if(V >= 185 && V <= 255)
                        {
                            flag += 1;
                        }
                    }
                }
            }
            if(armor_color == 0)
            {
                if((flag / hsv_roi_img.cols*hsv_roi_img.rows) > 0.5)
                {
                    is_color = 1;
                    continue;
                }
            }
            else
            {
                if((flag / hsv_roi_img.cols*hsv_roi_img.rows) > 0.3)
                {
                    is_color = 1;
                    continue;
                }
            }
        }
    }
    return is_color;
}

bool RM_ArmorFitted::catchlightState(const RotatedRect &rect)
/**
  @brief: 不同位置、不同条件下灯条的模糊参数
  ----------------------------------
  @param: rect: 目标区域的 minAreaRect
  ----------------------------------
  @return: is_target_rect @typedef: bool
  --------------------------------------
  @author: Rcxxx
**/
{
    bool is_target_rect = false;
    float rect_w;
    float rect_h;
    if (rect.size.height > rect.size.width)
    {
        rect_h = rect.size.height;
        rect_w = rect.size.width;
    }
    else
    {
        rect_w = rect.size.height;
        rect_h = rect.size.width;
    }
    unsigned int return_value;
    float ratio = (float)rect_w/(float)rect_h;

    if(ratio <= 0.2)
    {
        return_value = 1;
    }
    else if (ratio <= 0.25)
    {
        return_value = 2;
    }
    else if (ratio <= 0.3)
    {
        return_value = 3;
    }
    else if (ratio <= 0.4)
    {
        return_value = 4;
    }
    else
    {
        return_value = 5;
    }

    switch (return_value)
    {
    case 1:{
        if (ratio < 0.7)
            is_target_rect = true;
    }
        break;
    case 2:{
        if (ratio < 0.8)
            is_target_rect = true;
    }
        break;
    case 3:{
        if (ratio < 0.9)
            is_target_rect = true;
    }
        break;
    case 4:{
        if (ratio < 1)
            is_target_rect = true;
    }
        break;
    default:{}
        break;
    }
    return is_target_rect;
}

bool RM_ArmorFitted::can_contour_match(const RotatedRect &rect1, const RotatedRect &rect2)
/**
  @brief: 轮廓匹配函数
  ---------------------------------------------
  @param: rect1,rect2 需要匹配的两个　minAreaRect
  ---------------------------------------------
  @return: isContour_matching  @typedef: bool
  ---------------------------------------------
  @authors: Rcxxx
            jiajia
  */
{
    bool iscontour_matching = false;

    /** get area ratio **/
    float area_rect1 = (float)rect1.size.width*(float)rect1.size.height;
    float area_rect2 = (float)rect2.size.width*(float)rect2.size.height;
    float area_ratio = area_rect1>area_rect2 ? (area_rect2/area_rect1):(area_rect1/area_rect2);
    /** get area ratio **/
    if(AREA_RATIO_MIN < area_ratio && area_ratio < AREA_RATIO_MAX)
    {
        /** exchange width and height **/
        float w1 = (rect1.size.height > rect1.size.width ? rect1.size.width:rect1.size.height);
        float h1 = (rect1.size.height > rect1.size.width ? rect1.size.height:rect1.size.width);
        float w2 = (rect1.size.height > rect2.size.width ? rect2.size.width:rect2.size.height);
        float h2 = (rect2.size.height > rect2.size.width ? rect2.size.height:rect2.size.width);
        /** exchange width and height **/

        /*- get center -*/
        float x1 = rect1.center.x;
        float y1 = rect1.center.y;
        float x2 = rect2.center.x;
        float y2 = rect2.center.y;
        /*- get center -*/
        float center_slope = fabs((y1-y2)/(x1-x2));

        if(center_slope <= CENTER_SLPOE_MAX || center_slope == 0)
        {
            float center_distance = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
            float low = max(w1,w2) * LOW_MULTIPLE;
            float high = max(w1,w2) * HIGH_MULTIPLE;
            if(low < center_distance && center_distance < high)
            {
                float max_h = h1>h2 ? h1:h2;
                if(center_distance > max_h && center_distance < max_h*8)
                {
                    iscontour_matching = true;
                }
            }
        }
    }
    else
    {
        iscontour_matching = false;
    }
    return iscontour_matching;
}

bool RM_ArmorFitted::iscentral_region(Mat src, Point point, const float ranging_results)
/**
 * @brief: 无效区域判断函数
 * ------------------------------------------------
 * @param: src 输入图像用于获取尺寸　Point 当前点坐标值
 * @param: ranging_results 测距结果，使区域大小自适应
 * ------------------------------------------------
 * @return: iscentral_region_point  @typedef: bool
 * ------------------------------------------------
 * @authors: Rcxxx
 */
{
    bool iscentral_region_point;
    Point center = Point(src.cols/2,src.rows/2);
    unsigned int left_focus_reduce;
    unsigned int left_focus_add;
    unsigned int ellipse_long_axis;
    left_focus_reduce =(unsigned int)(ranging_results * 0);
    left_focus_add =(unsigned int)(ranging_results * 0);
    ellipse_long_axis =(unsigned int)(ranging_results * 0);
    Point left_focus = Point(center.x-left_focus_reduce,center.y);
    Point right_focus = Point(center.x+left_focus_add,center.y);
    float PF1 = centerDistance(point,left_focus);
    float PF2 = centerDistance(point,right_focus);
    if((PF1 + PF2) < ellipse_long_axis)
    {
        iscentral_region_point = true;
    }
    else
    {
        iscentral_region_point = false;
    }
    ellipse(dst_img,point,Size(ranging_results,ranging_results*0.5),0,0,360,Scalar(255,129,0),2,8);
    return iscentral_region_point;
}

void RM_ArmorFitted::imageProcessing(Mat frame)
/**
 * @brief: 图像预处理函数
 *-------------------------------
 * @param: src_img 输入图像 @typedef: Mat
**/
{
    src_img = frame;
    int threshold_Value;
    if(armor_color == 0)
    {
        threshold_Value = threshold_Value_red;
    }
    else
    {
        threshold_Value = threshold_value_blue;
    }
    resize(src_img,src_img,Size(640,480),INTER_NEAREST);
    src_img.copyTo(dst_img);
    /** -------------Image Segmentation	---------------**/
    cvtColor(src_img, gray_img, COLOR_BGR2GRAY);
    threshold(gray_img, bin_img, threshold_Value, 255, THRESH_BINARY);
    medianBlur(bin_img, bin_img,5);
    Canny(bin_img,bin_img,120,240);
    /** -------------Image Segmentation	---------------**/
}

void RM_ArmorFitted::armorFitted()
/**
  @brief: 识别部分执行函数
  -------------------------------------
  @param: bin_img 预处理结束后的二值图
  -------------------------------------
  @note:
  -------------------------------------
  @authors: Rcxxx
            jiajia
  */
{
//    bool success_send = false;
//    int sendbuff_COUNT = 0;

    float t1 = getTickCount();
    vector<vector<Point> > contours;
    vector<Rect> boundRect;
    vector<RotatedRect> rotateRect;
    vector<Vec4i> hierarchy;
    vector<Point2f> midPoint(2);
    vector<vector<Point2f> > midPoint_pair;
    vector<float> fuzzy_distance;    

    findContours(bin_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0,0));
    for (unsigned int i = 0;i<(unsigned int)contours.size(); ++i)
    {
        if (contours.size() <= 1)
            break;
        Rect B_rect_i = boundingRect(contours[i]);
        RotatedRect R_rect_i = minAreaRect(contours[i]);
        if(B_rect_i.height >= B_rect_i.width)
        {
            if (catchlightState(R_rect_i))
            {
                boundRect.push_back(B_rect_i);
                rotateRect.push_back(R_rect_i);
            }
        }
    }
    float distance_max = 0.f;
    for (unsigned int k1 =0; k1<(unsigned int)rotateRect.size(); ++k1)
    {
        if(rotateRect.size()<=1)
            break;
        for (unsigned int k2 = k1+1; k2 <(unsigned int)rotateRect.size(); ++k2)
        {
            if(can_contour_match(rotateRect[k1],rotateRect[k2]))
            {
                float distance_temp = centerDistance(rotateRect[k1].center,rotateRect[k2].center);
                if(isarmoredColorroi(rotateRect[k1]) && isarmoredColorroi(rotateRect[k2]))
                {
                    /** change max distance **/
                    if(distance_temp >= distance_max)
                    {
                        distance_max = distance_temp;
                    }
                    /** change max distance **/

                    /** get fuzzy distance **/
                    float test_fuzzy_distance;
                    test_fuzzy_distance = distancetoCamera(rotateRect[k1],rotateRect[k2]);
                    /** get fuzzy distance **/
                    rectangle(dst_img,boundRect[k1].tl(), boundRect[k1].br(), Scalar(0,255,255),2,8,0);
                    rectangle(dst_img,boundRect[k2].tl(), boundRect[k2].br(), Scalar(0,255,255),2,8,0);
                    midPoint[0] = rotateRect[k1].center;
                    midPoint[1] = rotateRect[k2].center;
                    midPoint_pair.push_back(midPoint);
                    fuzzy_distance.push_back(test_fuzzy_distance);
                }
            }
        }
    }

    for(unsigned int k3 = 0; k3 < (unsigned int)midPoint_pair.size(); ++k3)
    {
        float midPoint_distance = centerDistance(midPoint_pair[k3][0],midPoint_pair[k3][1]);
        float midPoint_slope = fabs((midPoint_pair[k3][0].y-midPoint_pair[k3][1].y)/(midPoint_pair[k3][0].x-midPoint_pair[k3][1].x));
        if(midPoint_slope < CENTER_SLPOE_MAX)
        {
            if(midPoint_distance >= distance_max)
            {
                Point2f true_center = Point2f((midPoint_pair[k3][0].x+midPoint_pair[k3][1].x)/2,(midPoint_pair[k3][0].y+midPoint_pair[k3][1].y)/2);
                rectangle(dst_img,
                          Point(true_center.x-int(midPoint_distance*0.47),true_center.y-int(midPoint_distance*0.2)),
                          Point(true_center.x+int(midPoint_distance*0.47),true_center.y+int(midPoint_distance*0.2)),
                          Scalar(120,255,150),2,8,0);
                float test_fuzzy_distance = fuzzy_distance[k3];
                bool is = iscentral_region(src_img,true_center,test_fuzzy_distance);
                float t2 = getTickCount();
                float runtime = (t2-t1)/getTickFrequency();

                Point2f new_center = kf.point_Predict(1.3,true_center);
                circle(dst_img,new_center,5,Scalar(120,120,150),2,8,0);
                rectangle(dst_img,
                          Point(new_center.x-int(midPoint_distance * 0.47),new_center.y-int(midPoint_distance*0.2)),
                          Point(new_center.x+int(midPoint_distance * 0.47),new_center.y+int(midPoint_distance*0.2)),
                          Scalar(120,120,150),2,8,0);
            }
        }
    }
    imshow("th",bin_img);
    imshow("input" ,src_img);
    imshow("output",dst_img);
}
