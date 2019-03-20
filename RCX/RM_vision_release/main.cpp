/**
  @param:When formal use Switch to the Release in configure.h
********************************************************
  @brief:RM vision overall code
********************************************************
  @param:
********************************************************
  @author:rcxxx_
  */
#include "configure.h"
#include "rm_videocapture.h"
#include "serialport.h"
#include "rm_armorfitted.h"
#include "rm_bigchrysanthemum.h"

int main()
{
    /** Camera Srart **/
    VideoCapture capture(capture_defult);
    RM_VideoCapture cap(1);
    /** Camera Srart **/

    /** The Serial Port Part **/
    SerialPort port;
    port.serialSet(1);
    /** The Serial Port Part **/

    /** param initial **/
    Mat src_img;

    /** param initial **/

    /** function initial **/
    RM_ArmorFitted armor;
    RM_BigChrysanthemum agency;
    /** function initial **/

    for(;;)
    {
        /** Param Set **/
        if(cap.isindustryimgInput())
        {
            src_img = cvarrToMat(cap.iplImage,true);//这里只是进行指针转换，将IplImage转换成Mat类型
        }
        else
        {
            capture >> src_img;
        }
        //Receive Serial Data
        unsigned int receive_serial_data_No_ = 1;// default_mode;//有默认值
        /** Change Mode　**/
        switch (receive_serial_data_No_)
        {
        /**-Support Shooting mode-**/
        case 1:
        {
            armor.imageProcessing(src_img);
            armor.armorFitted();
        }   /**-Support Shooting mode-**/
            break;
        /** Energy Agency Mode **/
        case 2:
        {

        }   /** Energy Agency Mode **/
            break;
        /**-Empty mode-**/
        default:
            /**-Empty mode-**/
            break;
        }   /** Change Mode　**/

        int key = waitKey(1);
        if(char(key) == 27)
        {
            cap.cameraReleasebuff();
            break;
        }
        else
        {
            cap.cameraReleasebuff();
        }
    }
    /**  **/


    return 1;
}
