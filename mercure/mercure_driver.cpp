
/*-------------------------------------------------------------
brief  :  Mercure Camera driver 
author :  XDU Robomaster, skystarry
date   :  2019.05.29
---------------------------------------------------------------*/
#include "mercure_driver.h"

namespace camera{

MercureDriver::MercureDriver():
    status_(GX_STATUS_SUCCESS),
    device_(nullptr),
    colorfilter_(GX_COLOR_FILTER_NONE),
    pFrameBuffer_(nullptr),
    bufferNum_(5)
{ 
  init_sdk();
  LoadParam();
  Image_.create(1200, 1920, CV_8UC3);
}

GX_STATUS MercureDriver::init_sdk()
{
    ROS_WARN("Mercure Camera Initializing......\n");
    uint32_t ui32DeviceNum = 0;
    
    status_ = GXInitLib(); 
    if(status_ != GX_STATUS_SUCCESS)
    {
      ROS_ERROR("Initialize Mercure Camera Driver libary failed. Error Code: %d", status_);
      return status_;
    }

    status_ = GXUpdateDeviceList(&ui32DeviceNum, 1000);
    if(status_ != GX_STATUS_SUCCESS)
    { 
        ROS_ERROR("Get Mercure Camera device enumerated number failed. Error Code: %d", status_);
        GXCloseLib();
        return status_;
    }

    if(ui32DeviceNum <= 0)
    {
        ROS_WARN("No Mercure Camera device found.");
        GXCloseLib();
        return status_;
    }

    status_ = GXOpenDeviceByIndex(1, &device_);

    if(status_ != GX_STATUS_SUCCESS)
    {
        ROS_ERROR("Open Mercure Camera failed. Error Code: %d", status_);
        GXCloseLib();
        return status_;           
    }
    
    GetVision();

    status_ = GXGetEnum(device_, GX_ENUM_PIXEL_COLOR_FILTER, &colorfilter_);    
    status_ = GXGetInt(device_, GX_INT_PAYLOAD_SIZE, &payloadsize_);
    status_ = GXSetEnum(device_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    status_ = GXSetEnum(device_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    
    status_ = GXSetAcqusitionBufferNumber(device_, bufferNum_);

    bool bStreamTransferSize = false;
    status_ = GXIsImplemented(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
    if(bStreamTransferSize) {
        //Set size of data transfer block
        status_ = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
    }

    bool bStreamTransferNumberUrb = false;
    status_ = GXIsImplemented(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
    if(bStreamTransferNumberUrb) {
        //Set qty. of data transfer block
        status_ = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
    }

    rgbImagebuf_ = new uint8_t[payloadsize_ * 3]; 
}

void MercureDriver::LoadParam()
{
    std::string file_name = ros::package::getPath("roborts_camera") + "/config/mercure.xml";
    cv::FileStorage fs(file_name, cv::FileStorage::READ);
    if(!fs.isOpened())
        ROS_ERROR ("Cannot open mercure.xml, please check if the file is exist");

    fs["exposure_auto"] >> exp_auto_;
    fs["exposure_time"] >> exp_time_;
    
    if(exp_auto_)
        status_ = GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    else{
        status_ = GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
        status_ = GXSetFloat(device_, GX_FLOAT_EXPOSURE_TIME, exp_time_);
    }
    
    fs["w_auto"]  >> w_auto_;
    fs["w_red"]   >> w_red_;
    fs["w_green"] >> w_green_;
    fs["w_blue"]  >> w_blue_;

    status_ = GXSetInt(device_, GX_INT_AWBROI_WIDTH,   1920);
        status_ = GXSetInt(device_, GX_INT_AWBROI_HEIGHT,  1200);
        status_ = GXSetInt(device_, GX_INT_AWBROI_OFFSETX, 0);
        status_ = GXSetInt(device_, GX_INT_AWBROI_OFFSETY, 0);
        
    if(w_auto_){
        

        status_ = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        
        //status_ = GXSetEnum(device_, GX_ENUM_AWB_LAMP_HOUSE, GX_AWB_LAMP_HOUSE_FLUORESCENCE);
        /*
        GX_AWB_LAMP_HOUSE_ADAPTIVE                     = 0,           ///< Adaptive light source
        GX_AWB_LAMP_HOUSE_D65                          = 1,           ///< Color temperature 6500k
        GX_AWB_LAMP_HOUSE_FLUORESCENCE                 = 2,           ///< Fluorescent
        GX_AWB_LAMP_HOUSE_INCANDESCENT                 = 3,           ///< Incandescent
        GX_AWB_LAMP_HOUSE_D75                          = 4,           ///< Color temperature 7500k
        GX_AWB_LAMP_HOUSE_D50                          = 5,           ///< Color temperature 5000k
        GX_AWB_LAMP_HOUSE_U30                          = 6,           ///< Color temperature 3000k
        */
    }
    else{
        status_ = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);

        status_ = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
        status_ = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, w_red_);

        status_ = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
        status_ = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, w_green_);

        status_ = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
        status_ = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, w_blue_);
    }


    fs["gain_auto"] >> gain_auto_;
    fs["gain"]      >> gain_;
    
    if(gain_auto_)
    {
        status_ = GXSetEnum(device_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
    }
    else{
        status_ = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
        //status_ = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_RED);
        //status_ = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_GREEN);
        //status_ = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_BLUE);
        //获 取 增 益 调 节 范 围
        //GX_FLOAT_RANGE gainRange;
        //status_ = GXGetFloatRange(device_, GX_FLOAT_GAIN, &gainRange);
        //设 置 最 小 增 益 值
        //status_ = GXSetFloat(device_, GX_FLOAT_GAIN, gainRange.dMin);
        //status_ = GXSetFloat(device_, GX_FLOAT_GAIN, gainRange.dMax);
    }
    /*
    if(black_auto_){
        status_ = GXSetEnum(device_, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_CONTINUOUS);
    }else{
        status_ = GXSetEnum(device_, GX_ENUM_BLACKLEVEL_SELECTOR, GX_BLACKLEVEL_SELECTOR_ALL);
        //status_=GXSetEnum(device_, GX_ENUM_BLACKLEVEL_SELECTOR, GX_BLACKLEVEL_SELECTOR_RED);
        //status_=GXSetEnum(device_, GX_ENUM_BLACKLEVEL_SELECTOR, GX_BLACKLEVEL_SELECTOR_GREEN);
        //status_=GXSetEnum(device_, GX_ENUM_BLACKLEVEL_SELECTOR, GX_BLACKLEVEL_SELECTOR_BLUE);
        //获 取 黑 电 平 调 节 范 围
        //GX_FLOAT_RANGE blackLevelRange;
        //blackLevelRange.dMin
        //blackLevelRange.dMax
        //status_ = GXGetFloatRange(device_, GX_FLOAT_BLACKLEVEL, &blackLevelRange);
        //status_ = GXSetFloat(device_, GX_Float_BLACKLEVEL, blackLevelRange.dMin);
        //status_ = GXSetFloat(device_, GX_Float_BLACKLEVEL, blackLevelRange.dMax);
    }*/

}

void MercureDriver::GetVision()
{
    ROS_WARN("<Libary Version : %s>", GXGetLibVersion());
    size_t nSize = 0;
    GXGetStringLength(device_, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
    char *pszVendorName = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
    ROS_WARN("<Vendor Name : %s>", pszVendorName);
    delete[] pszVendorName;
    pszVendorName = NULL;

    GXGetStringLength(device_, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    char *pszModelName = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
    ROS_WARN("<Model Name : %s>", pszModelName);
    delete[] pszModelName;
    pszModelName = NULL;

    GXGetStringLength(device_, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
    char *pszSerialNumber = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
    ROS_WARN("<Serial Number : %s>", pszSerialNumber);
    delete[] pszSerialNumber;
    pszSerialNumber = NULL;

    GXGetStringLength(device_, GX_STRING_DEVICE_VERSION, &nSize);
    char *pszDeviceVersion = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
    ROS_WARN("<Device Version : %s>", pszDeviceVersion);
    delete[] pszDeviceVersion;
    pszDeviceVersion = NULL;
}

void MercureDriver::ReadCamera()
{
    status_ = GXStreamOn(device_);

    if(status_ != GX_STATUS_SUCCESS)
    {
      if (rgbImagebuf_ != nullptr)
      {
        delete[] rgbImagebuf_;
        rgbImagebuf_ = nullptr;
      }
      GXCloseDevice(device_);
      device_ = nullptr;
      GXCloseLib();
      ROS_ERROR("Exit!");  
    }
    
    while(true)
    {
      auto speed_test_start_begin_time = std::chrono::steady_clock::now();

      status_ = GXDQBuf(device_, &pFrameBuffer_, 1000);

      DxRaw8toRGB24((uint8_t*)pFrameBuffer_->pImgBuf, 
                        rgbImagebuf_, 
                        pFrameBuffer_->nWidth,
                        pFrameBuffer_->nHeight,
                        RAW2RGB_NEIGHBOUR, 
                        DX_PIXEL_COLOR_FILTER(colorfilter_), 
                        false);

      memcpy(Image_.data, rgbImagebuf_, pFrameBuffer_->nHeight*pFrameBuffer_->nWidth*3);
      cv::imshow("RGB", Image_);
      cv::waitKey(1);

      status_ = GXQBuf(device_, pFrameBuffer_);

      auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0;
      ROS_INFO("time cost = %.2f ms", cost);	
    }

    status_ = GXStreamOff(device_);
    if(status_ != GX_STATUS_SUCCESS)
    {
      delete[] rgbImagebuf_;
      rgbImagebuf_ = nullptr;
      GXCloseDevice(device_);
      device_ = nullptr;
      GXCloseLib();
      ROS_ERROR("Exit!");  
    }
}

MercureDriver::~MercureDriver()
{
    delete[] rgbImagebuf_;
    rgbImagebuf_ = nullptr;

    status_ = GXCloseDevice(device_);
    if(status_ != GX_STATUS_SUCCESS)
    {
        ROS_ERROR("GXCloseDevice failed. ");
        device_ = nullptr;
        GXCloseLib();
    }
    status_ = GXCloseLib();
    if(status_ != GX_STATUS_SUCCESS)
    {
        ROS_ERROR("GXCloseLib failed. ");
    }
}

} // namespace camera