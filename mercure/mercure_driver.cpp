
/*-------------------------------------------------------------
brief  :  Mercure Camera driver 
author :  XDU Robomaster, skystarry
date   :  2019.05.29
---------------------------------------------------------------*/
#include "mercure_driver.h"

MercureDriver::MercureDriver()
{ 
  
  status_ = GX_STATUS_SUCCESS;
  device_ = NULL;  
  g_i64ColorFilter = GX_COLOR_FILTER_NONE;
  init();

}

GX_STATUS MercureDriver::init()
{
    ROS_WARN("Mercure Camera Initializing......\n");
    uint32_t ui32DeviceNum = 0;
    
    status_ = GXInitLib(); 
    if(status_ != GX_STATUS_SUCCESS)
    {
      ROS_ERROR("Initialize Mercure Camera Driver libary failed. Error Code: %d", status_);
      return status_;
    }

    //step2 Get device enumerated number
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

    status_ = GXGetEnum(device_, GX_ENUM_PIXEL_COLOR_FILTER, &g_i64ColorFilter);
    GX_VERIFY_EXIT(status_);
    
    status_ = GXGetInt(device_, GX_INT_PAYLOAD_SIZE, &g_nPayloadSize);
    g_pRGBImageBuf = new unsigned char[g_nPayloadSize * 3]; 

    GX_VERIFY(status_);

    //Set acquisition mode
    status_ = GXSetEnum(device_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    GX_VERIFY_EXIT(status_);

    //Set trigger mode
    status_ = GXSetEnum(device_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GX_VERIFY_EXIT(status_);

    //Set buffer quantity of acquisition queue
    uint64_t nBufferNum = ACQ_BUFFER_NUM;
    status_ = GXSetAcqusitionBufferNumber(device_, nBufferNum);
    GX_VERIFY_EXIT(status_);

    bool bStreamTransferSize = false;
    status_ = GXIsImplemented(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
    GX_VERIFY_EXIT(status_);

    if(bStreamTransferSize)
    {
        //Set size of data transfer block
        status_ = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        GX_VERIFY_EXIT(status_);
    }

    bool bStreamTransferNumberUrb = false;
    status_ = GXIsImplemented(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
    GX_VERIFY_EXIT(status_);

    if(bStreamTransferNumberUrb)
    {
        //Set qty. of data transfer block
        status_ = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
        GX_VERIFY_EXIT(status_);
    }

    //Set Balance White Mode : Continuous
    status_ = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    //status_ = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    GX_VERIFY_EXIT(status_);
    
    int64_t i64Entry = GX_EXPOSURE_AUTO_OFF;
    status_ = GXGetEnum(device_, GX_ENUM_EXPOSURE_AUTO, &i64Entry);
   
    double dExposureTime = 10000;
    status_ = GXSetFloat(device_, GX_FLOAT_EXPOSURE_TIME, dExposureTime);

    //Allocate the memory for pixel format transform 
    //PreForAcquisition();
}

GX_STATUS MercureDriver::GetVision()
{
    ROS_WARN("<Libary Version : %s>", GXGetLibVersion());
    size_t nSize = 0;
    GXGetStringLength(device_, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
    GX_VERIFY_EXIT(status_);
    char *pszVendorName = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
    ROS_WARN("<Vendor Name : %s>", pszVendorName);
    delete[] pszVendorName;
    pszVendorName = NULL;

    GXGetStringLength(device_, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    GX_VERIFY_EXIT(status_);
    char *pszModelName = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
    ROS_WARN("<Model Name : %s>", pszModelName);
    delete[] pszModelName;
    pszModelName = NULL;

    GXGetStringLength(device_, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
    GX_VERIFY_EXIT(status_);
    char *pszSerialNumber = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
    ROS_WARN("<Serial Number : %s>", pszSerialNumber);
    delete[] pszSerialNumber;
    pszSerialNumber = NULL;

    status_ = GXGetStringLength(device_, GX_STRING_DEVICE_VERSION, &nSize);
    GX_VERIFY_EXIT(status_);
    char *pszDeviceVersion = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
    ROS_WARN("<Device Version : %s>", pszDeviceVersion);
    delete[] pszDeviceVersion;
    pszDeviceVersion = NULL;
}