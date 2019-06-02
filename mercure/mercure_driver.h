/*-------------------------------------------------------------
brief  :  Mercure Camera driver 
author :  XDU Robomaster, skystarry
date   :  2019.05.29
---------------------------------------------------------------*/

#pragma once

#include <ros/ros.h>
#include "DxImageProc.h"
#include "GxIAPI.h"
#include <opencv2/opencv.hpp>
#include <chrono>

//Show error message
#define GX_VERIFY(status_) ;

//Show error message, close device and lib
#define GX_VERIFY_EXIT(status_) ;

#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block
#define FILE_NAME_LEN           50   

#define PIXFMT_CVT_FAIL             -1             ///< PixelFormatConvert fail
#define PIXFMT_CVT_SUCCESS          0              ///< PixelFormatConvert success

class MercureDriver
{
  GX_STATUS     status_;
  GX_DEV_HANDLE device_;
  unsigned char* g_pRGBImageBuf; 
  int64_t g_i64ColorFilter;
  int64_t g_nPayloadSize;  
public:
  explicit MercureDriver();
  GX_STATUS init();
  GX_STATUS GetVision();

  void ReadCamera()
  {
    //Device start acquisition
    status_ = GXStreamOn(device_);

    if(status_ != GX_STATUS_SUCCESS)
    {
      //Release the memory allocated
        //Release resources
      if (g_pRGBImageBuf != NULL)
      {
          delete[] g_pRGBImageBuf;
          g_pRGBImageBuf = NULL;
      }
      GX_VERIFY_EXIT(status_);
    }

    ProcGetImage();

     //Device stop acquisition
    status_ = GXStreamOff(device_);
    if(status_ != GX_STATUS_SUCCESS)
    {
        //Release the memory allocated
        //UnPreForAcquisition();
        delete[] g_pRGBImageBuf;
        g_pRGBImageBuf = NULL;
        GX_VERIFY_EXIT(status_);
    }
  }

  //-------------------------------------------------
void ProcGetImage()
{
    GX_STATUS status_ = GX_STATUS_SUCCESS;

    PGX_FRAME_BUFFER pFrameBuffer = NULL;

    cv::Mat m_pImage;
    //m_pImage.create(pFrameBuffer->nHeight, pFrameBuffer->nWidth, CV_8UC3);
    m_pImage.create(1200, 1920, CV_8UC3);

    int i = 0;
    while(1)
    {
      auto speed_test_start_begin_time = std::chrono::steady_clock::now();
      // Get a frame from Queue
      status_ = GXDQBuf(device_, &pFrameBuffer, 1000);
      //printf("<Successful acquisition: FrameCount: %u Width: %d Height: %d FrameID: %llu>\n", 
       //   , pFrameBuffer->nWidth, pFrameBuffer->nHeight, pFrameBuffer->nFrameID);
      
      PixelFormatConvert(pFrameBuffer);

      memcpy(m_pImage.data, g_pRGBImageBuf, pFrameBuffer->nHeight*pFrameBuffer->nWidth*3);
      //cv::imshow("RGB", m_pImage);
      //cv::waitKey(1);

      status_ = GXQBuf(device_, pFrameBuffer);
      auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0;
      ROS_INFO("time cost = %.2f ms", cost);	
    }
    //return 0;
}
  int PixelFormatConvert(PGX_FRAME_BUFFER pFrameBuffer)
{
    GX_STATUS status_ = GX_STATUS_SUCCESS;
    VxInt32 emDXStatus = DX_OK;

    // Convert RAW8 or RAW16 image to RGB24 image
    // Convert to the RGB image
    emDXStatus = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, 
                        g_pRGBImageBuf, 
                        pFrameBuffer->nWidth,
                        pFrameBuffer->nHeight,
                        RAW2RGB_NEIGHBOUR, 
                        DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), 
                        false);

    if (emDXStatus != DX_OK)
    {
        printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
        return PIXFMT_CVT_FAIL;
    }
    return PIXFMT_CVT_SUCCESS;
}
  ~MercureDriver()
  {
    //Release the resources and stop acquisition thread
    //UnPreForAcquisition();
    delete[] g_pRGBImageBuf;
        g_pRGBImageBuf = NULL;

    //Close device
    status_ = GXCloseDevice(device_);
    if(status_ != GX_STATUS_SUCCESS)
    {
        //GetErrorString(status_);
        device_ = NULL;
        GXCloseLib();
        //return status_;
    }

    //Release libary
    status_ = GXCloseLib();
    if(status_ != GX_STATUS_SUCCESS)
    {
        //GetErrorString(status_);
        //return status_;
    }

    printf("<App exit!>\n");
  }
};