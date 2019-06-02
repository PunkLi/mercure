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

#define GX_VERIFY(status_) ;
#define GX_VERIFY_EXIT(status_) ;

#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block
#define FILE_NAME_LEN           50   

class MercureDriver
{
  GX_STATUS     status_;
  GX_DEV_HANDLE device_;

  uint8_t*      rgbImagebuf_; 

  int64_t colorfilter_;
  int64_t payloadsize_;  

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
      if (rgbImagebuf_ != NULL)
      {
          delete[] rgbImagebuf_;
          rgbImagebuf_ = NULL;
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
        delete[] rgbImagebuf_;
        rgbImagebuf_ = NULL;
        GX_VERIFY_EXIT(status_);
    }
  }

  //-------------------------------------------------
void ProcGetImage()
{
    
    PGX_FRAME_BUFFER pFrameBuffer = NULL;

    cv::Mat m_pImage;
    //m_pImage.create(pFrameBuffer->nHeight, pFrameBuffer->nWidth, CV_8UC3);
    m_pImage.create(1200, 1920, CV_8UC3);

    while(1)
    {
      auto speed_test_start_begin_time = std::chrono::steady_clock::now();

      status_ = GXDQBuf(device_, &pFrameBuffer, 1000);

      DxRaw8toRGB24((uint8_t*)pFrameBuffer->pImgBuf, 
                        rgbImagebuf_, 
                        pFrameBuffer->nWidth,
                        pFrameBuffer->nHeight,
                        RAW2RGB_NEIGHBOUR, 
                        DX_PIXEL_COLOR_FILTER(colorfilter_), 
                        false);

      memcpy(m_pImage.data, rgbImagebuf_, pFrameBuffer->nHeight*pFrameBuffer->nWidth*3);
      //cv::imshow("RGB", m_pImage);
      //cv::waitKey(1);

      status_ = GXQBuf(device_, pFrameBuffer);

      auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0;
      ROS_INFO("time cost = %.2f ms", cost);	
    }
    //return 0;
}
  ~MercureDriver();
};