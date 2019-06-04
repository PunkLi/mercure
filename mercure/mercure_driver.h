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
#include <ros/package.h>

#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB  64             ///< Qty. of data transfer block

namespace camera{

class MercureDriver
{
  GX_STATUS     status_;
  GX_DEV_HANDLE device_;

  uint8_t* rgbImagebuf_; 

  int64_t colorfilter_;
  int64_t payloadsize_;  

  uint64_t bufferNum_;

  cv::Mat Image_;

  PGX_FRAME_BUFFER pFrameBuffer_;
  
  int exp_auto_;
  double exp_time_; // us

  int w_auto_;
  double w_red_;
  double w_green_;
  double w_blue_;
  
  int gain_auto_;
  double gain_;

public:
  explicit MercureDriver();
  GX_STATUS init_sdk();
  void GetVision();

  void ReadCamera();
  void LoadParam();
  ~MercureDriver();


};

} // namespace camera