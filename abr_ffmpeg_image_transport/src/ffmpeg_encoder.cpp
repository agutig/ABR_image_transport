// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "abr_ffmpeg_image_transport/ffmpeg_encoder.hpp"

#ifdef USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <fstream>
#include <iomanip>
#include <string>

#include "abr_ffmpeg_image_transport/safe_param.hpp"
#include "abr_ffmpeg_image_transport/utils.hpp"

namespace abr_ffmpeg_image_transport
{

/*
  _____ _   _ _____ _______ 
 |_   _| \ | |_   _|__   __|
   | | |  \| | | |    | |   
   | | | . ` | | |    | |   
  _| |_| |\  |_| |_   | |   
 |_____|_| \_|_____|  |_|   
                          
*/

FFMPEGEncoder::FFMPEGEncoder() : logger_(rclcpp::get_logger("FFMPEGEncoder")) {}

FFMPEGEncoder::~FFMPEGEncoder()
{
  Lock lock(mutex_);
  closeCodec();
}

void FFMPEGEncoder::reset()
{
  Lock lock(mutex_);
  closeCodec();
}

static void free_frame(AVFrame ** frame)
{
  if (*frame) {
    av_free(*frame);
    *frame = nullptr;
  }
}

void FFMPEGEncoder::closeCodec() 
{
  if (!codecContext_) {
    return;  
  }


  if (codecName_ == "h264_nvenc"){
    avcodec_flush_buffers(codecContext_);
  }

  avcodec_close(codecContext_);

  if (codecContext_->hw_frames_ctx) {
    av_buffer_unref(&codecContext_->hw_frames_ctx);
  }

  avcodec_free_context(&codecContext_); 

  free_frame(&frame_);
  free_frame(&hw_frame_);
  free_frame(&wrapperFrame_);

  if (packet_) {
    av_packet_free(&packet_);
    packet_ = nullptr;
  }

  if (hwDeviceContext_) {
    av_buffer_unref(&hwDeviceContext_);
    hwDeviceContext_ = nullptr;
  }

  if (swsContext_) {
    sws_freeContext(swsContext_);
    swsContext_ = nullptr;
  }

  RCLCPP_INFO(logger_, "Encoder closed and resources freed.");
}


AVPixelFormat FFMPEGEncoder::pixelFormat(const std::string & f) const
{
  if (f.empty()) {
    return (AV_PIX_FMT_NONE);
  }
  const auto fmt = av_get_pix_fmt(f.c_str());
  if (fmt == AV_PIX_FMT_NONE) {
    RCLCPP_ERROR_STREAM(logger_, "ignoring unknown pixel format: " << f);
  }
  return (fmt);
}

void FFMPEGEncoder::setParameters(
    rclcpp::Node* node, 
    const nlohmann::json &json_config,
    int framerate,              
    int64_t selected_bitrate     
)
{
    Lock lock(mutex_);
    const std::string ns = "abr_ffmpeg_image_transport.";

    RCLCPP_INFO(logger_, "Initializing encoder parameters...");

    /* 1. Codec configuration */
    codecName_ = get_safe_param<std::string>(
                     node, 
                     ns + "encoding", 
                     json_config.value("codecName", std::string("libx264"))
                 );
    setCodec(codecName_);
    RCLCPP_INFO(node->get_logger(), "Codec set to: %s", codecName_.c_str());

    /* 2. Configure NVENC-specific settings o libx264 */
    if (codecName_ == "h264_nvenc" ) {

        std::string presetFromJson = json_config["h264_nvenc_options"].value("codec_preset", "p1");
        std::string tuneFromJson   = json_config["h264_nvenc_options"].value("codec_tune",   "ll");
        setPreset(get_safe_param<std::string>(node, ns + "preset", presetFromJson));
        setTune(get_safe_param<std::string>(node, ns + "tune",   tuneFromJson));

        RCLCPP_INFO(
            logger_,
            "Using NVENC codec. Applied parameters: preset = %s, tune = %s",
            preset_.c_str(), tune_.c_str()
        );
    } else {

        std::string presetFromJson = json_config["libx264_options"].value("codec_preset", "fast");
        std::string tuneFromJson   = "zerolatency";  
        setPreset(get_safe_param<std::string>(node, ns + "preset", presetFromJson));
        setTune(get_safe_param<std::string>(node, ns + "tune",   tuneFromJson));

        RCLCPP_INFO(
            logger_,
            "Using codec %s. Applied parameters: preset = %s, tune = %s",
            codecName_.c_str(), preset_.c_str(), tune_.c_str()
        );

        pixFormat_ = pixelFormat(
            get_safe_param<std::string>(node, ns + "pixel_format", "")
        );
    }

    /* 3. Configure GOP size */
    int64_t gopSizeFromJson = json_config.value("gop_size", static_cast<int64_t>(0));
    GOPSize_ = get_safe_param<int64_t>(node, ns + "gop_size", gopSizeFromJson);
  
    /* 4. Configure bitrate */

    bitRate_ = get_safe_param<int64_t>(node, ns + "bit_rate", selected_bitrate);


    /* 5. Configure frame rate */

    int usedFrameRate = get_safe_param<int>(node, ns + "frame_rate", framerate);
    setFrameRate(usedFrameRate, 1);
    RCLCPP_INFO(logger_, "Frame rate set to: %d FPS", usedFrameRate);

    /* 7. Reset encoder for the applied configuration */
    reset();
    RCLCPP_INFO(logger_, "Encoder reset completed with updated parameters.");
}

bool FFMPEGEncoder::initialize(int width, int height, Callback callback)
{
  Lock lock(mutex_);
  callback_ = callback;
  return (openCodec(width, height));
}

/*
  _    _          _____  _______          __     _____  ______ 
 | |  | |   /\   |  __ \|  __ \ \        / /\   |  __ \|  ____|
 | |__| |  /  \  | |__) | |  | \ \  /\  / /  \  | |__) | |__   
 |  __  | / /\ \ |  _  /| |  | |\ \/  \/ / /\ \ |  _  /|  __|  
 | |  | |/ ____ \| | \ \| |__| | \  /\  / ____ \| | \ \| |____ 
 |_|  |_/_/    \_\_|  \_\_____/   \/  \/_/    \_\_|  \_\______|
                                                               
*/


//Legacy , actually VAAPI doesnt support ABR
void FFMPEGEncoder::openVAAPIDevice(const AVCodec * codec, int width, int height)
{
  int err = 0;
  err = av_hwdevice_ctx_create(&hwDeviceContext_, AV_HWDEVICE_TYPE_VAAPI, NULL, NULL, 0);
  utils::check_for_err("cannot create hw device context", err);
  AVBufferRef * hw_frames_ref = av_hwframe_ctx_alloc(hwDeviceContext_);
  if (!hw_frames_ref) {
    throw std::runtime_error("cannot allocate hw device!");
  }

  AVHWFramesContext * frames_ctx = reinterpret_cast<AVHWFramesContext *>(hw_frames_ref->data);
  frames_ctx->format = utils::find_hw_config(&usesHardwareFrames_, AV_HWDEVICE_TYPE_VAAPI, codec);

  if (usesHardwareFrames_) {
    const auto fmts = utils::get_hwframe_transfer_formats(hw_frames_ref);
    frames_ctx->sw_format = utils::get_preferred_pixel_format("h264_vaapi", fmts);
    if (pixFormat_ != AV_PIX_FMT_NONE) {
      RCLCPP_INFO_STREAM(
        logger_, "user overriding software pix fmt " << utils::pix(frames_ctx->sw_format));
      RCLCPP_INFO_STREAM(logger_, "with " << utils::pix(pixFormat_));
      frames_ctx->sw_format = pixFormat_;  // override default at your own risk!
    } else {
      RCLCPP_INFO_STREAM(
        logger_, "using software pixel format: " << utils::pix(frames_ctx->sw_format));
    }
    if (frames_ctx->sw_format == AV_PIX_FMT_NONE) {
      av_buffer_unref(&hw_frames_ref);
      throw(std::runtime_error("cannot find valid sw pixel format!"));
    }
  }

  frames_ctx->width = width;
  frames_ctx->height = height;
  frames_ctx->initial_pool_size = 20;
  if ((err = av_hwframe_ctx_init(hw_frames_ref)) < 0) {
    av_buffer_unref(&hw_frames_ref);
    utils::throw_err("failed to initialize VAAPI frame context", err);
  }
  codecContext_->hw_frames_ctx = av_buffer_ref(hw_frames_ref);

  av_buffer_unref(&hw_frames_ref);

  if (codecContext_->hw_frames_ctx == nullptr) {
    throw(std::runtime_error("vaapi: cannot create buffer ref!"));
  }
}


//Prepares NVENC Hardware
void FFMPEGEncoder::openNVENCDevice(const AVCodec* codec, int width, int height)
{
  int err = 0;

  err = av_hwdevice_ctx_create(&hwDeviceContext_, AV_HWDEVICE_TYPE_CUDA, nullptr, nullptr, 0);
  if (err < 0) {
    utils::throw_err("Cannot create CUDA hw device context", err);
  }

  AVBufferRef* hw_frames_ref = av_hwframe_ctx_alloc(hwDeviceContext_);
  if (!hw_frames_ref) {
    throw std::runtime_error("Cannot allocate CUDA hw frame context!");
  }

  AVHWFramesContext* frames_ctx = reinterpret_cast<AVHWFramesContext*>(hw_frames_ref->data);

  frames_ctx->format = AV_PIX_FMT_CUDA;
  frames_ctx->sw_format = AV_PIX_FMT_NV12;

  if (pixFormat_ != AV_PIX_FMT_NONE) {
    RCLCPP_INFO_STREAM(logger_, "User overriding SW pixel format " 
                                 << utils::pix(frames_ctx->sw_format)
                                 << " with " << utils::pix(pixFormat_));
    frames_ctx->sw_format = pixFormat_;
  } else {
    RCLCPP_INFO_STREAM(logger_, "Using default SW pixel format: " 
                                 << utils::pix(frames_ctx->sw_format));
  }

  frames_ctx->width = width;
  frames_ctx->height = height;
  frames_ctx->initial_pool_size = 20;

  err = av_hwframe_ctx_init(hw_frames_ref);
  if (err < 0) {
    av_buffer_unref(&hw_frames_ref);
    utils::throw_err("Failed to initialize CUDA frame context", err);
  }

  codecContext_->hw_frames_ctx = av_buffer_ref(hw_frames_ref);
  av_buffer_unref(&hw_frames_ref);

  if (!codecContext_->hw_frames_ctx) {
    throw std::runtime_error("CUDA: cannot create buffer ref for hw_frames_ctx!");
  }

  codecContext_->pix_fmt = AV_PIX_FMT_CUDA;

  usesHardwareFrames_ = true;

  RCLCPP_INFO(logger_, "Initialized NVENC device context with CUDA successfully.");
}





/*
   _____ ____  _____  ______ _____ 
  / ____/ __ \|  __ \|  ____/ ____|
 | |   | |  | | |  | | |__ | |     
 | |   | |  | | |  | |  __|| |     
 | |___| |__| | |__| | |___| |____ 
  \_____\____/|_____/|______\_____|
                                   
                                   
*/



bool FFMPEGEncoder::openCodec(int width, int height)
{
  try {
    doOpenCodec(width, height);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, e.what());
    closeCodec();
    return (false);
  }
  RCLCPP_DEBUG_STREAM(
    logger_, "intialized codec " << codecName_ << " for image: " << width << "x" << height);
  return (true);
}

void FFMPEGEncoder::doOpenCodec(int width, int height)
{
  int err = 0;
  codecContext_ = nullptr;
  if (codecName_.empty()) {
    throw(std::runtime_error("no codec set!"));
  }
  if ((width % 32) != 0) {
    RCLCPP_WARN(logger_, "horiz res must be multiple of 32!");
  }
  if (codecName_ == "h264_nvmpi" && ((width % 64) != 0)) {
    RCLCPP_WARN(logger_, "horiz res must be multiple of 64!");
    throw(std::runtime_error("h264_nvmpi must have horiz rez mult of 64"));
  }
  // find codec
  const AVCodec * codec = avcodec_find_encoder_by_name(codecName_.c_str());
  if (!codec) {
    throw(std::runtime_error("cannot find encoder: " + codecName_));
  }

  auto pixFmts = utils::get_encoder_formats(codec);
  // allocate codec context
  codecContext_ = avcodec_alloc_context3(codec);
  if (!codecContext_) {
    throw(std::runtime_error("cannot allocate codec context!"));
  }

  /*
    ******************************************************************************************
   _____                                                     _       _           _  
 |  __ \                                                   (_)     | |         | | 
 | |__) __ _ _ __ __ _ _ __ ___  ___    __ _ ___  ___   ___ _  __ _| |_ ___  __| | 
 |  ___/ _` | '__/ _` | '_ ` _ \/ __|  / _` / __|/ _ \ / __| |/ _` | __/ _ \/ _` | 
 | |  | (_| | | | (_| | | | | | \__ \ | (_| \__ | (_) | (__| | (_| | ||  __| (_| | 
 |_|   \__,_|_|  \__,_|_| |_| |_|___/  \__,_|___/\___/ \___|_|\__,_|\__\___|\__,_| 
                                                                                   
                                                                                   
  _                        _            _____            _            _            
 | |                      | |          / ____|          | |          | |           
 | |_ ___     ___ ___   __| | ___  ___| |     ___  _ __ | |_ _____  _| |_          
 | __/ _ \   / __/ _ \ / _` |/ _ \/ __| |    / _ \| '_ \| __/ _ \ \/ | __|         
 | || (_) | | (_| (_) | (_| |  __| (__| |___| (_) | | | | ||  __/>  <| |_          
  \__\___/   \___\___/ \__,_|\___|\___|\_____\___/|_| |_|\__\___/_/\_\\__|         
                                                                       ______                                                                                                                                                  ______ 
  */


  codecContext_->width = width;
  codecContext_->height = height;
  codecContext_->time_base = timeBase_;
  codecContext_->framerate = frameRate_;
  codecContext_->gop_size = GOPSize_;
  codecContext_->max_b_frames = 0;  // nvenc can only handle zero!
    
  //bitRate_ = bitRate_ ;
  codecContext_->bit_rate = bitRate_;
  codecContext_->rc_max_rate = bitRate_*1.01;    // Upper fluctuation of bitrate allowed
  codecContext_->rc_min_rate = bitRate_ * 0.99; // Lower fluctuation of bitrate allowed
  codecContext_->rc_buffer_size = bitRate_;
  codecContext_->rc_initial_buffer_occupancy = bitRate_ * 0.9; // Init buffer at 90%


  codecContext_->qmin = 5;
  codecContext_->qmax = 50;

  av_opt_set(codecContext_->priv_data, "deblock", "1:1", 0);


  /*
  **********************************************************************************
  */


  if (codecName_.find("vaapi") != std::string::npos) {
    openVAAPIDevice(codec, width, height);
  } else if (codecName_.find("nvenc") != std::string::npos) {
    openNVENCDevice(codec, width, height);
  }


if (usesHardwareFrames_) {
    RCLCPP_INFO(
        logger_, 
        "Using hardware frames. Accessing AVHWFramesContext..."
    );
    
    AVHWFramesContext * frames_ctx =
      reinterpret_cast<AVHWFramesContext *>(codecContext_->hw_frames_ctx->data);

    codecContext_->sw_pix_fmt = frames_ctx->sw_format;
    codecContext_->pix_fmt = frames_ctx->format;

    RCLCPP_INFO(
        logger_,
        "Hardware frames context initialized: sw_pix_fmt = %s, pix_fmt = %s",
        utils::pix(codecContext_->sw_pix_fmt).c_str(),
        utils::pix(codecContext_->pix_fmt).c_str()
    );
} else {
    RCLCPP_INFO(
        logger_,
        "Not using hardware frames. Setting preferred pixel format..."
    );

    codecContext_->pix_fmt = utils::get_preferred_pixel_format(codecName_, pixFmts);
    codecContext_->sw_pix_fmt = codecContext_->pix_fmt;

    RCLCPP_INFO(
        logger_,
        "Software frames context initialized: pix_fmt = %s",
        utils::pix(codecContext_->pix_fmt).c_str()
    );
}

  /*
  ******************************************************************************************
 |  __ \                                                   (_)     | |         | | 
 | |__) __ _ _ __ __ _ _ __ ___  ___    __ _ ___  ___   ___ _  __ _| |_ ___  __| | 
 |  ___/ _` | '__/ _` | '_ ` _ \/ __|  / _` / __|/ _ \ / __| |/ _` | __/ _ \/ _` | 
 | |  | (_| | | | (_| | | | | | \__ \ | (_| \__ | (_) | (__| | (_| | ||  __| (_| | 
 |_|   \__,_|_|  \__,_|_| |_| |_|___/  \__,_|___/\___/ \___|_|\__,_|\__\___|\__,_| 
                                                                                   
                                                                                   
  _              __      ______        _   _                                       
 | |            /\ \    / / __ \      | | (_)                                      
 | |_ ___      /  \ \  / | |  | |_ __ | |_ _  ___  _ __                            
 | __/ _ \    / /\ \ \/ /| |  | | '_ \| __| |/ _ \| '_ \                           
 | || (_) |  / ____ \  / | |__| | |_) | |_| | (_) | | | |                          
  \__\___/  /_/    \_\/   \____/| .__/ \__|_|\___/|_| |_|                          
                                | |                                                
                                |_|                                                                                                                                          | |                                                                                                                                 |_|                      
  */

  setAVOption("preset", preset_);
  setAVOption("tune", tune_);

  /*
  ******************************************************************************************
  */


  err = avcodec_open2(codecContext_, codec, NULL);
  utils::check_for_err("cannot open codec", err);

  frame_ = av_frame_alloc();
  if (!frame_) {
    throw(std::runtime_error("cannot alloc software frame!"));
  }
  if (usesHardwareFrames_) {
    hw_frame_ = av_frame_alloc();
    if (!hw_frame_) {
      throw(std::runtime_error("cannot alloc hardware frame!"));
    }
  }
  frame_->width = width;
  frame_->height = height;
  frame_->format = codecContext_->sw_pix_fmt;
  // allocate image for frame
  err = av_image_alloc(
    frame_->data, frame_->linesize, width, height, static_cast<AVPixelFormat>(frame_->format), 64);
  utils::check_for_err("cannot alloc image", err);

  if (usesHardwareFrames_) {
    err = av_hwframe_get_buffer(codecContext_->hw_frames_ctx, hw_frame_, 0);
    utils::check_for_err("cannot get hw frame buffer", err);
    if (!hw_frame_->hw_frames_ctx) {
      throw(std::runtime_error("no hardware frame context, out of mem?"));
    }
  }

  // Initialize packet
  packet_ = av_packet_alloc();
  packet_->data = NULL;
  packet_->size = 0;

  // create (src) frame that wraps the received uncompressed image
  wrapperFrame_ = av_frame_alloc();
  wrapperFrame_->width = width;
  wrapperFrame_->height = height;
  wrapperFrame_->format = AV_PIX_FMT_BGR24;

  // initialize format conversion library
  if (!swsContext_) {
    swsContext_ = sws_getContext(
      width, height, AV_PIX_FMT_BGR24,                            // src
      width, height, static_cast<AVPixelFormat>(frame_->format),  // dest
      SWS_FAST_BILINEAR | SWS_ACCURATE_RND, NULL, NULL, NULL);
    if (!swsContext_) {
      throw(std::runtime_error("cannot allocate sws context"));
    }
  }
}

void FFMPEGEncoder::setAVOption(const std::string & field, const std::string & value)
{
  if (!value.empty()) {
    const int err =
      av_opt_set(codecContext_->priv_data, field.c_str(), value.c_str(), AV_OPT_SEARCH_CHILDREN);
    if (err != 0) {
      RCLCPP_ERROR_STREAM(
        logger_, "cannot set option " << field << " to value " << value << ": " << utils::err(err));
    }
  }
}

void FFMPEGEncoder::encodeImage(const Image & msg, int forced_width, int forced_height)
{
  rclcpp::Time t0;
  if (measurePerformance_) {
    t0 = rclcpp::Clock().now();
  }
  // TODO(Bernd): forcing the encoding to be BGR8 is wasteful when
  // the encoder supports monochrome.

  cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

  /*Redimention to fit the structure*/
  if (forced_width > 0 && forced_height > 0) {
    cv::resize(img, img, cv::Size(forced_width, forced_height), 0, 0, cv::INTER_AREA);
  }

  encodeImageMat(img, msg.header, t0);
  if (measurePerformance_) {
    const auto t1 = rclcpp::Clock().now();
    tdiffDebayer_.update((t1 - t0).seconds());
  }
}

void FFMPEGEncoder::encodeImageMat(const cv::Mat & img, const Header & header, const rclcpp::Time & t0)
{

  Lock lock(mutex_);
  rclcpp::Time t1, t2, t3;

  if (measurePerformance_) {
    frameCnt_++;
    t1 = rclcpp::Clock().now();
    totalInBytes_ += img.cols * img.rows;  // raw size!
  }

  // bend the memory pointers in colorFrame to the right locations
  av_image_fill_arrays(
    wrapperFrame_->data, wrapperFrame_->linesize, &(img.data[0]),
    static_cast<AVPixelFormat>(wrapperFrame_->format), wrapperFrame_->width, wrapperFrame_->height,
    1 /* alignment, could be better*/);


  sws_scale(
    swsContext_, wrapperFrame_->data, wrapperFrame_->linesize, 0,  // src
    codecContext_->height, frame_->data, frame_->linesize);        // dest

  if (measurePerformance_) {
    t2 = rclcpp::Clock().now();
    tdiffFrameCopy_.update((t2 - t1).seconds());
  }

  frame_->pts = pts_++;  //
  ptsToStamp_.insert(PTSMap::value_type(frame_->pts, header.stamp));

  int ret;
  if (usesHardwareFrames_) {
    ret = av_hwframe_transfer_data(hw_frame_, frame_, 0);  // from software -> hardware frame
    utils::check_for_err("error while copying frame to hw", ret);
    hw_frame_->pts = frame_->pts;
  }

  ret = avcodec_send_frame(codecContext_, usesHardwareFrames_ ? hw_frame_ : frame_);

  if (measurePerformance_) {
    t3 = rclcpp::Clock().now();
    tdiffSendFrame_.update((t3 - t2).seconds());
  }
  // now drain all packets
  while (ret == 0) {
    ret = drainPacket(header, img.cols, img.rows);
  }
  if (measurePerformance_) {
    const rclcpp::Time t4 = rclcpp::Clock().now();
    tdiffTotal_.update((t4 - t0).seconds());
  }
}

  /*
   _____                _         _    _ ___   __ _  _     __  __  _____  _____ 
  / ____|              | |       | |  | |__ \ / /| || |   |  \/  |/ ____|/ ____|
 | |     _ __ ___  __ _| |_ ___  | |__| |  ) / /_| || |_  | \  / | (___ | |  __ 
 | |    | '__/ _ \/ _` | __/ _ \ |  __  | / | '_ |__   _| | |\/| |\___ \| | |_ |
 | |____| | |  __| (_| | ||  __/ | |  | |/ /| (_) | | |   | |  | |____) | |__| |
  \_____|_|  \___|\__,_|\__\___| |_|  |_|____\___/  |_|   |_|  |_|_____/ \_____|
                                                                                                                                                                 
  */
int FFMPEGEncoder::drainPacket(const Header & header, int width, int height)
{
  Lock lock(drainMutex_);
  rclcpp::Time t0, t1, t2;
  if (measurePerformance_) {
    t0 = rclcpp::Clock().now();
  }
  int ret = avcodec_receive_packet(codecContext_, packet_);
  if (measurePerformance_) {
    t1 = rclcpp::Clock().now();
    tdiffReceivePacket_.update((t1 - t0).seconds());
  }
  const AVPacket & pk = *packet_;
  if (ret == 0 && pk.size > 0) {

    if (width != codecContext_->width || height != codecContext_->height) {
      RCLCPP_WARN(logger_, "Mismatch between provided dimensions and codec context dimensions. Provided: width=%d, height=%d; Codec context: width=%d, height=%d",
                  width, height, codecContext_->width, codecContext_->height);
      width = codecContext_->width;
      height = codecContext_->height;
    }

    FFMPEGPacket * packet = new FFMPEGPacket;
    FFMPEGPacketConstPtr pptr(packet);
    packet->data.resize(pk.size);
    packet->width = width;
    packet->height = height;
    packet->pts = pk.pts;
    packet->flags = pk.flags;
    memcpy(&(packet->data[0]), pk.data, pk.size);

    if (measurePerformance_) {
      t2 = rclcpp::Clock().now();
      totalOutBytes_ += pk.size;
      tdiffCopyOut_.update((t2 - t1).seconds());
    }

    packet->header = header;
    auto it = ptsToStamp_.find(pk.pts);
    if (it != ptsToStamp_.end()) {
      packet->header.stamp = rclcpp::Clock().now();
      packet->encoding = codecName_;

      callback_(pptr);  // deliver packet callback
      if (measurePerformance_) {
        const auto t3 = rclcpp::Clock().now();
        tdiffPublish_.update((t3 - t2).seconds());
      }
      ptsToStamp_.erase(it);
    } else {
      RCLCPP_ERROR_STREAM(logger_, "pts " << pk.pts << " has no time stamp!");
    }

    av_packet_unref(packet_);  // free packet allocated by encoder
  }
  return (ret);
}


/*
  _______ _____ __  __ ______ _____   _____ 
 |__   __|_   _|  \/  |  ____|  __ \ / ____|
    | |    | | | \  / | |__  | |__) | (___  
    | |    | | | |\/| |  __| |  _  / \___ \ 
    | |   _| |_| |  | | |____| | \ \ ____) |
    |_|  |_____|_|  |_|______|_|  \_\_____/ 
                                            
                                            
*/

void FFMPEGEncoder::printTimers(const std::string & prefix) const
{
  Lock lock(mutex_);
  RCLCPP_INFO_STREAM(
    logger_, prefix << " pktsz: " << totalOutBytes_ / frameCnt_ << " compr: "
                    << totalInBytes_ / (double)totalOutBytes_ << " debay: " << tdiffDebayer_
                    << " fmcp: " << tdiffFrameCopy_ << " send: " << tdiffSendFrame_
                    << " recv: " << tdiffReceivePacket_ << " cout: " << tdiffCopyOut_
                    << " publ: " << tdiffPublish_ << " tot: " << tdiffTotal_);
}
void FFMPEGEncoder::resetTimers()
{
  Lock lock(mutex_);
  tdiffDebayer_.reset();
  tdiffFrameCopy_.reset();
  tdiffSendFrame_.reset();
  tdiffReceivePacket_.reset();
  tdiffCopyOut_.reset();
  tdiffPublish_.reset();
  tdiffTotal_.reset();
  frameCnt_ = 0;
  totalOutBytes_ = 0;
  totalInBytes_ = 0;
}
}  // namespace abr_ffmpeg_image_transport
