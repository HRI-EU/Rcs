/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "VideoRecorder.h"

#include <Rcs_macros.h>
#include <Rcs_utils.h>

#if defined (USE_FFMPEG)

#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osgViewer/Viewer>
#include <osg/Texture2D>
#include <osg/Camera>
#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}

#include <sstream>
#include <cstring>

constexpr int DefaultBitrate = 5000000;
constexpr int DefaultAlignment = 32;
constexpr int DefaultImageAlignment = 32;
constexpr int DefaultGOPSize = 10;

static int imgAlign(int size)
{
  return (size / DefaultImageAlignment) * DefaultImageAlignment;
}

static std::string avErrorToString(int errnum)
{
  char errbuf[AV_ERROR_MAX_STRING_SIZE];  // FFmpeg defines a standard size for error strings
  if (av_strerror(errnum, errbuf, sizeof(errbuf)) < 0)
  {
    std::ostringstream oss;
    oss << "Unknown error code: " << errnum;
    return oss.str();
  }
  return std::string(errbuf) + " (errnum=" + std::to_string(errnum) + ")";
}

namespace Rcs
{

class VideoRecorder
{
public:
  VideoRecorder(const std::string& filename, int width_, int height_, int fps_)
    : width(imgAlign(width_)), height(imgAlign(height_)), fps(fps_), stopRecording(false),
      verbose(false)
  {
    RLOG(0, "Creating VideoRecorder with width=%d height=%d fps=%d", width, height, fps);

    bool highQuality = false;

    // Set the FFmpeg log level to suppress informational messages
    av_log_set_level(AV_LOG_WARNING);

    // Register all formats and codecs
    avformat_alloc_output_context2(&formatContext, NULL, NULL, filename.c_str());
    if (!formatContext)
    {
      RLOG_CPP(0, "Could not allocate format context");
      return;
    }

    // Find the encoder
    codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec)
    {
      RLOG_CPP(0, "Codec not found");
      return;
    }

    // Create codec context
    codecContext = avcodec_alloc_context3(codec);
    if (!codecContext)
    {
      RLOG_CPP(0, "Could not allocate video codec context");
      return;
    }

    AVRational timeBase = { 1, fps };
    AVRational framerate = { fps, 1 };
    codecContext->width = width;
    codecContext->height = height;
    codecContext->time_base = timeBase;
    codecContext->framerate = framerate;
    codecContext->pix_fmt = AV_PIX_FMT_YUV420P;
    codecContext->bit_rate = DefaultBitrate; // Set a higher bitrate (5 Mbps as an example), was 400000;
    codecContext->gop_size = DefaultGOPSize; // emit one intra frame every ten frames
    codecContext->max_b_frames = 1;

    if (highQuality)
    {
      codecContext->bit_rate = 5000000; // Set a higher bitrate (5 Mbps as an example), was 400000;
      codecContext->gop_size = 12; // emit one intra frame every ten frames was 10
      codecContext->max_b_frames = 2; // was 1

      // Use High profile
      av_opt_set(codecContext->priv_data, "profile", "high", 0);

      // Set CRF value to control quality (lower is better quality)
      av_opt_set(codecContext->priv_data, "crf", "20", 0);

      // Tune for film content
      av_opt_set(codecContext->priv_data, "tune", "film", 0);

      // Use a slower preset for better compression (quality)
      av_opt_set(codecContext->priv_data, "preset", "slow", 0);
    }

    // Open the codec
    int res = avcodec_open2(codecContext, codec, NULL);
    if (res < 0)
    {
      RLOG_CPP(0, "Could not open codec: " << avErrorToString(res));
      return;
    }

    // Allocate video stream
    videoStream = avformat_new_stream(formatContext, codec);
    if (!videoStream)
    {
      RLOG_CPP(0, "Could not allocate stream");
      return;
    }
    videoStream->id = formatContext->nb_streams - 1;
    videoStream->time_base = timeBase;

    res = avcodec_parameters_from_context(videoStream->codecpar, codecContext);
    if (res < 0)
    {
      RLOG_CPP(0, "Error in avcodec_parameters_from_context: " << avErrorToString(res));
      return;
    }

    // Open the output file
    if (!(formatContext->oformat->flags & AVFMT_NOFILE))
    {
      if (avio_open(&formatContext->pb, filename.c_str(), AVIO_FLAG_WRITE) < 0)
      {
        RLOG_CPP(0, "Could not open output file" << filename);
        return;
      }
    }

    // Write the file header
    res = avformat_write_header(formatContext, NULL);
    if (res < 0)
    {
      RLOG_CPP(0, "Error occurred when opening output file" << avErrorToString(res));
      return;
    }

    // Allocate frame and buffer
    frame = av_frame_alloc();
    frame->format = codecContext->pix_fmt;
    frame->width = codecContext->width;
    frame->height = codecContext->height;

    res = av_image_alloc(frame->data, frame->linesize, codecContext->width, codecContext->height, codecContext->pix_fmt, DefaultAlignment);
    if (res < 0)
    {
      RLOG_CPP(0, "Could not allocate raw picture buffer" << avErrorToString(res));
      return;
    }

    // Allocate RGB frame for input
    frameRGB = av_frame_alloc();
    int numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGB24, width, height, DefaultAlignment);
    buffer = (uint8_t*)av_malloc(numBytes * sizeof(uint8_t));
    res = av_image_fill_arrays(frameRGB->data, frameRGB->linesize, buffer, AV_PIX_FMT_RGB24, width, height, DefaultAlignment);
    if (res < 0)
    {
      RLOG_CPP(0, "Error in av_image_fill_arrays: " << avErrorToString(res));
      return;
    }

    // Initialize SWS context for conversion
    swsContext = sws_getContext(width, height, AV_PIX_FMT_RGB24, width, height, AV_PIX_FMT_YUV420P, SWS_BILINEAR, NULL, NULL, NULL);

    if (!swsContext)
    {
      RLOG_CPP(0, "Could not initialize the conversion context");
      return;
    }

    // Start the encoding thread
    RLOG(0, "Starting encoding thread");
    encodingThread = std::thread(&VideoRecorder::encodingLoop, this);
  }

  ~VideoRecorder()
  {
    // This enforces showing the queue size after destruction, since this
    // might lead to delays before finally destructing the class.
    verbose = true;

    {
      std::unique_lock<std::mutex> lock(mutex);
      stopRecording = true;
      condition.notify_all();
    }
    encodingThread.join();

    // Write the trailer
    av_write_trailer(formatContext);

    // Free the YUV frame
    av_frame_free(&frame);
    av_frame_free(&frameRGB);
    av_free(buffer);

    // Close the codec
    avcodec_free_context(&codecContext);

    // Close the output file
    if (!(formatContext->oformat->flags & AVFMT_NOFILE))
    {
      avio_close(formatContext->pb);
    }

    // Free the format context
    avformat_free_context(formatContext);

    RLOG_CPP(0, "VideoRecorder says good bye");
  }

  void captureFrame(osg::ref_ptr<osg::Image> osgImage)//const osg::Image* osgImage)
  {
    if (osgImage->getPixelFormat() != GL_RGB || osgImage->getDataType() != GL_UNSIGNED_BYTE)
    {
      RLOG_CPP(0, "Unsupported image format in captureFrame");
      return;
    }

    // Create a copy of the osgImage data
    std::unique_lock<std::mutex> lock(mutex);
    frameQueue.push(osgImage);
    condition.notify_all();
  }

private:
  int width, height, fps, frameCount = 0;
  AVFormatContext* formatContext = nullptr;
  AVCodecContext* codecContext = nullptr;
  const AVCodec* codec = nullptr;
  AVStream* videoStream = nullptr;
  AVFrame* frame = nullptr;
  AVFrame* frameRGB = nullptr;
  uint8_t* buffer = nullptr;
  struct SwsContext* swsContext = nullptr;

  std::queue<osg::ref_ptr<osg::Image>> frameQueue;
  std::thread encodingThread;
  std::mutex mutex;
  std::condition_variable condition;
  bool stopRecording;
  bool verbose;

  void encodingLoop()
  {
    while (true)
    {
      // Blocks the current thread until there is a frame available in the
      // queue or the recording process is signaled to stop.
      std::unique_lock<std::mutex> lock(mutex);
      condition.wait(lock, [this] { return !frameQueue.empty() || stopRecording; });

      // We only quit after the last frame has been encoded. This might delay the
      // shutdown a bit in case several frames have queued up.
      if (stopRecording && frameQueue.empty())
      {
        break;
      }

      osg::ref_ptr<osg::Image> osgImage = frameQueue.front();
      frameQueue.pop();
      lock.unlock();

      if (verbose || frameQueue.size()>10)
      {
        RLOG_CPP(0, "FFMPEG frame queue is " << frameQueue.size());
      }

      // Here we scale the image to the proper alignment and flip it
      // vertically so that it matches the ffmpeg conventions.
      osgImage->scaleImage(width, height, osgImage->r());
      osgImage->flipVertical();

      // Convert OSG image to FFmpeg frame
      std::memcpy(frameRGB->data[0], osgImage->data(), width * height * 3);

      // Convert RGB to YUV
      sws_scale(swsContext, frameRGB->data, frameRGB->linesize, 0, height, frame->data, frame->linesize);

      frame->pts = frameCount++;

      // Encode the image
      int res = encodeFrame(frame);
      if (res < 0)
      {
        RLOG_CPP(0, "Error during encoding - quitting encodingLoop: " << avErrorToString(res));
        break;
      }

    }

    // Flush the encoder after the loop
    encodeFrame(nullptr);
  }


  int encodeFrame(AVFrame* frame)
  {
    // Encode the image or send a null frame to flush the encoder
    int ret = avcodec_send_frame(codecContext, frame);
    if (ret < 0)
    {
      RLOG_CPP(0, "Error sending frame for encoding: " << avErrorToString(ret));
      return ret;
    }

    while (ret >= 0)
    {
      AVPacket pkt;
      av_init_packet(&pkt);
      pkt.data = nullptr;
      pkt.size = 0;

      ret = avcodec_receive_packet(codecContext, &pkt);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
      {
        // EAGAIN means the encoder needs more input frames to produce
        // output, and we succeeded.
        // RLOG_CPP(0, "No more packets are available: " << avErrorToString(ret));
        ret = 0;
        break;  // No more packets are available
      }
      else if (ret < 0)
      {
        RLOG_CPP(0, "Error during flushing: " << avErrorToString(ret));
        break;
      }

      // Rescale the PTS and DTS to the stream time_base
      av_packet_rescale_ts(&pkt, codecContext->time_base, videoStream->time_base);

      // Ensure that DTS is always <= PTS
      if (pkt.dts > pkt.pts)
      {
        pkt.dts = pkt.pts;
      }

      // Write the final packets to the video file
      ret = av_interleaved_write_frame(formatContext, &pkt);
      if (ret < 0)
      {
        RLOG_CPP(0, "Error writing flushed frame: " << avErrorToString(ret));
        break;
      }

      av_packet_unref(&pkt);  // Free the packet after use
    }

    return ret;
  }



};



FrameCaptureCallback::FrameCaptureCallback() : recorder(nullptr)
{
}

FrameCaptureCallback::~FrameCaptureCallback()
{
  deleteRecorder();
}

void FrameCaptureCallback::createRecorder(int width, int height, int fps)
{
  if (!recorder)
  {
    char fileName[512] = "video.mp4";
    File_createUniqueName(fileName, "video", "mp4");
    recorder = new VideoRecorder(fileName, width, height, fps);
  }
}

void FrameCaptureCallback::deleteRecorder()
{
  delete recorder;
  recorder = nullptr;
}

bool FrameCaptureCallback::isRecording() const
{
  return recorder ? true : false;
}

void FrameCaptureCallback::operator()(osg::RenderInfo& renderInfo) const
{
  // Here we keep the native resolution. It is not aligned to 16 bytes.
  // We take care of it later in the encoding thread.
  osg::Camera* camera = renderInfo.getCurrentCamera();
  osg::ref_ptr<osg::Image> image = new osg::Image;
  image->readPixels(0, 0, camera->getViewport()->width(),
                    camera->getViewport()->height(),
                    GL_RGB, GL_UNSIGNED_BYTE);
  recorder->captureFrame(image);
}

bool FrameCaptureCallback::hasRecorder()
{
  return true;
}

}   // namespace Rcs

#else   // not USE_FFMPEG

namespace Rcs
{

FrameCaptureCallback::FrameCaptureCallback()
{
}

FrameCaptureCallback::~FrameCaptureCallback()
{
}

void FrameCaptureCallback::createRecorder(int width, int height, int fps)
{
}

void FrameCaptureCallback::deleteRecorder()
{
}

bool FrameCaptureCallback::isRecording() const
{
  return false;
}

void FrameCaptureCallback::operator()(osg::RenderInfo& renderInfo) const
{
}

bool FrameCaptureCallback::hasRecorder()
{
  return false;
}

}   // namespace Rcs

#endif   // USE_FFMPEG
