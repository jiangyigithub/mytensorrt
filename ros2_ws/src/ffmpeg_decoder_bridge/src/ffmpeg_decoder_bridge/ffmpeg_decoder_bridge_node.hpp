#pragma once

#include <memory>
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace ffmpeg_decoder_bridge
{

using AvCodecContextPtr = std::unique_ptr<AVCodecContext, std::function<void (AVCodecContext*)>>;
using AVFramePtr = std::unique_ptr<AVFrame, std::function<void (AVFrame*)>> ;

class FFmpegDecoderBridgeNode : public rclcpp::Node
{
public:
	FFmpegDecoderBridgeNode();
	void spin();

private:
	void callbackEncodedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
	rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr m_subscribeCompressedImage;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publishDecodedImage;
	AvCodecContextPtr m_context;
	SwsContext* m_imgConvertContext;
};

}  // namespace ffmpeg_decoder_bridge
