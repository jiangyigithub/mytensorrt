#include "ffmpeg_decoder_bridge_node.hpp"
using std::placeholders::_1;


namespace ffmpeg_decoder_bridge
{

FFmpegDecoderBridgeNode::FFmpegDecoderBridgeNode()
	: Node("h264_decoder_bridge")
	, m_imgConvertContext(nullptr)
{
	avcodec_register_all();
	auto codec = avcodec_find_decoder(AV_CODEC_ID_H264);
	if (!codec) {
		std::cout<<"Codec not found\n";
		return;
	}

	m_context = AvCodecContextPtr(avcodec_alloc_context3(codec), [&] (AVCodecContext* c) { avcodec_close(c); av_free(c);});

	if (avcodec_open2(m_context.get(), codec, NULL) < 0) {
		std::cout<<"Could not open codec\n";
		return;
	}
    m_subscribeCompressedImage = this->create_subscription<sensor_msgs::msg::CompressedImage>(/*"/perception/camera/camera_01_01/image"*/
    "/input/video", 100, std::bind(&ffmpeg_decoder_bridge::FFmpegDecoderBridgeNode::callbackEncodedImage, this, _1));
    // m_subscribeCompressedImage = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    // "/image_raw", 10, std::bind(&ffmpeg_decoder_bridge::FFmpegDecoderBridgeNode::callbackEncodedImage, this, _1));
	
	m_publishDecodedImage = this->create_publisher<sensor_msgs::msg::Image>("/output/video", 100);
}

void FFmpegDecoderBridgeNode::callbackEncodedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
	RCLCPP_INFO(this->get_logger(), "Msg");
	AVPacket packet;
	av_init_packet(&packet);

	if(msg->data.size() == 0){
		RCLCPP_INFO(this->get_logger(), "Bad Image! Size: " + std::to_string(msg->data.size()));
		return;
	}

	packet.data = msg->data.data();
	packet.size = msg->data.size();

	RCLCPP_INFO(this->get_logger(), "Packet Size: " + std::to_string(packet.size));

	auto yCbCrFrame = std::move(AVFramePtr(av_frame_alloc(), [](AVFrame* f) { av_frame_free(&f); }));
	int got_picture = 0;
	int ret = avcodec_decode_video2(m_context.get(), yCbCrFrame.get(), &got_picture, &packet);
	if (ret < 0) {
		RCLCPP_INFO(this->get_logger(), "Decode Error. " + std::to_string(ret));
		return;
	}
	if (got_picture) {
		RCLCPP_INFO(this->get_logger(), "Picture loaded" );
	}

	auto rgbFrame = std::move(AVFramePtr(av_frame_alloc(), [](AVFrame* f) { av_frame_free(&f); }));
	int numBytes = avpicture_get_size(AV_PIX_FMT_BGR24, m_context->width, m_context->height);
	std::vector<uint8_t> rgb;
	rgb.resize(numBytes);
	// int linesize[1];
	// linesize[0]=int(rgb.size;
	// for(auto test:rgbFrame)
	// {
	// 	std::cout<<"00_rgb"<<test<<std::endl;
	// }
	cv::Mat image(m_context->height, m_context->width, CV_8UC3);
	int linesize[1];
	linesize[0]=image.step1();

	m_imgConvertContext = sws_getCachedContext(m_imgConvertContext,
										m_context->width, m_context->height, m_context->pix_fmt,
										m_context->width, m_context->height, AV_PIX_FMT_BGR24,
										SWS_X, nullptr, nullptr, nullptr);
	if (!m_imgConvertContext) {

		std::cout << "can't init convert context" << std::endl;
		return;
	}
	// int scaleRet = sws_scale(m_imgConvertContext, yCbCrFrame->data, yCbCrFrame->linesize,
	// 						 0, m_context->height, rgbFrame->data, rgbFrame->linesize);
	
	int scaleRet = sws_scale(m_imgConvertContext, yCbCrFrame->data, yCbCrFrame->linesize,
							 0, m_context->height, &image.data, linesize);
	cv::Mat test; 
	image.convertTo(test,CV_32F);
	// auto rgb1 = rgb;
	// rgb1 = test;
	rgb = (std::vector<uint8_t>)(test.reshape(1,1));


	std::cout<<"01:"<<scaleRet<<std::endl;
	std::cout<<"02:"<<yCbCrFrame->data<<std::endl;
	std::cout<<"03:"<<typeid(rgbFrame->data[0]).name()<<std::endl;
	std::cout<<"03.1:"<<typeid(rgbFrame->data[1]).name()<<std::endl;
	std::cout<<"03.1:"<<typeid(rgbFrame->data[2]).name()<<std::endl;
	std::cout<<"04:"<<test.at<int>(1,2)<<std::endl;

	if (scaleRet < 0) {
		std::cout << "cannot convert color space" << std::endl;
		return;
	}

	// cv::Mat image(m_context->height,m_context->width,CV_8UC3,rgbFrame->data,rgbFrame->linesize);
	// std::cout<<"05000:"<<image<<std::endl;
	// rgb = image.data();

/*	std::ofstream ofs("/tmp/test.rgb", std::ios::binary); //ffplay -f rawvideo -pixel_format bgr24 -video_size 1920x1080 test.rgb
	ofs.write((char*)rgb.data(), rgb.size());

	std::ofstream ofs1("/tmp/test.yuv", std::ios::binary); //ffplay -f rawvideo -pixel_format yuv420p -video_size 1920x1080 test.yuv
	ofs1.write((char*)yCbCrFrame->data[0], m_context->width * m_context->height);
	ofs1.write((char*)yCbCrFrame->data[1], m_context->width * m_context->height / 4);
	ofs1.write((char*)yCbCrFrame->data[2], m_context->width * m_context->height / 4);
	int x = 0;
*/
	sensor_msgs::msg::Image decoded_image;
	decoded_image.height = m_context->height;
	decoded_image.width = m_context->width;
	decoded_image.header.frame_id = msg->header.frame_id;
	decoded_image.header.stamp = msg->header.stamp;
	decoded_image.encoding = "bgr8";
	decoded_image.data =  std::move(rgb);  //used to be rgb
	decoded_image.step = m_context->width*3;
	m_publishDecodedImage->publish(decoded_image);
}

}  // namespace ffmpeg_decoder_bridge
