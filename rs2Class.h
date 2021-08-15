#pragma once
#include "librealsense2/rs.hpp"
#include <opencv2/opencv.hpp>

#define WIDTH 640
#define HEIGHT 480
#define FPS 30

/// <summary>
/// D455カメラ操作クラス
/// </summary>
class RsCamera
{
public:
	// コンストラクタ
	RsCamera() { pipe.start(); }
	RsCamera(rs2::config cfg) { pipe.start(cfg); }

	// デストラクタ
	~RsCamera(){ pipe.stop(); }

	float getCenterDistance();
	void getColorFrame(cv::Mat& color_image);
	void getDepthFrame(cv::Mat& depth_image);
	void getComboFrame(cv::Mat& combo_image, rs2::align align);

private:
	rs2::pipeline pipe;
	rs2::frameset frames;
	rs2::colorizer color_map;
};

/// <summary>
/// 画面中央の深度を取得
/// </summary>
/// <returns>深度の値[m]</returns>
float RsCamera::getCenterDistance()
{
	rs2::frameset frames = pipe.wait_for_frames();

	rs2::depth_frame depth = frames.get_depth_frame();

	auto width = depth.get_width();
	auto height = depth.get_height();

	float distance_to_center = depth.get_distance(width / 2, height / 2);

	return distance_to_center;
}

/// <summary>
/// カラーフレームを取得する
/// </summary>
/// <param name="color_image">cv::Mat</param>
void RsCamera::getColorFrame(cv::Mat& color_image) 
{
	rs2::frameset frames = pipe.wait_for_frames();
	rs2::video_frame color_frame = frames.get_color_frame();

	cv::Mat image(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

	image.copyTo(color_image);
}

/// <summary>
/// 深度フレームを取得する
/// </summary>
/// <param name="depth_image">cv::Mat</param>
void RsCamera::getDepthFrame(cv::Mat& depth_image)
{
	rs2::frameset frames = pipe.wait_for_frames();
	rs2::video_frame depth_frame = frames.get_depth_frame().apply_filter(color_map);

	cv::Mat image(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

	image.copyTo(depth_image);
}

void RsCamera::getComboFrame(cv::Mat& combo_image, rs2::align align)
{
	rs2::frameset frames = pipe.wait_for_frames();
	auto aligned_frames = align.process(frames);

	rs2::video_frame color_frame = aligned_frames.get_color_frame();
	rs2::video_frame depth_frame = aligned_frames.get_depth_frame().apply_filter(color_map);

	cv::Mat color_image(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
	cv::Mat depth_image(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

	cv::Mat color_positon(combo_image, cv::Rect(0, 0, WIDTH, HEIGHT));
	color_image.copyTo(color_positon);
	cv::Mat depth_positon(combo_image, cv::Rect(WIDTH, 0, WIDTH, HEIGHT));
	depth_image.copyTo(depth_positon);
}