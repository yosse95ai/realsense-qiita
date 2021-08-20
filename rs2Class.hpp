#pragma once
#include <iostream>
#include "librealsense2/rs.hpp"
#include <opencv2/opencv.hpp>

#define WIDTH 640
#define HEIGHT 480
#define FPS 30

typedef std::map<std::string, std::string> DeviceInfo;

/// <summary>
/// D455カメラ操作クラス
/// </summary>
class RsCamera
{
public:
	// コンストラクタ
	RsCamera();
	RsCamera(rs2::config cfg);

	// デストラクタ
	~RsCamera() { pipe.stop(); }


	float getCenterDistance();
	void getColorFrame(cv::Mat& color_image);
	void getDepthFrame(cv::Mat& depth_image);
	void getComboFrame(cv::Mat& combo_image, rs2::align align);
	DeviceInfo getDeviceInfo();
	DeviceInfo getDeviceInfoVer2();

private:
	bool isConnectedDevices();

	rs2::pipeline pipe;
	rs2::frameset frames;
	rs2::colorizer color_map;
	rs2::context ctx;
	rs2::device_list devices;
};

/*********************************************************
*
* Public functions
*
**********************************************************/
/// <summary>
/// コンストラクタ
/// </summary>
RsCamera::RsCamera()
{
	if (isConnectedDevices()) {
		pipe.start();
	}
	else {
		throw std::runtime_error("ERR: No cameras are connected.");
	}
}

/// <summary>
/// コンストラクタ
/// </summary>
/// <param name="cfg">設定情報</param>
RsCamera::RsCamera(rs2::config cfg)
{
	if (isConnectedDevices()) {
		pipe.start(cfg);
	}
	else {
		throw std::runtime_error("ERR: No cameras are connected.");
	}
}

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
/// <param name="color_image">フレーム出力先</param>
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
/// <param name="depth_image">フレーム出力先</param>
void RsCamera::getDepthFrame(cv::Mat& depth_image)
{
	rs2::frameset frames = pipe.wait_for_frames();
	rs2::video_frame depth_frame = frames.get_depth_frame().apply_filter(color_map);

	cv::Mat image(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

	image.copyTo(depth_image);
}

/// <summary>
/// カラーフレームとDepthフレームを取得する
/// </summary>
/// <param name="combo_image">フレーム出力先</param>
/// <param name="align">アラインメントの設定</param>
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

/// <summary>
/// デバイスの情報を連想配列で取得する
/// </summary>
/// <returns>デバイスの情報の連想配列</returns>
DeviceInfo RsCamera::getDeviceInfo()
{
	rs2::device device = devices[0];
	DeviceInfo dev_info;

	dev_info["NAME"] = device.get_info(RS2_CAMERA_INFO_NAME);
	dev_info["ADVANCED_MODE"] = device.get_info(RS2_CAMERA_INFO_ADVANCED_MODE);
	dev_info["ASIC_SERIAL_NUMBER"] = device.get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER);
	dev_info["CAMERA_LOCKED"] = device.get_info(RS2_CAMERA_INFO_CAMERA_LOCKED);
	dev_info["DEBUG_OP_CODE"] = device.get_info(RS2_CAMERA_INFO_DEBUG_OP_CODE);
	dev_info["FIRMWARE_UPDATE_ID"] = device.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID);
	dev_info["FIRMWARE_VERSION"] = device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
	dev_info["PRODUCT_LINE"] = device.get_info(RS2_CAMERA_INFO_PRODUCT_LINE);
	dev_info["PRODUCT_ID"] = device.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
	dev_info["PHYSICAL_PORT"] = device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
	dev_info["RECOMMENDED_FIRMWARE_VERSION"] = device.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION);
	dev_info["SERIAL_NUMBER"] = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	dev_info["USB_TYPE_DESCRIPTOR"] = device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);

	return dev_info;
}

/// <summary>
/// RS2_CAMERA_INFO_COUNTを使用した例
/// </summary>
/// <returns>デバイスの情報の連想配列</returns>
DeviceInfo RsCamera::getDeviceInfoVer2()
{
	rs2::device device = devices[0];
	DeviceInfo dev_info;
	for (int i = 0; i < RS2_CAMERA_INFO_COUNT; ++i) {
		rs2_camera_info info = (rs2_camera_info)i;

		if (device.supports(info)) {
			auto info_key = rs2_camera_info_to_string(info);
			dev_info[info_key] = device.get_info(info);
		}
	}
	return dev_info;
}

/*********************************************************
*
* Private functions
*
**********************************************************/
/// <summary>
/// 接続デバイス情報を更新
/// </summary>
/// <returns>T:接続あり，F:接続無し</returns>
bool RsCamera::isConnectedDevices() {
	devices = ctx.query_devices();
	if (!devices.size()) { 
		return false;
	}
	return true;
}