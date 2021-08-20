﻿#pragma once
#include <iostream>
#include "rs2Class.h"

/// <summary>
/// デバイス情報を表示する
/// </summary>
/// <param name="info">デバイスの情報</param>
void showDeviceInfo(RsCamera camera)
{
	DeviceInfo info = camera.getDeviceInfo();
	std::cout
		<< "** Device info **" << std::endl
		<< "  NAME         : " << info["NAME"] << std::endl
		<< "  PRODUCT LINE : " << info["PRODUCT_LINE"] << std::endl
		<< "  PRODUCT ID   : " << info["PRODUCT_ID"] << std::endl
		<< "  SERIAL NUMBER       : " << info["SERIAL_NUMBER"] << std::endl
		<< "  ASIC SERIAL NUMBER  : " << info["ASIC_SERIAL_NUMBER"] << std::endl
		<< "  FIRMWARE UPDATE ID  : " << info["FIRMWARE_UPDATE_ID"] << std::endl
		<< "  FIRMWARE_VERSION    : " << info["FIRMWARE_VERSION"] << std::endl
		<< "  ADVANCED MODE       : " << info["ADVANCED_MODE"] << std::endl
		<< "  CAMERA LOCKED       : " << info["CAMERA_LOCKED"] << std::endl
		<< "  DEBUG OP CODE       : " << info["DEBUG_OP_CODE"] << std::endl
		<< "  USB TYPE DESCRIPTOR : " << info["USB_TYPE_DESCRIPTOR"] << std::endl
		<< "  RECOMMENDED FIRMWARE VERSION : " << info["RECOMMENDED_FIRMWARE_VERSION"] << std::endl
		<< "  PHYSICAL_PORT :  " << info["PHYSICAL_PORT"] << std::endl;
}

int main()
try {

	// Pipeline config
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);	// Color
	cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);	// Depth

	// RsCamera instance
	RsCamera camera(cfg);

	// Show device info
	//showDeviceInfo(camera);
	for (auto info : camera.getDeviceInfoVer2()) {
		std::cout << info.first << " :  " << info.second << std::endl;
	}

	// Align color stream to depth stream
	rs2::align align(RS2_STREAM_COLOR);

	// Display image
	cv::Mat image(cv::Size(2 * WIDTH, HEIGHT), CV_8UC3);


	while (false)
	{
		// カラーフレームを取得
		//camera.getColorFrame(image);

		// Depthフレームを取得
		//camera.getDepthFrame(image);

		// カラーフレームとDepthフレームを並べて取得
		camera.getComboFrame(image, align);

		// 画像を表示
		cv::imshow("COMBO_FRAME", image);

		// ESCで終了
		if (cv::waitKey(10) == 27) {
			cv::destroyAllWindows();
			break;
		}
	}

	return EXIT_SUCCESS;

}
catch (const rs2::error& e) {
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}