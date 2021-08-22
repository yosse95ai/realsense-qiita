#pragma once
#include <iostream>
#include "librealsense2/rs.hpp"
#include <opencv2/opencv.hpp>

#define WIDTH 640
#define HEIGHT 480
#define FPS 30

typedef std::map<std::string, std::string> DeviceInfo;
typedef std::map<std::string, DeviceInfo> SensorInfo;

/// <summary>
/// D455�J��������N���X
/// </summary>
class RsCamera
{
public:
	// �R���X�g���N�^
	RsCamera();
	RsCamera(rs2::config cfg);

	// �f�X�g���N�^
	~RsCamera() { pipe.stop(); }


	float getCenterDistance();
	void getColorFrame(cv::Mat& color_image);
	void getDepthFrame(cv::Mat& depth_image);
	void getComboFrame(cv::Mat& combo_image, rs2::align align);
	DeviceInfo getDeviceInfoVer2();
	SensorInfo getSensorsInfo();

private:
	bool isConnectedDevices();
	DeviceInfo getSensorRange(rs2::sensor sensor);

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
/// �R���X�g���N�^
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
/// �R���X�g���N�^
/// </summary>
/// <param name="cfg">�ݒ���</param>
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
/// ��ʒ����̐[�x���擾
/// </summary>
/// <returns>�[�x�̒l[m]</returns>
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
/// �J���[�t���[�����擾����
/// </summary>
/// <param name="color_image">�t���[���o�͐�</param>
void RsCamera::getColorFrame(cv::Mat& color_image)
{
	rs2::frameset frames = pipe.wait_for_frames();
	rs2::video_frame color_frame = frames.get_color_frame();

	cv::Mat image(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

	image.copyTo(color_image);
}

/// <summary>
/// �[�x�t���[�����擾����
/// </summary>
/// <param name="depth_image">�t���[���o�͐�</param>
void RsCamera::getDepthFrame(cv::Mat& depth_image)
{
	rs2::frameset frames = pipe.wait_for_frames();
	rs2::video_frame depth_frame = frames.get_depth_frame().apply_filter(color_map);

	cv::Mat image(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

	image.copyTo(depth_image);
}

/// <summary>
/// �J���[�t���[����Depth�t���[�����擾����
/// </summary>
/// <param name="combo_image">�t���[���o�͐�</param>
/// <param name="align">�A���C�������g�̐ݒ�</param>
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
/// �f�o�C�X�����擾����
/// </summary>
/// <returns>�f�o�C�X�̏��̘A�z�z��</returns>
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

/// <summary>
/// �Z���T�[�̏����擾����
/// </summary>
/// <returns>�Z���T�[�̏��</returns>
SensorInfo RsCamera::getSensorsInfo()
{
	rs2::device device = devices[0];
	std::vector<rs2::sensor> sensors = device.query_sensors();

	SensorInfo sensors_info;

	for (rs2::sensor sensor : sensors) {

		std::string name = sensor.get_info(RS2_CAMERA_INFO_NAME);

		sensors_info[name] = getSensorRange(sensor);
	}
	return sensors_info;
}

/*********************************************************
*
* Private functions
*
**********************************************************/
/// <summary>
/// �ڑ��f�o�C�X�����X�V
/// </summary>
/// <returns>T:�ڑ�����CF:�ڑ�����</returns>
bool RsCamera::isConnectedDevices() {
	devices = ctx.query_devices();
	if (!devices.size()) { 
		return false;
	}
	return true;
}

/// <summary>
/// �Z���T�[�I�v�V�����̃p�����[�^�͈͂��i�[
/// </summary>
/// <param name="sensor">�Z���T�[</param>
/// <returns>�I�v�V�����̃p�����[�^���</returns>
DeviceInfo RsCamera::getSensorRange(rs2::sensor sensor)
{
	DeviceInfo info;
	for (int i = 0; i < RS2_OPTION_COUNT; ++i) {

		rs2_option option = (rs2_option)i;

		if (sensor.supports(option)) {

			sensor.get_option(option);
			rs2::option_range range = sensor.get_option_range(option);
			auto range_key = rs2_option_to_string(option);
			auto def_value = std::to_string((int)range.def);
			auto min = std::to_string((int)range.min);
			auto max = std::to_string((int)range.max);
			auto step = std::to_string((int)range.step);

			info[range_key] = def_value + " ( " + min + " <" + step + "> " + max + " ) ";
		}
	}
	return info;
}