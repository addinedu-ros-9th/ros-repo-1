/**
 * @file      CameraPublisher.cpp
 * @brief     angstrong camera UDP streamer. (ROS Topic Publisher REMOVED)
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 * @author    Angstrong SDK develop Team / Modified for UDP streaming
 * @date      2025/07/31
 * @version   1.5 (Periodic Timer Stream Management)
 */

#include "CameraPublisher.h"
#include <vector>
#include "opencv2/opencv.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <chrono>

// SDK 헤더에 정의된 플래그를 사용하기 위해 추가
#include "as_camera_sdk_def.h"

#define LOG_PARA(value, str)\
    do { \
        if (value != -1) { \
            RCLCPP_INFO(this->get_logger(), "get %s %d", str, value); \
        } \
    } while(0)

// CameraPublisher 생성자
CameraPublisher::CameraPublisher() : Node("camera_publisher_udp")
{
    initLaunchParams();
    m_nodeNameSpace = get_namespace();

    const char* dest_ip = "127.0.0.1";
    RCLCPP_INFO(this->get_logger(), "UDP 소켓을 목적지 %s로 초기화합니다.", dest_ip);

    // UDP 소켓 초기화
    rgb_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (rgb_sock_ < 0) { RCLCPP_ERROR(this->get_logger(), "RGB 소켓 생성 실패"); }
    memset(&rgb_dest_addr_, 0, sizeof(rgb_dest_addr_));
    rgb_dest_addr_.sin_family = AF_INET;
    rgb_dest_addr_.sin_port = htons(9090);
    if (inet_pton(AF_INET, dest_ip, &rgb_dest_addr_.sin_addr) <= 0) { RCLCPP_ERROR(this->get_logger(), "잘못된 RGB 목적지 주소"); }

    depth_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (depth_sock_ < 0) { RCLCPP_ERROR(this->get_logger(), "Depth 소켓 생성 실패"); }
    memset(&depth_dest_addr_, 0, sizeof(depth_dest_addr_));
    depth_dest_addr_.sin_family = AF_INET;
    depth_dest_addr_.sin_port = htons(9091);
    if (inet_pton(AF_INET, dest_ip, &depth_dest_addr_.sin_addr) <= 0) { RCLCPP_ERROR(this->get_logger(), "잘못된 Depth 목적지 주소"); }

    // --- 해결책: 1초마다 스트림 상태를 확인하고 시작하는 주기적인 타이머 생성 ---
    m_stream_start_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&CameraPublisher::ensureStreamsStarted, this));
}

// 소멸자
CameraPublisher::~CameraPublisher()
{
    stop();
    if (rgb_sock_ >= 0) close(rgb_sock_);
    if (depth_sock_ >= 0) close(depth_sock_);
}

// 타이머에 의해 주기적으로 호출될 스트림 관리 함수
void CameraPublisher::ensureStreamsStarted()
{
    // 이 함수는 주기적으로 호출되므로 cancel()을 제거합니다.
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "주기적 스트림 상태 확인 중...");

    if (m_camera_map.empty()) {
        return;
    }

    for (auto const& [cam_ptr, cam_obj] : m_camera_map) {
        if (cam_ptr) {
            unsigned int stream_flags = DEPTH_IMG_FLG | RGB_IMG_FLG;
            int ret = AS_SDK_StartStream(cam_ptr, stream_flags);
            if (ret != 0) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "주기적 스트림 시작 실패! Error code: %d", ret);
            }
        }
    }
}

// onCameraOpen: 초기화만 담당
int CameraPublisher::onCameraOpen(AS_CAM_PTR pCamera)
{
    RCLCPP_INFO(this->get_logger(), "카메라 열림");
    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        camIt->second->init();
    }

    int ret = setResolution(pCamera, m_launch_param);
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "해상도 설정 실패");
    }
    return 0;
}

// 새 프레임 수신 시 UDP로 직접 전송
void CameraPublisher::onCameraNewFrame(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData)
{
    (void)pCamera; // pCamera 변수는 사용하지 않으므로 경고 방지

    // RGB 이미지 처리 및 UDP 전송
    if (pstData->rgbImg.size > 0 && rgb_sock_ >= 0) {
        cv::Mat rgb_image(pstData->rgbImg.height, pstData->rgbImg.width, CV_8UC3, pstData->rgbImg.data);
        std::vector<uchar> rgb_buf;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        cv::imencode(".jpg", rgb_image, rgb_buf, params);
        if (rgb_buf.size() < 65000) {
            sendto(rgb_sock_, rgb_buf.data(), rgb_buf.size(), 0, (struct sockaddr *)&rgb_dest_addr_, sizeof(rgb_dest_addr_));
        }
    }

    // Depth 이미지 처리 및 UDP 전송
    if (pstData->depthImg.size > 0 && depth_sock_ >= 0) {
        cv::Mat depth_image(pstData->depthImg.height, pstData->depthImg.width, CV_16UC1, pstData->depthImg.data);
        std::vector<uchar> depth_buf;
        std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 1};
        cv::imencode(".png", depth_image, depth_buf, params);
        if (depth_buf.size() < 65000) {
            sendto(depth_sock_, depth_buf.data(), depth_buf.size(), 0, (struct sockaddr *)&depth_dest_addr_, sizeof(depth_dest_addr_));
        }
    }
}


// ... (start, stop, onCameraAttached 등 나머지 함수들은 기존과 거의 동일) ...
// (전체 코드를 위해 아래에 다시 포함)

// 서버 시작
int CameraPublisher::start()
{
    int ret = 0;
    std::string config_path;
    this->declare_parameter<std::string>("confiPath", "");
    this->get_parameter<std::string>("confiPath", config_path);
    if (config_path.size() == 0) {
        RCLCPP_ERROR(this->get_logger(), "Config 파일 경로 에러");
        return -1;
    }

    if (server == nullptr) {
        server = new CameraSrv(this, config_path);
        ret = server->start();
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "서버 시작 실패");
        }
    }
    return 0;
}

// 서버 중지
void CameraPublisher::stop()
{
    if (server != nullptr) {
        for (auto const& [cam_ptr, cam_obj] : m_camera_map) {
            AS_SDK_StopStream(cam_ptr);
        }
        server->stop();
        delete server;
        server = nullptr;
    }
    m_camera_map.erase(m_camera_map.begin(), m_camera_map.end());
}


// 카메라 연결 시 호출
int CameraPublisher::onCameraAttached(AS_CAM_PTR pCamera, CamSvrStreamParam_s &param,
                                      const AS_SDK_CAM_MODEL_E &cam_type)
{
    (void)param; // param 변수는 사용하지 않으므로 경고 방지
    AS_CAM_ATTR_S attr_t;
    memset(&attr_t, 0, sizeof(AS_CAM_ATTR_S));
    AS_SDK_GetCameraAttrs(pCamera, attr_t);
    logCameraPathInfo(attr_t);

    unsigned int dev_idx = m_camera_map.size();
    m_camera_map.insert(std::make_pair(pCamera, new Camera(pCamera, cam_type, m_nodeNameSpace, dev_idx)));
    m_cam_type_map.insert(std::make_pair(pCamera, cam_type));
    RCLCPP_INFO(this->get_logger(), "카메라 연결됨. UDP 스트리밍 준비 완료.");
    
    return 0;
}

// 카메라 연결 해제 시 호출
int CameraPublisher::onCameraDetached(AS_CAM_PTR pCamera)
{
    RCLCPP_INFO(this->get_logger(), "카메라 연결 해제됨");
    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        delete camIt->second;
        m_camera_map.erase(pCamera);
    }
    if (m_cam_type_map.find(pCamera) != m_cam_type_map.end()) {
        m_cam_type_map.erase(pCamera);
    }
    return 0;
}

// 해상도 설정
int CameraPublisher::setResolution(AS_CAM_PTR pCamera, LAUNCH_CONFI_PARAM_S para)
{
    int ret = 0;
    if ((para.set_depth_width != -1) && (para.set_depth_height != -1) && (para.set_fps != -1)) {
        AS_STREAM_Param_s depthInfo;
        depthInfo.width = para.set_depth_width;
        depthInfo.height = para.set_depth_height;
        depthInfo.fps = para.set_fps;
        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_DEPTH, &depthInfo);
        if (ret < 0) RCLCPP_ERROR(this->get_logger(), "Depth 파라미터 설정 실패");
    }
    if ((para.set_rgb_width != -1) && (para.set_rgb_height != -1) && (para.set_fps != -1)) {
        AS_STREAM_Param_s rgbInfo;
        rgbInfo.width = para.set_rgb_width;
        rgbInfo.height = para.set_rgb_height;
        rgbInfo.fps = para.set_fps;
        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_RGB, &rgbInfo);
        if (ret < 0) RCLCPP_ERROR(this->get_logger(), "RGB 파라미터 설정 실패");
    }
    return ret;
}

// 카메라 경로 정보 로깅
void CameraPublisher::logCameraPathInfo(AS_CAM_ATTR_S &attr_t)
{
    switch (attr_t.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        RCLCPP_INFO(this->get_logger(), "USB 카메라: bnum:%d, dnum:%d, port:%s", 
                    attr_t.attr.usbAttrs.bnum, attr_t.attr.usbAttrs.dnum, attr_t.attr.usbAttrs.port_numbers);
        break;
    case AS_CAMERA_ATTR_NET:
        RCLCPP_INFO(this->get_logger(), "네트워크 카메라: ip:%s, port:%d", 
                    attr_t.attr.netAttrs.ip_addr, attr_t.attr.netAttrs.port);
        break;
    default:
        break;
    }
}

// 런치 파일 파라미터 초기화
int CameraPublisher::initLaunchParams()
{
    memset(&m_launch_param, 0, sizeof(LAUNCH_CONFI_PARAM_S));
    this->declare_parameter<int>("depth_width", -1);
    this->get_parameter_or<int>("depth_width", m_launch_param.set_depth_width, -1);
    this->declare_parameter<int>("depth_height", -1);
    this->get_parameter_or<int>("depth_height", m_launch_param.set_depth_height, -1);
    this->declare_parameter<int>("rgb_width", -1);
    this->get_parameter_or<int>("rgb_width", m_launch_param.set_rgb_width, -1);
    this->declare_parameter<int>("rgb_height", -1);
    this->get_parameter_or<int>("rgb_height", m_launch_param.set_rgb_height, -1);
    this->declare_parameter<int>("fps", -1);
    this->get_parameter_or<int>("fps", m_launch_param.set_fps, -1);
    
    printLaunchParams(m_launch_param);
    return 0;
}

// 읽어온 런치 파라미터 출력
int CameraPublisher::printLaunchParams(LAUNCH_CONFI_PARAM_S para)
{
    LOG_PARA(para.set_depth_width, "depth_width");
    LOG_PARA(para.set_depth_height, "depth_height");
    LOG_PARA(para.set_rgb_width, "rgb_width");
    LOG_PARA(para.set_rgb_height, "rgb_height");
    LOG_PARA(para.set_fps, "set_fps");
    return 0;
}

// SDK가 요구할 수 있는 비어있는 콜백 함수들
int CameraPublisher::onCameraClose(AS_CAM_PTR pCamera) { (void)pCamera; return 0; }
int CameraPublisher::onCameraStart(AS_CAM_PTR pCamera) { (void)pCamera; return 0; }
int CameraPublisher::onCameraStop(AS_CAM_PTR pCamera) { (void)pCamera; return 0; }
void CameraPublisher::onCameraNewMergeFrame(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData) { (void)pCamera; (void)pstData; }
