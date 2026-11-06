#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <cstring>

// Linux/Unix Hardware Headers
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <time.h> // Added for clock_gettime


// ROS 2 and Computer Vision Headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#define IMAGE_PUB_TOPIC "/sensors/camera/image"
#define CAMERA_INFO_PUB_TOPIC "/sensors/camera/camera_info"
#define REQ_BUFFER_COUNT 4

struct V4L2Buffer {
    void* start;
    size_t length;
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class V4L2CameraDriverNode : public rclcpp::Node {
public:
    V4L2CameraDriverNode() : rclcpp::Node("mirena_camera") {
        // 1. Declare parameters with default fallback values
        this->declare_parameter("video_device", "/dev/ter_camera_video");
        this->declare_parameter("frame_id", "camera_optical_frame");
        this->declare_parameter("image_width", 640);
        this->declare_parameter("image_height", 480);
        this->declare_parameter("fps", 90);
        this->declare_parameter("camera_info_url", "file:///home/kor/Personal/Proyects/Racing/mirena_ws/src/mirena_hardware/config/camera_info.yaml"); 

        this->declare_parameter("camera_name", "mirena_camera");

        // 2. Fetch the parameter values
        device_    = this->get_parameter("video_device").as_string();
        frame_id_  = this->get_parameter("frame_id").as_string();
        width_     = this->get_parameter("image_width").as_int();
        height_    = this->get_parameter("image_height").as_int();
        fps_       = this->get_parameter("fps").as_int();
        cinfo_url_ = this->get_parameter("camera_info_url").as_string();
        cname_     = this->get_parameter("camera_name").as_string();

        RCLCPP_INFO(get_logger(), "ROS Parameters Loaded: %dx%d @ %d FPS on %s", 
                    width_, height_, fps_, device_.c_str());

        // Initialize ROS publishers
        auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(IMAGE_PUB_TOPIC, 10);
        info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(CAMERA_INFO_PUB_TOPIC, qos);

        cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, cname_, cinfo_url_);

        // Lifecycle Publishers
        //rclcpp_lifecycle::LifecyclePublisher<mirena_common::msg::Heartbeat>::SharedPtr _report_pub;


        // Initialize hardware access and establish MMAP rings
        if (!init_v4l2_device()) {
            RCLCPP_ERROR(get_logger(), "Initialization Failed: Unable to map V4L2 device %s", device_.c_str());
            exit(39);
        }

        // Start Hardware Streaming instantly on creation
        if (!start_v4l2_streaming()) {
            RCLCPP_ERROR(get_logger(), "Streaming Activation Failed: Stream mapping rejected by V4L2 subsystem.");
            exit(39);
        }

        is_streaming_ = true;
        capture_thread_ = std::thread(&V4L2CameraDriverNode::capture_loop, this);
    }

    ~V4L2CameraDriverNode() {
        is_streaming_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        stop_v4l2_streaming();
        unmap_v4l2_buffers();
        close_v4l2_device();
    }

private:
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> image_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> info_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;

    std::thread capture_thread_;
    std::atomic<bool> is_streaming_{false};
    bool logged_timestamp_source_{false}; // Ensure we only log hardware capabilities once

    std::string device_, cinfo_url_, cname_, frame_id_;
    int width_, height_, fps_;
    int fd_{-1};
    std::vector<V4L2Buffer> buffers_;

    bool init_v4l2_device() {
        fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (fd_ < 0) return false;

        struct v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width_;
        fmt.fmt.pix.height = height_;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) return false;

        struct v4l2_streamparm streamparm{};
        streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        streamparm.parm.capture.timeperframe.numerator = 1;
        streamparm.parm.capture.timeperframe.denominator = fps_;
        ioctl(fd_, VIDIOC_S_PARM, &streamparm);

        struct v4l2_requestbuffers req{};
        req.count = REQ_BUFFER_COUNT;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) return false;

        buffers_.resize(req.count);
        for (size_t i = 0; i < req.count; ++i) {
            struct v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) return false;

            buffers_[i].length = buf.length;
            buffers_[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
            if (buffers_[i].start == MAP_FAILED) return false;
        }
        return true;
    }

    bool start_v4l2_streaming() {
        for (size_t i = 0; i < buffers_.size(); ++i) {
            struct v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) return false;
        }
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        return ioctl(fd_, VIDIOC_STREAMON, &type) >= 0;
    }

    void capture_loop() {
        while (is_streaming_ && rclcpp::ok()) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(fd_, &fds);

            struct timeval tv{};
            tv.tv_sec = 1; 
            tv.tv_usec = 0;

            int r = select(fd_ + 1, &fds, NULL, NULL, &tv);
            if (r <= 0) continue; 

            struct v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) continue;

            // ==========================================
            // Epoch Time Compensation for V4L2 Timestamps
            // ==========================================
            rclcpp::Time frame_stamp;

            // Ensure the kernel provided a monotonic timestamp
            if ((buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
                
                // 1. Capture current system clocks instantly
                struct timespec ts_mono, ts_real;
                clock_gettime(CLOCK_MONOTONIC, &ts_mono);
                clock_gettime(CLOCK_REALTIME, &ts_real);

                // 2. Convert all to nanoseconds for math
                int64_t v4l2_ns = static_cast<int64_t>(buf.timestamp.tv_sec) * 1e9 +
                                  static_cast<int64_t>(buf.timestamp.tv_usec) * 1e3;
                int64_t mono_ns = static_cast<int64_t>(ts_mono.tv_sec) * 1e9 + ts_mono.tv_nsec;
                int64_t real_ns = static_cast<int64_t>(ts_real.tv_sec) * 1e9 + ts_real.tv_nsec;

                // 3. Compute dynamic offset and apply to hardware stamp
                int64_t offset_ns = real_ns - mono_ns;
                int64_t frame_epoch_ns = v4l2_ns + offset_ns;

                frame_stamp = rclcpp::Time(frame_epoch_ns, RCL_SYSTEM_TIME);

                // Log hardware capabilities on the first frame
                if (!logged_timestamp_source_) {
                    std::string tstamp_src = "EOF (End of Frame)";
                    if ((buf.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) == V4L2_BUF_FLAG_TSTAMP_SRC_SOE) {
                        tstamp_src = "SOE (Start of Exposure)";
                    }
                    RCLCPP_INFO(get_logger(), "Hardware Timestamp Active | Source: %s | Epoch compensated.", tstamp_src.c_str());
                    logged_timestamp_source_ = true;
                }
            } else {
                // Fallback to ROS arrival time if V4L2 driver uses an unsupported clock (rare)
                frame_stamp = this->now();
                if (!logged_timestamp_source_) {
                    RCLCPP_WARN(get_logger(), "V4L2 driver does not provide MONOTONIC timestamps. Falling back to software arrival time.");
                    logged_timestamp_source_ = true;
                }
            }
            // ==================================================================================================================================

            auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
            img_msg->header.stamp = frame_stamp;
            img_msg->header.frame_id = frame_id_;

            cv::Mat raw_mjpeg_data(1, buf.bytesused, CV_8UC1, buffers_[buf.index].start);
            cv::Mat bgr_frame = cv::imdecode(raw_mjpeg_data, cv::IMREAD_COLOR);

            if (!bgr_frame.empty()) {
                img_msg->height = bgr_frame.rows;
                img_msg->width = bgr_frame.cols;
                img_msg->encoding = "bgr8";
                img_msg->is_bigendian = 0;
                img_msg->step = bgr_frame.cols * bgr_frame.elemSize();

                size_t size = img_msg->step * bgr_frame.rows;
                img_msg->data.resize(size);

                std::memcpy(&img_msg->data[0], bgr_frame.data, size);

                image_pub_->publish(std::move(img_msg));

                sensor_msgs::msg::CameraInfo info_msg = cinfo_manager_->getCameraInfo();
                info_msg.header.stamp = frame_stamp;
                info_msg.header.frame_id = frame_id_;
                if (!cinfo_manager_->isCalibrated()) {
                    info_msg.width = bgr_frame.cols;
                    info_msg.height = bgr_frame.rows;
                }
                info_pub_->publish(info_msg);
            }

            ioctl(fd_, VIDIOC_QBUF, &buf);
        }
    }

    void stop_v4l2_streaming() {
        if (fd_ >= 0) {
            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(fd_, VIDIOC_STREAMOFF, &type);
        }
    }

    void unmap_v4l2_buffers() {
        for (auto& buf : buffers_) {
            if (buf.start != MAP_FAILED) munmap(buf.start, buf.length);
        }
        buffers_.clear();
    }

    void close_v4l2_device() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }
    void report_callback()
    {
        //if (!_report_pub->is_activated())
        //{
        //    return;
        //}

        //mirena_common::msg::Heartbeat msg;
        //msg.timestamp = this->now();
        //msg.node_name = this->get_name();

        // Package all metadata into JSON layout structure matching supervisor requirements
       /*
        nlohmann::json report_json;
        report_json["status_state"] = "WORKING";
        report_json["last_update_ms_elapsed"] = this->_last_update_ms_elapsed;
        report_json["last_update_data_stamp"] = this->_last_update_data_stamp.seconds();
        report_json["last_update_total_landmark_count"] = this->_last_update_total_landmark_count;
        report_json["last_run_cones_seen"] = this->_last_update_total_landmark_count;

       */ 
        //msg.report = report_json.dump();
        //_report_pub->publish(msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<V4L2CameraDriverNode>());
    rclcpp::shutdown();
    return 0;
}
