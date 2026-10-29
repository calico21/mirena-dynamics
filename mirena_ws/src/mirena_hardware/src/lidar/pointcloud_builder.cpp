#include "pointcloud_builder.hpp"
#include <cstring>
#include <cmath>
#include <arpa/inet.h>

// Global optimization for the whole file. -ffast-math is already on from
// CMakeLists; unroll-loops is what this adds.
#pragma GCC optimize("unroll-loops")

#define NUM_CHANNELS 5
#define NUM_DATA_BLOCKS 25
#define RADIAL_RES 0.005f
#define ANG_RES (0.01f * M_PI / 180.0f)

using namespace mirena;

struct alignas(4) LidarPoint
{
    float x, y, z, intensity;
};

namespace {
constexpr size_t kPointSize = sizeof(LidarPoint);
constexpr size_t kBytesPerPacket = NUM_DATA_BLOCKS * NUM_CHANNELS * kPointSize;
} // namespace

Timestamp parse_timestamp(const uint8_t timestamp[10])
{
    Timestamp ts;

    ts.seconds = ((int64_t)timestamp[0] << 40) |
                 ((int64_t)timestamp[1] << 32) |
                 ((int64_t)timestamp[2] << 24) |
                 ((int64_t)timestamp[3] << 16) |
                 ((int64_t)timestamp[4] << 8) |
                 ((int64_t)timestamp[5]);

    ts.microseconds = ((int64_t)timestamp[6] << 24) |
                      ((int64_t)timestamp[7] << 16) |
                      ((int64_t)timestamp[8] << 8) |
                      ((int64_t)timestamp[9]);

    return ts;
}

void PointcloudBuilder::start_frame(const StampedMSOPPacket &stamped_packet,
                                    const Timestamp &packet_timestamp)
{
    product = this->make_new_pointcloud(stamped_packet.timestamp);
    frame_start_time = packet_timestamp;
    frame_start_ros = stamped_packet.timestamp.seconds();
    frame_start_theta = deskew_enabled_ ? imu_.angle_at(frame_start_ros) : 0.0;
    compensated_ = true;
}

std::unique_ptr<sensor_msgs::msg::PointCloud2> PointcloudBuilder::process_packet(const StampedMSOPPacket &stamped_packet)
{
    // Header check
    const lidar::MSOPPacket *packet = &stamped_packet.packet;
    uint8_t expected_pkt_header[4] = {0x55, 0xaa, 0x5a, 0xa5};
    if (std::memcmp(packet->header.pkt_header, expected_pkt_header, 4) != 0)
    {
        return nullptr;
    }

    Timestamp packet_timestamp = parse_timestamp(packet->header.timestamp);

    if (product == nullptr)
    {
        start_frame(stamped_packet, packet_timestamp);
    }

    uint16_t seq_number = ntohs(packet->header.pkt_psn);
    std::unique_ptr<sensor_msgs::msg::PointCloud2> ret = nullptr;

    // Frame boundary
    if (seq_number < last_sequence_number && (last_sequence_number - seq_number) > (max_seq_num / 10))
    {
        if (!product->data.empty())
        {
            product->width = product->data.size() / product->point_step;
            product->row_step = product->data.size();
            ret = std::move(product); // Transfer ownership
        }
        start_frame(stamped_packet, packet_timestamp);
    }
    last_sequence_number = seq_number;

    size_t current_offset = product->data.size();
    product->data.resize(current_offset + kBytesPerPacket);
    uint8_t *write_ptr = &product->data[current_offset];
    size_t bytes_written = 0;

    // Offset of this packet from the frame start, on the lidar's own clock.
    const double packet_frame_offset = packet_timestamp.diff_seconds(frame_start_time);

    // De-skew rotation, resolved once per packet rather than 25 times. The
    // blocks inside a packet span at most 255 us, far too short for the yaw
    // rate to change, so the angle is carried across them linearly from the
    // rate at the packet's own instant -- one interpolation plus a
    // multiply-add per block.
    ImuHistory::State at_packet{0.0, 0.0};
    double packet_theta = 0.0;
    if (deskew_enabled_)
    {
        bool covered = false;
        at_packet = imu_.state_at(frame_start_ros + packet_frame_offset, &covered);
        packet_theta = at_packet.theta - frame_start_theta;
        if (!covered) compensated_ = false;
    }

    for (int block_n = 0; block_n < NUM_DATA_BLOCKS; block_n++)
    {
        const lidar::MSOPDataBlock *block = &packet->blocks[block_n];
        // time_offset is a uint8_t: microseconds of this block within the
        // packet, 0-255. It used to be masked with 0x00111111, which against a
        // byte reduces to 0x11 and leaves only the values {0, 1, 16, 17} -- a
        // hex literal where a bit pattern was meant. The whole term is bounded
        // by 255 us either way, so the bug was worth about 0.03 deg of de-skew
        // at 2 rad/s; the full byte is the reading that makes physical sense,
        // but confirm against the datasheet if the top bits are flags.
        const double block_offset_s = static_cast<double>(block->time_offset) * 1e-6;
        const float block_azimuth_fix =
            static_cast<float>(packet_theta + at_packet.omega * block_offset_s);

// Explicitly helping the compiler vectorize this loop
#pragma GCC ivdep
        for (int channel_n = 0; channel_n < NUM_CHANNELS; channel_n++)
        {
            const lidar::ChannelBlock *src = &block->channels[channel_n];

            float radius = (float)((src->radius[0] << 8) | src->radius[1]) * RADIAL_RES;
            if (radius < 0.4f)
                continue;

            float elevation = (float)((src->elevation[0] << 8 | src->elevation[1]) - 32768) * ANG_RES;
            float azimuth = (float)((src->azimuth[0] << 8 | src->azimuth[1]) - 32768) * ANG_RES;
            azimuth += block_azimuth_fix;

            // Using sincosf for Jetson hardware acceleration
            float s_el, c_el, s_az, c_az;
            sincosf(elevation, &s_el, &c_el);
            sincosf(azimuth, &s_az, &c_az);

            float r_c_el = radius * c_el;

            LidarPoint p;
            p.x = r_c_el * c_az;
            p.y = r_c_el * s_az;
            p.z = radius * s_el;
            p.intensity = (float)src->intensity / 255.0f;

            std::memcpy(write_ptr, &p, kPointSize);
            write_ptr += kPointSize;
            bytes_written += kPointSize;
        }
    }

    product->data.resize(current_offset + bytes_written);
    return ret;
}

std::unique_ptr<sensor_msgs::msg::PointCloud2> PointcloudBuilder::make_new_pointcloud(const builtin_interfaces::msg::Time &stamp)
{
    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    msg->header.frame_id = frame_id;
    msg->header.stamp = stamp;
    msg->height = 1;
    msg->point_step = kPointSize;
    msg->is_bigendian = false;
    // Returns below 0.4 m are skipped rather than written as NaN, so the cloud
    // holds no invalid points. Left false, every PCL consumer runs a
    // removeNaNFromPointCloud pass over 78k points for nothing.
    msg->is_dense = true;
    msg->fields.resize(4);
    const char *names[] = {"x", "y", "z", "intensity"};
    for (int i = 0; i < 4; i++)
    {
        msg->fields[i].name = names[i];
        msg->fields[i].offset = i * 4;
        msg->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg->fields[i].count = 1;
    }

    msg->data.reserve(max_seq_num * kBytesPerPacket);
    return msg;
}
