#pragma once
#include <stdint.h>

namespace mirena::lidar
{
    // These structs should only be used for packet unpacking convenience. Do not use them as normal data structures willy nilly
    struct MSOPHeader
    {
        uint8_t pkt_header[4];
        uint16_t pkt_psn;
        uint16_t protocol_version;
        uint8_t wave_mode;
        uint8_t time_sync_mode;
        uint8_t timestamp[10];
        uint8_t reserved[10];
        uint8_t lidar_type;
        uint8_t mems_tmp;
    } __attribute__((packed));

    struct ChannelBlock
    {
        uint8_t radius[2];
        uint8_t elevation[2];
        uint8_t azimuth[2];
        uint8_t intensity;
        uint8_t reserved[2];
    } __attribute__((packed));

    struct MSOPDataBlock
    {
        uint8_t time_offset;
        uint8_t return_seq;
        ChannelBlock channels[5];
    } __attribute__((packed));

    struct MSOPPacket
    {
        MSOPHeader header;
        MSOPDataBlock blocks[25];
        uint8_t tail[3];
    } __attribute__((packed));
    
    struct DIFOPData
    {
        uint8_t frame_rate_setting;
        uint8_t eth_src_ip[4];
        uint8_t eth_dest_ip[4];
        uint8_t eth_local_mac_addr[6];
        uint8_t eth_msop_port[2];
        uint8_t eth_difop_port[2];
        uint8_t fov_hor_start_angle[2];
        uint8_t fov_hor_end_angle[2];
        uint8_t fov_ver_start_angle[2];
        uint8_t fov_ver_end_angle[2];
        uint8_t version_number_prog_logic[5];
        uint8_t version_number_prog_sys[5];
        uint8_t product_serial_num[6];
        uint8_t wave_mode[1];
        uint8_t time_sincronization_mode[1];
        uint8_t time_sincronization_status[1];
        uint8_t time[10];
        uint8_t operating_status[31];
        uint8_t diagnose_info[29];
        uint8_t reserved[60+71];
    } __attribute__((packed));

    struct DIFOPPacket
    {
        uint8_t header[8];
        uint8_t reserved;
        DIFOPData data;
    } __attribute__((packed));

} // namespace mirena
