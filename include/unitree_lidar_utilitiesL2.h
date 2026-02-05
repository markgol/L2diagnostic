//**********************************************************************
//  Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
//
// BSD 3-Clause License
//
// Copyright (c) 2024, Unitree Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// **********************************************************************

//-----------------------------------------------------------------------
//  Additions/Corrections
//  2026-01-21  Added MAX_3DPOINTS_PER_FRAME definition
//              Added MAX_2DPOINTS_PER_FRAME definition
//              Correction to point array size in parseFromPacketPointCloud2D()
//              should have been MAX_2DPOINT_PER_FRAME not MAX_3DPOINTS_PER_FRAME
//  2026-01-24  Removed unused #includes
//
//-----------------------------------------------------------------------

#pragma once

// modifcation for use with Qt using Qvector instead of std::vector
#include <cstdint>
//#include <iostream>  // not used here
//#include <fstream>   // not used here
//#include <iomanip>  // not used here
//#include <deque>  // not used here
//#include <vector>    // not used here
//#include <memory>  // not used here
#include <cmath>
#include <chrono>
#include <numbers>

// this is required, DO NOT REMOVE
#pragma pack(push, 1)
#include "unitree_lidar_protocolL2.h"
#pragma pack(pop)

namespace unilidar_sdk2{

///////////////////////////////////////////////////////////////////////////////
// CONSTANTS
///////////////////////////////////////////////////////////////////////////////
const float DEGREE_TO_RADIAN = std::numbers::pi_v<float> / 180.0f;
const float RADIAN_TO_DEGREE = 180.0f / std::numbers::pi_v<float>;

///////////////////////////////////////////////////////////////////////////////
// TYPES
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Point Type
 */
typedef struct
{
    float x;
    float y;
    float z;
    float intensity;
    float time;    // relative time of this point from cloud stamp
    uint32_t ring; // ring is always 1
} PointUnitree;

/**
 * @brief Point Cloud Type
 */
typedef struct
{
    double stamp;    // cloud start timestamp, the point timestamp is relative to this
    uint32_t id;     // sequence id
    uint32_t ringNum; // number of rings, for L2 ringnum is always 1
    std::vector<PointUnitree> points;
} PointCloudUnitree;

///////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

inline void getSystemTimeStamp(TimeStamp &timestamp)
{
    using namespace std::chrono;
    auto now = system_clock::now();
    auto sec = time_point_cast<seconds>(now);
    auto nsec = duration_cast<nanoseconds>(now - sec);

    timestamp.sec = static_cast<uint32_t>(sec.time_since_epoch().count());
    timestamp.nsec = static_cast<uint32_t>(nsec.count());
}

inline double getSystemTimeStamp()
{
    using namespace std::chrono;
    auto now = system_clock::now();
    return duration<double>(now.time_since_epoch()).count();
}

/**
 * @brief crc32 check
 * @param buf
 * @param len
 * @return uint32_t
 */
inline uint32_t crc32(const uint8_t *buf, uint32_t len)
{
    uint8_t i;
    uint32_t crc = 0xFFFFFFFF;
    while (len--)
    {
        crc ^= *buf++;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc = (crc >> 1);
        }
    }
    return ~crc;
}

/**
 * @brief Parse from a point packet to a 3D point cloud
 * @param[out] cloud
 * @param[in] packet lidar point data packet
 * @param[in] use_system_timestamp use system timestamp, otherwise use lidar hardware timestamp
 * @param[in] range_min allowed minimum point range in meters
 * @param[in] range_max allowed maximum point range in meters
 */
inline void parseFromPacketToPointCloud(
    PointCloudUnitree &cloud,
    const LidarPointDataPacket &packet,
    bool use_system_timestamp = true,
    float range_min = 0,
    float range_max = 100
    )
{
    // scan info
    const int num_of_points = packet.data.point_num;
    const float time_step = packet.data.time_increment;
    const float scan_period = packet.data.scan_period;

    // intermediate variables
    const float sin_beta = sin(packet.data.param.beta_angle);
    const float cos_beta = cos(packet.data.param.beta_angle);
    const float sin_xi = sin(packet.data.param.xi_angle);
    const float cos_xi = cos(packet.data.param.xi_angle);
    const float cos_beta_sin_xi = cos_beta * sin_xi;
    const float sin_beta_cos_xi = sin_beta * cos_xi;
    const float sin_beta_sin_xi = sin_beta * sin_xi;
    const float cos_beta_cos_xi = cos_beta * cos_xi;

    // cloud init
    if (use_system_timestamp)
    {
        cloud.stamp = getSystemTimeStamp() - scan_period;
    }else{
        cloud.stamp = packet.data.info.stamp.sec + packet.data.info.stamp.nsec / 1.0e9;
    }
    cloud.id = 1;
    cloud.ringNum = 1;
    cloud.points.clear();
    cloud.points.reserve(MAX_3DPOINTS_PER_FRAME);

    // transform raw data to a pointcloud
    auto &ranges = packet.data.ranges;
    auto &intensities = packet.data.intensities;

    float time_relative = 0;
    float alpha_cur = packet.data.angle_min + packet.data.param.alpha_angle_bias;
    float alpha_step = packet.data.angle_increment;
    float theta_cur = packet.data.com_horizontal_angle_start + packet.data.param.theta_angle_bias;
    float theta_step = packet.data.com_horizontal_angle_step;

    float range_float;
    float sin_alpha, cos_alpha, sin_theta, cos_theta;
    float A, B, C;

    PointUnitree point3d;
    point3d.ring = 1;
    // std::cout << "packet.data.param.range_scale = " << packet.data.param.range_scale << std::endl;

    for (int j = 0; j < num_of_points; j += 1, alpha_cur += alpha_step,
             theta_cur += theta_step, time_relative += time_step)
    {
        // jump invalid points
        if (ranges[j] <= 1)
        {
            continue;
        }

        // calculate point range in float type
        range_float = packet.data.param.range_scale * ((float)ranges[j] + packet.data.param.range_bias);

        // jump points beyond range limit
        if ( range_float < packet.data.range_min || range_float > packet.data.range_max)
        {
            continue;
        }

        // jump points beyond range limit
        if (range_float < range_min || range_float > range_max)
        {
            continue;
        }

        // transform to XYZ coordinate
        sin_alpha = sin(alpha_cur);
        cos_alpha = cos(alpha_cur);
        sin_theta = sin(theta_cur);
        cos_theta = cos(theta_cur);

        A = (-cos_beta_sin_xi + sin_beta_cos_xi * sin_alpha) * range_float + packet.data.param.b_axis_dist;
        B = cos_alpha * cos_xi * range_float;
        C = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * range_float;

        point3d.x = cos_theta * A - sin_theta * B;
        point3d.y = sin_theta * A + cos_theta * B;
        point3d.z = C + packet.data.param.a_axis_dist;

        // push back this point to cloud
        point3d.intensity = intensities[j];
        point3d.time = time_relative;
        cloud.points.push_back(point3d);
    }
}

/**
 * @brief Parse from a packet to a 2D LaserScan
 * @param[out] cloud
 * @param[in] packet lidar point data packet
 * @param[in] use_system_timestamp use system timestamp, otherwise use lidar hardware timestamp
 * @param[in] range_min allowed minimum point range in meters
 * @param[in] range_max allowed maximum point range in meters
 */
inline void parseFromPacketPointCloud2D(
    PointCloudUnitree &cloud,
    const Lidar2DPointDataPacket &packet,
    bool use_system_timestamp = true,
    float range_min = 0,
    float range_max = 100)
{
    // scan info
    const int num_of_points = packet.data.point_num;
    const float time_step = packet.data.time_increment;
    const float scan_period = packet.data.scan_period;

    // cloud init
    if (use_system_timestamp)
    {
        cloud.stamp = getSystemTimeStamp() - scan_period;
    }else{
        cloud.stamp = packet.data.info.stamp.sec + packet.data.info.stamp.nsec / 1.0e9;
    }
    cloud.id = 1;
    cloud.ringNum = 1;
    cloud.points.clear();
    cloud.points.reserve(MAX_2DPOINTS_PER_FRAME); // changed from original 300

    // transform raw data to a pointcloud
    auto &ranges = packet.data.ranges;
    auto &intensities = packet.data.intensities;

    float time_relative = 0;
    float alpha_cur = packet.data.angle_min + packet.data.param.alpha_angle_bias;
    float alpha_step = packet.data.angle_increment;

    float range_float;
    float sin_alpha, cos_alpha;

    PointUnitree point3d;
    point3d.ring = 1;

    for (int j = 0; j < num_of_points; j += 1, alpha_cur += alpha_step, time_relative += time_step)
    {
        // jump invalid points
        if (ranges[j] < 1)
        {
            continue;
        }

        // calculate point range in float type
        range_float = packet.data.param.range_scale * (ranges[j] + packet.data.param.range_bias);

        // jump points beyond range limit
        if (range_float < packet.data.range_min || range_float > packet.data.range_max)
        {
            continue;
        }

        // jump points beyond range limit
        if (range_float < range_min || range_float > range_max)
        {
            continue;
        }

        // transform to XYZ coordinate
        sin_alpha = sin(alpha_cur);
        cos_alpha = cos(alpha_cur);
        point3d.x = 0;
        point3d.y = cos_alpha * range_float;
        point3d.z = sin_alpha * range_float + packet.data.param.a_axis_dist;

        // push back this point to cloud
        point3d.intensity = intensities[j];
        point3d.time = time_relative;
        cloud.points.push_back(point3d);
    }
}

}
