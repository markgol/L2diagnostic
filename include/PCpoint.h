#pragma once

// ---- Lidar point ----
#include <cstdint>

struct PCpoint
{
    float x;
    float y;
    float z;
    float intensity; //  0-255
    float time; // timestamp in seconds
    uint32_t ring;
};

