#pragma once

struct StatsIMU
{
    float lastXA {0.0};
    float lastYA {0.0};
    float lastZA {0.0};
    float lastXG {0.0};
    float lastYG {0.0};
    float lastZG {0.0};
    float last0;
    float last1;
    float last2;
    float last3;
    float XAmean {0.0};
    float YAmean {0.0};
    float ZAmean {0.0};
    float XAsigma {0.0};
    float YAsigma {0.0};
    float ZAsigma {0.0};
    float XGmean {0.0};
    float YGmean {0.0};
    float ZGmean {0.0};
    float XGsigma {0.0};
    float YGsigma {0.0};
    float ZGsigma {0.0};
    float Mean0 {0.0};
    float Mean1 {0.0};
    float Mean2 {0.0};
    float Mean3 {0.0};
    float Sigma0 {0.0};
    float Sigma1 {0.0};
    float Sigma2 {0.0};
    float Sigma3 {0.0};
};
