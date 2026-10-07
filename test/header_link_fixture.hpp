#pragma once

#include <array>

inline std::array<double, 8> SharedHeaderValues()
{
    state_ikfom state;
    input_ikfom input;
    state.vel = V3D(1, 2, 3);
    input.acc = V3D(0, 0, 9.81);
    input.gyro = V3D(0, 0, 0.2);
    const auto derivative = get_f(state, input);
    const auto state_jacobian = df_dx(state, input);
    const auto noise_jacobian = df_dw(state, input);
    PointType first{}, second{};
    first.x = 2;
    first.y = 3;
    const auto stamp = get_ros_time(12.25);
    ImuProcess imu;
    return {derivative[0], derivative[5], state_jacobian(0, 12),
            noise_jacobian(3, 0), process_noise_cov()(0, 0),
            calc_dist(first, second), get_time_sec(stamp),
            static_cast<double>(imu.IsInitialized())};
}

std::array<double, 8> SharedHeaderValuesFromOtherTranslationUnit();
const M3D *SharedIdentityFromOtherTranslationUnit();
