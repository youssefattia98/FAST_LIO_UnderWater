#include "use-ikfom.hpp"
#include "use-ikfom.hpp"
#include "IMU_Processing.hpp"
#include "auxiliary_sensor_fusion.hpp"
#include "header_link_fixture.hpp"

std::array<double, 8> SharedHeaderValuesFromOtherTranslationUnit()
{
    return SharedHeaderValues();
}

const M3D *SharedIdentityFromOtherTranslationUnit()
{
    return &Eye3d;
}
