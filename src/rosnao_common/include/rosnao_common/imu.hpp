#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "rosnao_common/common.hpp"

#ifndef ROSNAO_COMMON_IMU_HPP
#define ROSNAO_COMMON_IMU_HPP

namespace rosnao{
    namespace transport{
        struct SHMIMU{
            boost::interprocess::interprocess_mutex mutex;
            uint32_t seq = 0;
            // int32_t sec = 0, usec = 0;
            // double ang_vel_x, ang_vel_y, ang_vel_z, acc_x, acc_y, acc_z, ang_x, ang_y, ang_z;
            double data[9];
        };
    }
}
#endif