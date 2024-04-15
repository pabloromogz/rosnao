#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include "rosnao_common/common.hpp"

#pragma once

namespace rosnao{
    namespace transport{
        struct SHMVelIMU{
            boost::interprocess::interprocess_mutex mutex;
            uint32_t seq = 0;
            // int32_t sec = 0, usec = 0;
            double lin_vel_x, lin_vel_y, ang_vel_z, imu_ang_z;
        };
    }
}