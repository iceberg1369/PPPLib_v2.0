//
// Created by cc on 7/16/20.
//

#ifndef PPPLIB_INSFUNC_H
#define PPPLIB_INSFUNC_H

#include "CmnFunc.h"

namespace PPPLib {

    typedef struct {
        cTime t_tag_;
        Vector3d gyro_;
        Vector3d acce_;
    }tImuDataUnit;

    class cImuData{
    public:
        cImuData();
        cImuData(cTime* ts,cTime* te);
        ~cImuData();

    public:
        void SetImuType(IMU_TYPE type);
        void SetImuCoordType(IMU_COORD_TYPE type);
        void SetTimeSpan(cTime* ts, cTime* te);

    public:
        cTime ts_,te_;
        IMU_TYPE imu_type_;
        IMU_COORD_TYPE imu_coord_type_;
        vector<tImuDataUnit> data_;
    };


}

#endif //PPPLIB_INSFUNC_H
