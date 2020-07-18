//
// Created by cc on 7/16/20.
//

#include "InsFunc.h"

namespace PPPLib {

    cImuData::cImuData(){}

    cImuData::cImuData(PPPLib::cTime *ts, PPPLib::cTime *te){
        if(ts) ts_=*ts;
        if(te) te_=*te;
    }

    cImuData::~cImuData() {data_.clear();}

    void cImuData::SetImuType(PPPLib::IMU_TYPE type) {imu_type_=type;}

    void cImuData::SetImuCoordType(PPPLib::IMU_COORD_TYPE type) {imu_coord_type_=type;}

    void cImuData::SetTimeSpan(cTime *ts, cTime *te) {
        if(ts) ts_=*ts;
        if(te) te_=*te;
    }

}