//
// Created by cc on 7/16/20.
//

#ifndef PPPLIB_INSFUNC_H
#define PPPLIB_INSFUNC_H

#include "CmnFunc.h"

namespace PPPLib {

    Eigen::Matrix3d VectorSkew(const Eigen::Vector3d& vec);
    Eigen::Matrix3d Quaternion2RotationMatrix(const Eigen::Quaterniond& q);
    Eigen::Quaterniond RotationMatrix2Quaternion(const Eigen::Matrix3d& m);
    Eigen::Quaterniond Euler2Quaternion(const Vector3d& rpy);
    Eigen::Matrix3d Euler2RotationMatrix(const Vector3d& rpy);
    Eigen::Vector3d RotationMatrix2Euler(const Matrix3d &m);
    Eigen::Vector3d Quaternion2Euler(const Quaterniond& q);
    Eigen::Quaterniond RotationVector2Quaternion(const Vector3d& rv);

    Eigen::Vector3d CalculateGravity(const Vector3d coord_blh,bool is_ecef);

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

    class cInsMech{
    public:
        cInsMech();
        ~cInsMech();

    public:
        tSolInfoUnit InsMechanization(tImuDataUnit& pre_imu_data,tImuDataUnit& cur_imu_data,const tSolInfoUnit& sol_info);

    private:
        Eigen::Quaterniond AttitudeUpdate(tImuDataUnit& pre_imu_data,tImuDataUnit& cur_imu_data,const tSolInfoUnit& pre_sol_info);
        Eigen::Vector3d VelocityUpdate(tImuDataUnit& pre_imu_data,tImuDataUnit& cur_imu_data,const tSolInfoUnit& pre_sol_info);
        Eigen::Vector3d PositionUpdate(const tSolInfoUnit& pre_sol_info,const Vector3d& cur_vel,double dt);
    };


}

#endif //PPPLIB_INSFUNC_H
