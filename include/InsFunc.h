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
        cTime t_tag;
        Vector3d gyro;
        Vector3d acce;
    }tImuDataUnit;

    class cImuData{
    public:
        cImuData();
        cImuData(cTime* ts,cTime* te);
        ~cImuData();

    public:
        void SetImu(IMU_TYPE imu_type,IMU_COORD_TYPE coord_type,IMU_DATA_FORMAT data_format);
        void SetImuType(IMU_TYPE type);
        void SetImuCoordType(IMU_COORD_TYPE type);
        void SetTimeSpan(cTime* ts, cTime* te);

    public:
        cTime ts_,te_;
        IMU_TYPE imu_type_;
        IMU_COORD_TYPE imu_coord_type_;
        IMU_DATA_FORMAT data_format_;
        vector<tImuDataUnit> data_;
    };

    typedef struct{
        cTime t_tag;
        Vector3d raw_gyro,raw_acce;
        Vector3d cor_gyro,cor_acce;

        Vector3d re,ve,ae;
        Vector3d rn,vn,an;
        Matrix3d Cbe,Cbn;

        Vector3d ba,bg;

        double dt;

    }tImuInfoUnit;

    class cInsMech{
    public:
        cInsMech();
        ~cInsMech();

    public:
        tImuInfoUnit InsMechanization(tPPPLibConf C,tImuInfoUnit& pre_imu_info,tImuInfoUnit& cur_imu_info);
        Eigen::MatrixXd StateTransferMat(tPPPLibConf C,tImuInfoUnit& pre_imu_info,tImuInfoUnit& cur_imu_info,int nx);

    private:
        Eigen::Quaterniond AttitudeUpdate(tImuInfoUnit& pre_imu_info,tImuInfoUnit& cur_imu_info);
        Eigen::Vector3d VelocityUpdate(tImuInfoUnit& pre_imu_info,tImuInfoUnit& cur_imu_info);
        Eigen::Vector3d PositionUpdate(const tImuInfoUnit& pre_imu_info,const Vector3d& cur_vel,double dt);
    };


}

#endif //PPPLIB_INSFUNC_H
