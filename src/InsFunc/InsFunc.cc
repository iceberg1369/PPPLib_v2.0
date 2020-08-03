//
// Created by cc on 7/16/20.
//

#include "InsFunc.h"

namespace PPPLib {

    Eigen::Matrix3d VectorSkew(const Eigen::Vector3d& vec){
        Eigen::Matrix3d dcm=Matrix3d::Zero();

        dcm(0,1)=-vec(2);
        dcm(0,2)=vec(1);

        dcm(1,0)=vec(2);
        dcm(1,2)=-vec(0);

        dcm(2,0)=-vec(1);
        dcm(2,1)=vec(0);
        return dcm;
    }

    Eigen::Matrix3d Quaternion2RotationMatrix(const Eigen::Quaterniond& q){
        return q.toRotationMatrix();
    }

    Eigen::Quaterniond RotationMatrix2Quaternion(const Eigen::Matrix3d& m){
        Eigen::Quaterniond q(m);
        return q;
    }

    Eigen::Quaterniond Euler2Quaternion(const Vector3d& rpy){
        Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(rpy(2),Eigen::Vector3d::UnitZ()));
        Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(rpy(1),Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(rpy(0),Eigen::Vector3d::UnitX()));

        return roll_angle*pitch_angle*yaw_angle;
    }

    Eigen::Matrix3d Euler2RotationMatrix(const Vector3d& rpy){
        return Quaternion2RotationMatrix(Euler2Quaternion(rpy));
    }

    Eigen::Vector3d RotationMatrix2Euler(const Matrix3d &m){
        return m.eulerAngles(2,1,0);
    }

    Eigen::Vector3d Quaternion2Euler(const Quaterniond& q){
        return RotationMatrix2Euler(q.toRotationMatrix());
    }

    Eigen::Quaterniond RotationVector2Quaternion(const Vector3d& rv){
        Eigen::Quaterniond qfromrv;
        Vector3d rv_2=rv*0.5;
        double norm=rv_2.norm();
        qfromrv.w()=cos(norm);
        qfromrv.vec()=norm<1E-8?rv_2:(sin(norm)/norm)*rv_2;
        return qfromrv;
    }

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

    cInsMech::cInsMech() {}

    cInsMech::~cInsMech() {}

    Eigen::Quaterniond cInsMech::AttitudeUpdate(PPPLib::tImuDataUnit &pre_imu_data,
                                                PPPLib::tImuDataUnit &cur_imu_data,
                                                const PPPLib::tSolInfoUnit &pre_sol_info) {
        double dt=cur_imu_data.t_tag_.TimeDiff(pre_imu_data.t_tag_.t_);
        Vector3d theta_k(cur_imu_data.gyro_),theta_k_1(pre_imu_data.gyro_);

        //等效旋转矢量
        Vector3d cur_phi=theta_k+VectorSkew(theta_k_1)*theta_k/12.0; //单子样+前一周期
        Quaterniond quat_bb=RotationVector2Quaternion(cur_phi);

        Vector3d wiee(0,0,OMGE_GPS);
        Vector3d zeta=wiee*dt;
        Quaterniond quat_ee=RotationVector2Quaternion(zeta).conjugate();
        Quaterniond quat_k_1=Euler2Quaternion(pre_sol_info.att);
        Quaterniond qbn_k=quat_ee*quat_k_1*quat_bb;

        return qbn_k.normalized();
    }

    Eigen::Vector3d cInsMech::VelocityUpdate(PPPLib::tImuDataUnit &pre_imu_data,
                                             PPPLib::tImuDataUnit &cur_imu_data,
                                             const PPPLib::tSolInfoUnit &pre_sol_info) {
        Vector3d pos=pre_sol_info.pos, vel=pre_sol_info.vel;
        double dt=cur_imu_data.t_tag_.TimeDiff(pre_imu_data.t_tag_.t_);
        Vector3d wiee(0,0,OMGE_GPS);
        Vector3d theta_k(cur_imu_data.gyro_),theta_k_1(pre_imu_data.gyro_);
        Vector3d vb_k(cur_imu_data.acce_),vb_k_1(pre_imu_data.acce_);

        Vector3d coord_blh=Xyz2Blh(pos);
        Matrix3d Cen=CalcCen(coord_blh,COORD_NED);
        Vector3d ge=Cen.transpose()*CalculateGravity(coord_blh,false);
        Vector3d omgea_n=wiee*2.0;
        Vector3d delta_gcor=(ge-omgea_n.cross(vel))*dt;

        Matrix3d Cee=Matrix3d::Identity()-VectorSkew(wiee*0.5*dt);

        Vector3d vrot=theta_k.cross(vb_k)*0.5;
        Vector3d vscul=(theta_k_1.cross(vb_k)+vb_k_1.cross(theta_k))/12.0;

        Quaterniond pre_quat=Euler2Quaternion(pre_sol_info.att);
        Matrix3d Cbe=pre_quat.toRotationMatrix();
        Vector3d delta_ve=Cee*Cbe*(vb_k+vrot+vscul);

        return (pre_sol_info.vel+delta_gcor+delta_ve);
    }

    Eigen::Vector3d cInsMech::PositionUpdate(const PPPLib::tSolInfoUnit &pre_sol_info,const Vector3d& cur_vel, double dt) {
        Eigen::Vector3d pos;
        pos=(pre_sol_info.vel+cur_vel)*0.5+pre_sol_info.pos;
    }

    tSolInfoUnit cInsMech::InsMechanization(tImuDataUnit &pre_imu_data, tImuDataUnit &cur_imu_data, const tSolInfoUnit &sol_info) {
        tSolInfoUnit sol=sol_info;

        sol.att=Quaternion2Euler(AttitudeUpdate(pre_imu_data,cur_imu_data,sol_info));
        sol.vel=VelocityUpdate(pre_imu_data,cur_imu_data,sol_info);
        sol.pos=PositionUpdate(sol_info,sol.vel,cur_imu_data.t_tag_.TimeDiff(pre_imu_data.t_tag_.t_));
        sol.t_tag=cur_imu_data.t_tag_;

        return sol;
    }

    Eigen::Vector3d CalculateGravity(const Vector3d coord_blh,bool is_ecef){
        if(is_ecef){
            const double constant_J2=0.00108263;
            const double constant_J4=-2.37091222e-6;
            const double constant_J6=6.08347e-9;
            double p = sqrt(coord_blh(0) * coord_blh(0) + coord_blh(1) * coord_blh(1) + coord_blh(2) * coord_blh(2));
            double t = coord_blh(2) / p;
            double a_p = WGS84_EARTH_LONG_RADIUS/ p;
            double a1 = -WGS84_GM / p / p;
            double a2 = 1 + 1.5 * constant_J2 * a_p * a_p - (15.0 / 8) * constant_J4 * pow(a_p,3) * a_p + (35.0 / 16) * constant_J6 * pow(a_p,3) * pow(a_p,3);
            double a3 = -4.5 * constant_J2 * a_p * a_p + (75.0 / 4) * constant_J4 * pow(a_p,3) * a_p - (735.0 / 16) * constant_J6 * pow(a_p,3) * pow(a_p,3);
            double a4 = -(175.0 / 8) * constant_J4 * pow(a_p,3) * a_p + (2205.0 / 16) * constant_J6 * pow(a_p,3) * pow(a_p,3);
            double a5 = -(1617.0 / 16) * constant_J6 * pow(a_p,3) * pow(a_p,3);

            double b1 = 3 * constant_J2 * a_p * a_p - (15.0 / 2) * constant_J4 * pow(a_p,3) * a_p + (105.0 / 8) * constant_J6 * pow(a_p,3) * pow(a_p,3);
            double b2 = (35.0 / 2) * constant_J4 * pow(a_p,3) * a_p - (945.0 / 12) * constant_J6 * pow(a_p,3) * pow(a_p,3);
            double b3 = (693.0 / 8) * constant_J6 * pow(a_p,3) * pow(a_p,3);

            double c1 = a2;
            double c2 = a3 - b1;
            double c3 = a4 - b2;
            double c4 = a5 - b3;
            double d1 = a2 + b1;
            double d2 = c2 + b2;
            double d3 = c3 + b3;
            double d4 = c4;
            Vector3d ge_vec;
            ge_vec(0) = (c1 + c2 * t * t + c3 * pow(t,3) * t + c4 * pow(t,3) * pow(t,3)) * coord_blh(0) * a1 / p + OMGE_GPS * OMGE_GPS * coord_blh(0);
            ge_vec(1) = (c1 + c2 * t * t + c3 * pow(t,3) * t + c4 * pow(t,3) * pow(t,3)) * coord_blh(1) * a1 / p + OMGE_GPS * OMGE_GPS * coord_blh(1);
            ge_vec(2) = (d1 + d2 * t * t + d3 * pow(t,3) * t + d4 * pow(t,3) * pow(t,3)) * coord_blh(2) * a1 / p;
            return ge_vec;
        }
        else{
            double gn = 9.7803267715 * (1 + 0.0052790414 * sin(coord_blh(0)) * sin(coord_blh(0)) + 0.0000232719 * pow(sin(coord_blh(0)),3) * sin(coord_blh(0)));
            gn += (-0.0000030876910891 + 0.0000000043977311 * sin(coord_blh(0)) * sin(coord_blh(0))) * coord_blh(2);
            gn += 0.0000000000007211 * coord_blh(2) * coord_blh(2);
            Vector3d gn_vec{0, 0, gn};
            return gn_vec;
        }
    }

}