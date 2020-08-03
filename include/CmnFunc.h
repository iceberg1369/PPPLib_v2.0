//
// Created by cc on 7/9/20.
//

#ifndef PPPLIB_CMNFUNC_H
#define PPPLIB_CMNFUNC_H

#include <Eigen/Dense>
#include <iomanip>
#include "LogInfo.h"
#include "Constant.h"

using namespace std;
using namespace Eigen;

namespace PPPLib{

    int Round(double d);
    template <typename Iter1,typename Iter2>
    double Dot(const Iter1 VecA,const Iter2 VecB,int SizeVec){
        double dInn=0.0;

        while (--SizeVec>=0){
            dInn+=VecA[SizeVec]*VecB[SizeVec];
        }
        return dInn;
    }
    template <typename Iter>
    double Norm(const Iter VecA,int SizeVec){
        return sqrt(Dot(VecA,VecA,SizeVec));
    }

    template <typename Iter1,typename Iter2>
    int NormV3(const Iter1 vec1,Iter2 vec2){
        double r;
        if((r=Norm(vec1,3))<=0.0) return 0;
        vec2[0]=vec1[0]/r;
        vec2[1]=vec1[1]/r;
        vec2[2]=vec1[2]/r;
        return 1;
    }

    template <typename Iter1,typename Iter2, typename Iter3>
    void CrossVec3(const Iter1 vec1,const Iter2 vec2,Iter3 vec3){
        vec3[0]=vec1[1]*vec2[2]-vec1[2]*vec2[1];
        vec3[1]=vec1[2]*vec2[0]-vec1[0]*vec2[2];
        vec3[2]=vec1[0]*vec2[1]-vec1[1]*vec2[0];
    }

    Eigen::Matrix3d VectorSkew(const Eigen::Vector3d& vec);


    string Doul2Str(int str_len, int dec_len, const string str_filler, const double src_num, string &dst_str);
    string Int2Str(int str_len, const string str_filler, const int src_num, string &dst_str);
    int Str2Double(string src_str,double &dst_num);
    int Str2Int(const string src_str,int &dst_num);
    string StrTrim(string s);

    typedef struct {
        time_t long_time;
        double sec;
    }tTime;

    typedef struct {
        long sn;
        double tos;
    }tSod;

    typedef struct {
        long day;
        tSod sod;
    }tMjd;

    class cTime {
    public:
        cTime();
        cTime(const double *ep);
        cTime(string str_time);
        cTime operator=(const tTime t);
        cTime operator+(double sec);
        void operator+=(double sec);
        ~cTime();

    public:
        double* GetEpoch();
        string GetTimeStr(int n);
        tMjd* GetMjd();
        int GetDoy();
        double TimeDiff(tTime t1);
        int Str2Time(string s);

        cTime *Epoch2Time(const double *ep);
        void Time2Epoch();
        double Time2Gpst(int* week,int* day, int sys);
        cTime* Gpst2Time(int week, double wos, int sys);
        cTime Utc2Gpst();
        cTime Gpst2Utc();
        cTime Gpst2Bdst();
        cTime Bdst2Gpst();
        double Utc2Gmst(double ut1_utc);
        cTime* AdjWeek(cTime t);
        cTime* AdjDay(cTime t);

    private:

        string Time2Str(int n);
        double Time2Doy();
        void Time2Mjd();
        double Time2Sec(cTime& day);

    private:
        double epoch_[6];
        string time_str_;
        int doy_;
        tMjd  mjd_;

    public:
        tTime t_;
    };

#if 0
    class cCoord{
    public:
        cCoord();
        cCoord(const Vector3d& coord, const COORDINATE_TYPE coord_type);
        cCoord(const double *coord, const COORDINATE_TYPE coord_type);
        ~cCoord();

    public:
        Vector3d GetCoordEnu(cCoord ref_coord);
        Vector3d GetCoordXyz();
        Vector3d GetCoordBlh();
        Vector3d GetCoordNed(cCoord ref_coord);
        Matrix3d GetCne(const COORDINATE_TYPE type);

    private:
        void Xyz2Blh();
        void Blh2Xyz();
        void Xyz2Enu(cCoord ref_coord);
        void Enu2Xyz(cCoord ref_coord);
        void Enu2Ned();
        void Ned2Enu();
        void CalcCne(const COORDINATE_TYPE type);
        double CalcLat();

    private:
        Vector3d coord_XYZ_;
        Vector3d coord_BLH_;
        Vector3d coord_ENU_;
        Vector3d coord_NED_;
        Matrix3d Cne_;
    };
#endif
    Vector3d Xyz2Blh(Vector3d& coord_xyz);
    Vector3d Blh2Xyz(Vector3d& coord_blh);
    Vector3d Xyz2Enu(Vector3d& coord_blh,Vector3d& coord_xyz);
    Vector3d Enu2Xyz(Vector3d& coord_blh,Vector3d& coord_enu);
    Vector3d Enu2Ned(Vector3d& coord_enu);
    Vector3d Ned2Enu(Vector3d& coord_ned);
    Matrix3d CalcCen(Vector3d& coord_blh,COORDINATE_TYPE lf_type);

    typedef struct{
        int nav_sys;
        double ele_min;
        GNSS_FRQ_OPT frq_opt;
        int gnss_frq[NSYS+1][MAX_GNSS_USED_FRQ_NUM];
        bool csc;
        GNSS_AC_OPT  ac_opt;
        GNSS_EPH_OPT eph_opt;
        GNSS_ION_OPT ion_opt;
        GNSS_TRP_OPT trp_opt;
        GNSS_TID_OPT tid_opt;
        double cs_thres[2];   //mw and gf
    }tGnssConf;

    typedef struct{
        FUSION_GNSS_MODE gnss_opt;

    }tInsConf;

    typedef struct{
        string rover;
        string base;
        string brd;
        string cbias;
        string clk;
        string sp3[3];
        string erp;
        string atx;
        string gim;
        string blq;
        string imu;
        string ref;
        string sol;
    }tFileConf;

    typedef struct{
        bool out_err;
        COORDINATE_TYPE sol_coord;
    }tSolConf;

    typedef struct{
        PPPLIB_MODE mode;
        PPPLIB_MODE_OPT mode_opt;
        int dynamic;
        SOLVE_ESTIMATOR estimator;
        tGnssConf gnssC;
        tInsConf  insC;
        tFileConf fileC;
        tSolConf  solC;
    }tPPPLibConf;

    typedef struct {
        cTime t_tag;
        SOL_STAT stat;
        Vector3d pos;
        Vector3d vel;
        double clk_error[NSYS];
        double rec_dcb[NSYS];
        double rec_ifb[NSYS];
        Vector2d zenith_trp_delay; // dry and wet

        Vector3d att{0,0,0};  //roll pitch yaw
        Vector3d gyro_bias{0,0,0};
        Vector3d accl_bias{0,0,0};
    }tSolInfoUnit;

    class cParSetting{
    public:
        cParSetting();
        cParSetting(tPPPLibConf conf);
        ~cParSetting();

    public:
        int GetGnssUsedFrqs();
        int GetSppParNum();
        int GetPppParNum();
        int GetDgnssParNum();
        int GetPpkParNum();

        int PvaParNum();
        int RecClkParNum();
        int RecDcbParNum();
        int RecIfbParNum();
        int GloIfcbParNum();
        int TrpParNum();
        int IonParNum();
        int AmbParNum();

        int ParIndexPva(int i);
        int ParIndexClk(int sys_index);
        int ParIndexDcb(int sys_index);
        int ParIndexIfb(int sys_index);
        int ParIndexGloIfcb();
        int ParIndexTrp();
        int ParIndexIon(int sat_no);
        int ParIndexAmb(int f,int sat_no);

    public:
        tPPPLibConf PPPLibC_;
    };


}


#endif //PPPLIB_CMNFUNC_H
