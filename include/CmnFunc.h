//
// Created by cc on 7/9/20.
//

#ifndef PPPLIB_CMNFUNC_H
#define PPPLIB_CMNFUNC_H

#include <Eigen/Dense>
#include "LogInfo.h"
#include "Constant.h"

using namespace std;
using namespace Eigen;

namespace PPPLib{

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
        cTime operator+(double sec) const;
        void operator+=(double sec);
        ~cTime();

    public:
        double* GetEpoch();
        string GetTimeStr(int n);
        tMjd* GetMjd();
        int GetDoy();
        double TimeDiff(tTime t1);
        int Str2Time(string s);

        double Time2Gpst(int* week,int* day, int sys);
        cTime* Gpst2Time(int week, double wos, int sys);
        cTime Utc2Gpst();
        cTime Gpst2Utc();
        cTime Gpst2Bdst() const;
        cTime Bdst2Gpst() const;
        double Utc2Gmst(double ut1_utc);
        cTime* AdjWeek(cTime t);
        cTime* AdjDay(cTime t);

    private:
        cTime *Epoch2Time(const double *ep);
        void Time2Epoch();

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
        Vector3d coord_ENU_;
        Vector3d coord_BLH_;
        Vector3d coord_NED_;
        Matrix3d Cne_;
    };
}


#endif //PPPLIB_CMNFUNC_H
