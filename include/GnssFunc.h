//
// Created by cc on 7/16/20.
//

#ifndef PPPLIB_GNSSFUNC_H
#define PPPLIB_GNSSFUNC_H

#include "CmnFunc.h"

namespace PPPLib{

    typedef struct{
        int no;
        int sys;
        int prn;
        string id;
    }tSat;

    class cSat{
    public:
        cSat();
        cSat(int sat_no);
        cSat(int sat_sys,int sat_prn);
        cSat(string sat_id);
        ~cSat();

    public:
        void SatPrn2No();
        void SatNo2Prn();
        void SatId2No();
        void SatNo2Id();

    public:
        tSat sat_={0};
    };

    typedef struct{
        cSat sat;
        int iode,iodc;
        int sva,svh;
        int week,code;
        cTime toe,toc,ttr;
        double A,e,i0,Omg0,omg,M0,deln,Omgd,idot;
        double crc,crs,cuc,cus,cic,cis;
        double toes;
        double f0,f1,f2;
        Vector4d tgd;
        double Adot,ndot;
    }tBrdEphUnit;

    typedef struct{
        cSat sat;
        int iode,frq;
        int svh,sva,age;
        cTime toe,tof;
        double taun,gamn,dtaun;
        Vector3d pos;
        Vector3d vel;
        Vector3d acc;
    }tBrdGloEphUnit;

    typedef struct{
        cTime t_tag;
        Vector4d pos[MAX_SAT_NUM];
        Vector4d vel[MAX_SAT_NUM];
        Vector4d std_pos[MAX_SAT_NUM];
        Vector4d std_vel[MAX_SAT_NUM];
        double clk[MAX_SAT_NUM];
        double std_clk[MAX_SAT_NUM];
    }tPreEphUnit;


    typedef struct{
        vector<tBrdEphUnit> brd_eph;
        vector<tPreEphUnit> pre_eph;
        vector<tBrdGloEphUnit> brd_glo_eph;
        int glo_frq_num[GLO_MAX_PRN+1];
        double glo_cp_bias[4];
        int leaps;
        double ion_para[NSYS][8];
        double utc_para[NSYS][4];
    }tNav;

    typedef struct {
        string name;
        string marker;
        string ant_desc;
        string ant_seri;
        string rec_type;
        string firm_ver;
        Vector3d del;
        Vector3d apr_pos;
        double ant_hgt;
    }tSta;

    typedef struct{
        cSat sat;
        double P[GNSS_NUM_FREQ+GNSS_NUM_EXOBS];
        unsigned char code[GNSS_NUM_FREQ+GNSS_NUM_EXOBS];
        double L[GNSS_NUM_FREQ+GNSS_NUM_EXOBS];
        float D[GNSS_NUM_FREQ+GNSS_NUM_EXOBS];
        unsigned char SNR[GNSS_NUM_FREQ+GNSS_NUM_EXOBS];
        unsigned char LLI[GNSS_NUM_FREQ+GNSS_NUM_EXOBS];
        double frq[GNSS_NUM_FREQ+GNSS_NUM_EXOBS];
    }tSatObsUnit;

    typedef struct{
        cTime obs_time;
        int sat_num;
        vector<tSatObsUnit> epoch_data;
    }tEpochSatUnit;

    class cGnssObs{
    public:
        cGnssObs();
        ~cGnssObs();

    public:
        void SetTimeSpan(cTime* ts, cTime* te);
        cTime* GetStartTime();
        cTime* GetEndTime();
        void SetRcvIdx(RECEIVER_INDEX rcv);
        vector<tEpochSatUnit>& GetGnssObs();
        tSta* GetStation();
        int* GetEpochNum();

    private:
        RECEIVER_INDEX rcv_idx_;
        cTime ts_,te_;
        int epoch_num;
        vector<tEpochSatUnit> obs_;
        tSta station_;
    };

}



#endif //PPPLIB_GNSSFUNC_H
