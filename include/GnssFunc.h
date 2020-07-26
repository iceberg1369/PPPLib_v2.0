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
        int sys_idx;
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
    }tPreOrbUnit;

    typedef struct{
        cTime t_tag;
        double clk[MAX_SAT_NUM];
        float  std[MAX_SAT_NUM];
    }tPreClkUnit;

    typedef struct{
        double mjd;
        double xp,yp;
        double xpr,ypr;
        double ut1_utc;
        double lod;
    }tErpUnit;

    typedef struct{
        cSat  sat;
        cTime ts,te;
        string ant_type;
        string ser_code;
        Vector3d pco[MAX_GNSS_FRQ_NUM*NSYS];
        double   pcv[MAX_GNSS_FRQ_NUM*NSYS][80*30];
        double dazi;
        double zen1,zen2,dzen;
    }tAntUnit;

    typedef struct{
        cTime t_tag;
        int ndata[3]{};
        double re;
        double lats[3]{};
        double lons[3]{};
        double hgts[3]{};
        vector<double> data;
        vector<float>  rms;
    }tTecUnit;

    typedef struct{
        vector<tBrdEphUnit>    brd_eph;
        vector<tBrdGloEphUnit> brd_glo_eph;
        vector<tPreOrbUnit> pre_eph;
        vector<tPreClkUnit> pre_clk;
        vector<tErpUnit>erp_paras;
        vector<tAntUnit>ant_paras;
        vector<tTecUnit>tec_paras;

        int glo_frq_num[GLO_MAX_PRN+1];
        double glo_cp_bias[4];
        int leaps;
        double ion_para[NSYS][8];
        double utc_para[NSYS][4];
        double code_bias[MAX_SAT_NUM][MAX_GNSS_CODE_BIAS_PAIRS];
        double ocean_paras[2][6*11];
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
    }tStaInfoUnit;

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
        tStaInfoUnit* GetStation();

    public:
        int epoch_num;

    private:
        RECEIVER_INDEX rcv_idx_;
        cTime ts_,te_;
        vector<tEpochSatUnit> obs_;
        tStaInfoUnit station_;
    };

    typedef struct{
        cSat sat;
        cTime t_tag;
        GNSS_SAT_STAT stat;
        int brd_eph_index;

        unsigned char P_code[MAX_GNSS_USED_FRQ_NUM];
        double raw_P[MAX_GNSS_USED_FRQ_NUM];        // L1 L2 L5
        double raw_L[MAX_GNSS_USED_FRQ_NUM];
        double raw_D[MAX_GNSS_USED_FRQ_NUM];
        double raw_S[MAX_GNSS_USED_FRQ_NUM];
        double cor_P[MAX_GNSS_USED_FRQ_NUM];        // corrected code bias BDS satellite-specific multipath
        double cor_L[MAX_GNSS_USED_FRQ_NUM];        // corrected phase wind-up
        double cor_if_P[MAX_GNSS_USED_FRQ_NUM];     // L1L2 L1L5 L1L2L5
        double cor_if_L[MAX_GNSS_USED_FRQ_NUM];
        double lam[MAX_GNSS_USED_FRQ_NUM];
        double frq[MAX_GNSS_USED_FRQ_NUM];

        Vector3d brd_pos;
        Vector2d brd_clk;  // clk, clk-rate
        Vector3d pre_pos;
        Vector2d pre_clk;

        Vector2d trp_dry_delay; // slant_dry,map_dry
        Vector4d trp_wet_delay; // slant_wet,map_wet,grand_e,grand_n
        Vector2d ion_delay; // L1_slant_ion, map_ion;

        double code_bias[MAX_GNSS_USED_FRQ_NUM];
        double bd2_mp[3];
        double phase_wp;
        double float_amb[MAX_GNSS_USED_FRQ_NUM]; //L1,L2,L5

        double omc_P[MAX_GNSS_USED_FRQ_NUM];
        double omc_L[MAX_GNSS_USED_FRQ_NUM];

        double prior_res_P[MAX_GNSS_USED_FRQ_NUM];
        double prior_res_L[MAX_GNSS_USED_FRQ_NUM];
        double post_res_P[MAX_GNSS_USED_FRQ_NUM];
        double post_res_L[MAX_GNSS_USED_FRQ_NUM];

        Vector2d el_az;
        double if_amb[MAX_GNSS_USED_FRQ_NUM];      //L1_l2, L1_L5, L1_L2_L5
        Vector2d raw_mw_amb;  //L1_L2, L1_L5
        Vector2d sm_mw_amb;   //L1_L2, L1_L5
        Vector2d gf;          //L1_L2, L1_L5
        Vector2d multipath_comb;
    }tSatInfoUnit;

}



#endif //PPPLIB_GNSSFUNC_H
