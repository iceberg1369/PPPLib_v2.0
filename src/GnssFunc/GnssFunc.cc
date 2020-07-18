//
// Created by cc on 7/16/20.
//

#include "GnssFunc.h"

namespace PPPLib{
    cSat::cSat() {}

    cSat::cSat(int sat_no) {sat_.no=sat_no;}

    cSat::cSat(string sat_id) {sat_.id=sat_id;}

    cSat::cSat(int sat_sys,int sat_prn){sat_.sys=sat_sys;sat_.prn=sat_prn;}

    cSat::~cSat() {}

    void cSat::SatPrn2No() {
        if(sat_.prn<=0){
            LOG(ERROR)<<"satellite prn error";
            return;
        }

        switch(sat_.sys){
            case SYS_GPS:
                if(sat_.prn<GPS_MIN_PRN||GPS_MAX_PRN<sat_.prn){sat_.no=0;return;}
                sat_.no=sat_.prn-GPS_MIN_PRN+1;break;
            case SYS_BDS:
                if(sat_.prn<BDS_MIN_PRN||BDS_MAX_PRN<sat_.prn){sat_.no=0;return;}
                sat_.no=NUM_GPS_SAT+sat_.prn-BDS_MIN_PRN+1;break;
            case SYS_GAL:
                if(sat_.prn<GAL_MIN_PRN||GAL_MAX_PRN<sat_.prn){sat_.no=0;return;}
                sat_.no=NUM_GPS_SAT+NUM_BDS_SAT+sat_.prn-GAL_MIN_PRN+1;break;
            case SYS_GLO:
                if(sat_.prn<GLO_MIN_PRN||GLO_MAX_PRN<sat_.prn){sat_.no=0;return;}
                sat_.no=NUM_GPS_SAT+NUM_BDS_SAT+NUM_GAL_SAT+sat_.prn-GLO_MIN_PRN+1;break;
            case SYS_QZS:
                if(sat_.prn<QZS_MIN_PRN||QZS_MAX_PRN<sat_.prn){sat_.no=0;return;}
                sat_.no=NUM_GPS_SAT+NUM_BDS_SAT+NUM_GAL_SAT+NUM_GLO_SAT+sat_.prn-QZS_MIN_PRN+1;break;
            case SYS_IRN:
                sat_.no=0;break;
            default:
                sat_.no=0;break;
        }
    }

    void cSat::SatNo2Prn() {
        int prn=sat_.no;
        sat_.sys=SYS_NONE;
        if(sat_.no<=0||MAX_SAT_NUM<sat_.no){
            LOG(ERROR)<<"satellite no. error";
            return;
        }
        else if(prn<=NUM_GPS_SAT){
            sat_.sys=SYS_GPS;prn+=GPS_MIN_PRN-1;
        }
        else if((prn-=NUM_GPS_SAT)<=NUM_BDS_SAT){
            sat_.sys=SYS_BDS;prn+=BDS_MIN_PRN-1;
        }
        else if((prn-=NUM_BDS_SAT)<=NUM_GAL_SAT){
            sat_.sys=SYS_GAL;prn+=GAL_MIN_PRN-1;
        }
        else if((prn-=NUM_GAL_SAT)<=NUM_GLO_SAT){
            sat_.sys=SYS_GLO;prn+=GLO_MIN_PRN-1;
        }
        else if((prn-=NUM_GLO_SAT)<=NUM_QZS_SAT){
            sat_.sys=SYS_QZS;prn+=QZS_MIN_PRN-1;
        }
        else if((prn-=NUM_QZS_SAT)<=NUM_IRN_SAT){
            sat_.sys=SYS_IRN;prn=0;
        }
        else prn=0;
        sat_.prn=prn;
    }

    void cSat::SatNo2Id() {
        SatNo2Prn();
        string buff;
        switch (sat_.sys){
            case SYS_GPS: sat_.id="G"+Int2Str(2,"0",sat_.prn-GPS_MIN_PRN+1,buff); break;
            case SYS_BDS: sat_.id="C"+Int2Str(2,"0",sat_.prn-BDS_MIN_PRN+1,buff); break;
            case SYS_GAL: sat_.id="E"+Int2Str(2,"0",sat_.prn-GAL_MIN_PRN+1,buff); break;
            case SYS_GLO: sat_.id="R"+Int2Str(2,"0",sat_.prn-GLO_MIN_PRN+1,buff); break;
            case SYS_QZS: sat_.id="J"+Int2Str(2,"0",sat_.prn-QZS_MIN_PRN+1,buff); break;
            default:
                sat_.id="";break;
        }
    }

    void cSat::SatId2No() {
        sat_.sys=SYS_NONE;

        if(Str2Int((sat_.id.substr(1,2)),sat_.prn)==0) return;
        char code= sat_.id[0];

        switch(code){
            case 'G': sat_.sys=SYS_GPS;sat_.prn+=GPS_MIN_PRN-1;break;
            case 'C': sat_.sys=SYS_BDS;sat_.prn+=BDS_MIN_PRN-1;break;
            case 'E': sat_.sys=SYS_GAL;sat_.prn+=GAL_MIN_PRN-1;break;
            case 'R': sat_.sys=SYS_GLO;sat_.prn+=GLO_MIN_PRN-1;break;
            case 'J': sat_.sys=SYS_QZS;sat_.prn+=QZS_MIN_PRN-1;break;
            default:
                sat_.sys=SYS_NONE;sat_.prn=0;break;
        }
        SatPrn2No();
    }

    cGnssObs::cGnssObs() {rcv_idx_=REC_ROVER;}

    cGnssObs::~cGnssObs() {}

    void cGnssObs::SetTimeSpan(cTime *ts, cTime *te) {
        if(ts) ts_=*ts;
        if(te) te_=*te;
    }

    cTime* cGnssObs::GetStartTime() { return &ts_;}

    cTime * cGnssObs::GetEndTime() {return &te_;}

    void cGnssObs::SetRcvIdx(RECEIVER_INDEX rcv) {rcv_idx_=rcv;}

    vector<tEpochSatUnit>& cGnssObs::GetGnssObs() {return obs_;}

    tSta* cGnssObs::GetStation() {return &station_;}

    int* cGnssObs::GetEpochNum() {return &epoch_num;}

}