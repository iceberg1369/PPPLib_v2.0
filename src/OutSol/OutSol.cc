//
// Created by cc on 8/3/20.
//

#include "OutSol.h"

namespace PPPLib{

    cOutSol::cOutSol() {}

    cOutSol::cOutSol(PPPLib::tPPPLibConf C) {
        C_=C;
        plot_sol_=new cPlotSol;
        plot_sat_=new cPlotSat;
    }

    cOutSol::cOutSol(PPPLib::tPPPLibConf C,vector<tSolInfoUnit>& ref_sols) {C_=C;ref_sols_=ref_sols;}

    cOutSol::~cOutSol() {}

    int cOutSol::MatchRefSol(PPPLib::cTime sol_time) {
        double sow1,sow2;
        int i=0;
        bool stat=false;

        for(i=ref_index_-100<0?0:ref_index_-10;i<ref_sols_.size();i++){
            sow1=ref_sols_.at(i).t_tag.Time2Gpst(nullptr, nullptr,SYS_GPS);
            sow2=sol_time.Time2Gpst(nullptr, nullptr,SYS_GPS);
            if(fabs(sow1-sow2)<DTTOL){
                ref_index_=i;stat=true;break;
            }
            else if((sow1-sow2)>2.0*DTTOL){
                stat=false;break;
            }
        }
        return stat;
    }

    tSolInfoUnit cOutSol::CompareSol(PPPLib::tSolInfoUnit &sol, PPPLib::tSolInfoUnit &ref_sol) {
        tSolInfoUnit dsol;
        Vector3d ref_blh=Xyz2Blh(ref_sol.pos);
        Vector3d dxyz=sol.pos-ref_sol.pos;
        if(C_.solC.sol_coord==COORD_ENU){
            dsol.pos=Xyz2Enu(ref_blh,dxyz);
        }
        dsol.vel=sol.vel+ref_sol.vel;


        return dsol;
    }

    void cOutSol::WriteSol(tSolInfoUnit sol,tSatInfoUnit sat_info,int epoch) {
        tSolInfoUnit dsol;
        if(C_.solC.out_err){
            if(C_.mode_opt==MODE_OPT_STATIC||C_.mode_opt==MODE_OPT_KINE_SIM){
                ref_index_=0;
            }
            else{
                MatchRefSol(sol.t_tag);
            }
            dsol=CompareSol(sol,ref_sols_.at(ref_index_));
        }
        else dsol=sol;
//        plot_sol_->PlotSolPos(dsol,epoch);
        plot_sat_->PlotMwAmb(sat_info,epoch);
//        plot_sat_->PlotGfComb(sat_info,epoch);
    }

}