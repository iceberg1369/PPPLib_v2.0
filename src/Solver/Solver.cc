//
// Created by cc on 7/23/20.
//

#include "Solver.h"

namespace PPPLib{

    cSolver::cSolver() {}

    cSolver::~cSolver() {}

    void cSolver::InitSolver() {}

    int cSolver::GnssObsRes(int post, tPPPLibConf C) {}

    void cSolver::MakeObsComb(tPPPLibConf C) {
        tSatInfoUnit *sat_info= nullptr;

        //if
        double alpha=0.0,beta=0.0;
        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);
            int sys=sat_info->sat.sat_.sys;
            if(C.gnssC.frq_opt==FRQ_SINGLE){
                sat_info->cor_if_P[0]=0.5*sat_info->cor_P[0]+0.5*sat_info->cor_L[0];
            }
            else if(C.gnssC.frq_opt==FRQ_DUAL){
                alpha=SQR(sat_info->frq[0])/(SQR(sat_info->frq[0])-SQR(sat_info->frq[1]));
                beta=-SQR(sat_info->frq[1])/(SQR(sat_info->frq[0])-SQR(sat_info->frq[1]));
                sat_info->cor_if_P[0]=alpha*sat_info->cor_P[0]+beta*sat_info->cor_P[1];
                sat_info->cor_if_L[0]=alpha*sat_info->cor_L[0]+beta*sat_info->cor_L[1];
            }
            else if(C.gnssC.frq_opt==FRQ_TRIPLE){

            }
        }

        //gf

        //mw

        // multipath combination
    }

    void cSolver::OutGnssObs(tSatInfoUnit sat_info, int f) {
        cout<<sat_info.t_tag.GetTimeStr(1)<<endl;
        cout<<sat_info.sat.sat_.id<<" ";
        cout<<fixed<<setprecision(3)<<sat_info.raw_P[0]<<" "<<sat_info.raw_P[1]<<" "<<sat_info.raw_P[2]<<endl;
        cout<<"    ";
        cout<<fixed<<setprecision(3)<<sat_info.raw_L[0]<<" "<<sat_info.raw_L[1]<<" "<<sat_info.raw_L[2]<<endl;
        cout<<"    ";
        cout<<fixed<<setprecision(3)<<sat_info.frq[0]<<" "<<sat_info.frq[1]<<" "<<sat_info.frq[2]<<endl;
    }

    void cSolver::ReAlignObs(tSatInfoUnit &sat_info, tSatObsUnit sat_obs,int f,int frq_idx) {
        sat_info.P_code[f]=sat_obs.code[frq_idx];
        sat_info.raw_P[f]=sat_obs.P[frq_idx];
        sat_info.raw_L[f]=sat_obs.L[frq_idx];
        sat_info.raw_D[f]=sat_obs.D[frq_idx];
        sat_info.raw_S[f]=sat_obs.SNR[frq_idx];
        sat_info.lam[f]=CLIGHT/kGnssFreqs[sat_obs.sat.sat_.sys_idx][frq_idx];
        sat_info.frq[f]=kGnssFreqs[sat_obs.sat.sat_.sys_idx][frq_idx];
    }

    void cSolver::UpdateGnssObs(tPPPLibConf C) {
        int sys,i;
        int *frqs;
        for(i=0;i<epoch_sat_obs_.sat_num;i++){
            tSatInfoUnit sat_info={0};
            sys=epoch_sat_obs_.epoch_data.at(i).sat.sat_.sys;
            sat_info.t_tag=epoch_sat_obs_.obs_time;
            sat_info.sat=epoch_sat_obs_.epoch_data.at(i).sat;
            sat_info.stat=SAT_USED;

            switch(sys){
                case SYS_BDS:
                    if(sat_info.sat.sat_.prn>18){
                        frqs=C.gnssC.gnss_frq[NSYS];break;
                    }
                    frqs=C.gnssC.gnss_frq[SYS_INDEX_BDS];break;
                case SYS_GAL: frqs=C.gnssC.gnss_frq[SYS_INDEX_GAL];break;
                case SYS_GLO: frqs=C.gnssC.gnss_frq[SYS_INDEX_GLO];break;
                case SYS_QZS: frqs=C.gnssC.gnss_frq[SYS_INDEX_QZS];break;
                default:
                    frqs=C.gnssC.gnss_frq[SYS_INDEX_GPS];break;
            }
            ReAlignObs(sat_info,epoch_sat_obs_.epoch_data.at(i),0,frqs[0]);
            if(C.gnssC.frq_opt==FRQ_DUAL){
                ReAlignObs(sat_info,epoch_sat_obs_.epoch_data.at(i),1,frqs[1]);
            }
            else if(C.gnssC.frq_opt==FRQ_TRIPLE){
                ReAlignObs(sat_info,epoch_sat_obs_.epoch_data.at(i),1,frqs[1]);
                ReAlignObs(sat_info,epoch_sat_obs_.epoch_data.at(i),2,frqs[2]);
            }

            epoch_sat_info_collect_.push_back(sat_info);
//            OutGnssObs(sat_info,0);
        }
    }

    void cSolver::GnssErrCorr(Vector3d blh,tSatInfoUnit& sat_info) {}

    bool cSolver::SolverProcess() {}

    bool cSolver::Estimator(tPPPLibConf C) {}

    bool cSolver::SolutionUpdate() {}

    cSppSolver::cSppSolver() {
        spp_para_=cParSetting(spp_conf_);
        num_full_x_=spp_para_.GetSppParNum();
        full_x_.resize(num_full_x_,0.0);
        full_Px.resize(num_full_x_*num_full_x_,0.0);
    }

    cSppSolver::cSppSolver(tPPPLibConf conf) {
        spp_conf_=conf;
        spp_para_=cParSetting(spp_conf_);
        num_full_x_=spp_para_.GetSppParNum();
        full_x_.resize(num_full_x_,0.0);
        full_Px.resize(num_full_x_*num_full_x_,0.0);
    }

    cSppSolver::~cSppSolver() {}

    void cSppSolver::GnssErrCorr(Vector3d blh,tSatInfoUnit& sat_info) {

    }

    int cSppSolver::GnssObsRes(int post, tPPPLibConf C) {
        tSatInfoUnit* sat_info= nullptr;
        Vector3d rover_xyz(5084655,2670323,-2768479);
        cCoord rr(rover_xyz,COORD_XYZ);
        Vector3d rover_blh=rr.GetCoordBlh();

        int num_used_frq=spp_para_.GetGnssUsedFrqs();
        if(num_used_frq<=0) return 0;

        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);
            if(sat_info->stat!=SAT_USED) continue;

            if(sat_info->sat.sat_.sys==SYS_BDS&&sat_info->sat.sat_.prn<19){
                spp_err_corr_.BD2MultipathModel(sat_info);
            }

            sat_info->el_az[0]=10*D2R;sat_info->el_az[1]=10*D2R;
            spp_err_corr_.ion_model_.InitSatInfo(sat_info,&rover_blh);
            spp_err_corr_.ion_model_.GetKlobIon();
            spp_err_corr_.ion_model_.UpdateSatInfo();
            spp_err_corr_.trp_model_.InitSatInfo(sat_info,&rover_blh);
            spp_err_corr_.trp_model_.GetSaasTrp(0.0, nullptr, nullptr);
            spp_err_corr_.trp_model_.UpdateSatInfo();

            for(int f=0;f<num_used_frq;f++){


            }
        }
    }

    void cSppSolver::CorrGnssObs() {
        tSatInfoUnit *sat_info= nullptr;
        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);
            spp_err_corr_.cbia_model_.InitSatInfo(sat_info, nullptr);
            spp_err_corr_.cbia_model_.GetCodeBias();
            spp_err_corr_.cbia_model_.UpdateSatInfo();

            for(int j=0;j<MAX_GNSS_USED_FRQ_NUM;j++){
                sat_info->cor_P[j]=sat_info->raw_P[j]-sat_info->code_bias[j];
                sat_info->cor_L[j]=sat_info->raw_L[j]*sat_info->lam[j];
            }

            if(sat_info->sat.sat_.sys==SYS_BDS&&sat_info->sat.sat_.prn<=18){
                spp_err_corr_.BD2MultipathModel(sat_info);
                for(int j=0;j<3;j++) sat_info->cor_P[j]-=sat_info->bd2_mp[j];
            }
        }
    }

    bool cSppSolver::Estimator(tPPPLibConf C) {
        int valid_sat_num=0;
        spp_sols_.stat=SOL_NONE;

        vector<double> x(full_x_),Px(full_x_);

        for(int i=0;i<iter_;i++){

            valid_sat_num=GnssObsRes(0,spp_conf_);
            if(valid_sat_num<4){
                return false;
            }

        }
    }

    void cSppSolver::InitSolver() {
        spp_err_corr_.cbia_model_.InitErrModel(spp_conf_,nav_);
    }

    bool cSppSolver::SolverProcess() {
        spp_err_corr_.InitGnssErrCorr(spp_conf_,nav_);

        for(int i=0;i<rover_obs_.epoch_num;i++){
            epoch_sat_obs_=rover_obs_.GetGnssObs().at(i);
            UpdateGnssObs(spp_conf_);
            CorrGnssObs();
            MakeObsComb(spp_conf_);
            Estimator(spp_conf_);
            SolutionUpdate();
        }

    }

    bool cSppSolver::SolutionUpdate() {
        int sat_no=0;

        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_no=epoch_sat_info_collect_.at(i).sat.sat_.no;
            previous_sat_info_[sat_no-1]=epoch_sat_info_collect_.at(i);
        }

        sol_collect_.push_back(spp_sols_);

        epoch_sat_info_collect_.clear();
    }

}