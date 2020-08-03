//
// Created by cc on 7/23/20.
//

#include "Solver.h"
#include "ReadFiles.h"

namespace PPPLib{
    cSolver::cSolver() {
        epoch_idx_=0;
    }

    cSolver::~cSolver() {}

    int cSolver::GnssObsRes(int post, tPPPLibConf C, double *x) {

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

            gnss_obs_operator_.ReAlignObs(C,sat_info,epoch_sat_obs_.epoch_data.at(i),0,frqs[0]);
            if(C.gnssC.frq_opt==FRQ_DUAL){
                gnss_obs_operator_.ReAlignObs(C,sat_info,epoch_sat_obs_.epoch_data.at(i),1,frqs[1]);
            }
            else if(C.gnssC.frq_opt==FRQ_TRIPLE){
                gnss_obs_operator_.ReAlignObs(C,sat_info,epoch_sat_obs_.epoch_data.at(i),1,frqs[1]);
                gnss_obs_operator_.ReAlignObs(C,sat_info,epoch_sat_obs_.epoch_data.at(i),2,frqs[2]);
            }

            if(sat_info.stat==SAT_NO_USE) continue;

            epoch_sat_info_collect_.push_back(sat_info);
        }
    }

    void cSolver::InitSolver(tPPPLibConf C) {
        para_=cParSetting(C);
    }

    bool cSolver::SolverProcess(tPPPLibConf C) {}

    bool cSolver::SolverEpoch() {}

    bool cSolver::Estimator(tPPPLibConf C) {}

    bool cSolver::SolutionUpdate() {}

    cSppSolver::cSppSolver() {
        spp_conf_.mode=MODE_SPP;
        spp_conf_.gnssC.ele_min=10.0;
        spp_conf_.gnssC.frq_opt=FRQ_SINGLE;
        spp_conf_.gnssC.trp_opt=TRP_SAAS;
        spp_conf_.gnssC.ion_opt=ION_KLB;
        spp_conf_.estimator=SOLVE_LSQ;

        para_=cParSetting();
        num_full_x_=para_.GetSppParNum();
        full_x_=VectorXd::Zero(num_full_x_);
        full_Px_=MatrixXd::Zero(num_full_x_,num_full_x_);
    }

    cSppSolver::cSppSolver(tPPPLibConf conf) {
        spp_conf_=conf;
        para_=cParSetting(spp_conf_);
        num_full_x_=para_.GetSppParNum();
        full_x_=VectorXd::Zero(num_full_x_);
        full_Px_=MatrixXd::Zero(num_full_x_,num_full_x_);
    }

    cSppSolver::~cSppSolver() {}

    void cSppSolver::CorrGnssObs() {
        tSatInfoUnit *sat_info= nullptr;
        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);
            gnss_err_corr_.cbia_model_.InitSatInfo(sat_info, nullptr);
            gnss_err_corr_.cbia_model_.GetCodeBias();
            gnss_err_corr_.cbia_model_.UpdateSatInfo();

            for(int j=0;j<MAX_GNSS_USED_FRQ_NUM;j++){
                sat_info->cor_P[j]=sat_info->raw_P[j]-sat_info->code_bias[j];
                sat_info->cor_L[j]=sat_info->raw_L[j]*sat_info->lam[j];
            }

            if(sat_info->sat.sat_.sys==SYS_BDS&&sat_info->sat.sat_.prn<=18){
                gnss_err_corr_.BD2MultipathModel(sat_info);
                for(int j=0;j<3;j++) sat_info->cor_P[j]-=sat_info->bd2_mp[j];
            }
            if(spp_conf_.gnssC.ion_opt==ION_IF)
                gnss_obs_operator_.MakeGnssObsComb(spp_conf_,COMB_IF,sat_info,previous_sat_info_[sat_info->sat.sat_.no-1]);
        }
    }

    int cSppSolver::GnssObsRes(int post, tPPPLibConf C,double* x) {
        num_L_=0;

        vector<double>H,omcs,meas_var_vec;
        int sys,sys_mask[NSYS]={0};
        double omc,r,meas_var;
        tSatInfoUnit* sat_info= nullptr;
        Vector3d rover_xyz(x[0],x[1],x[2]);
        Vector3d rover_blh=Xyz2Blh(rover_xyz),sig_vec;

        int num_used_frq=para_.GetGnssUsedFrqs();
        if(num_used_frq<=0) return 0;

        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);
            if(sat_info->sat.sat_.no==4) continue;
            if(sat_info->sat.sat_.no==39) continue;
            if(sat_info->sat.sat_.no==50) continue;
            LOG_IF(i==0,DEBUG)<<"SINGLE POINT POSITION RESIDUAL"<<"("<<epoch_idx_<<")"<<": "<<sat_info->t_tag.GetTimeStr(1)
                                       <<setprecision(12)<<" "<<"X="<<rover_xyz[0]<<" "<<"Y="<<rover_xyz[1]<<" "<<"Z="<<rover_xyz[2];

            sys=sat_info->sat.sat_.sys;
            if(sat_info->stat!=SAT_USED) continue;

            //SF_IF: cor_if_P[0]
            //DF_IF: cor_if_P[0]
            for(int f=0;f<num_used_frq;f++){
                double cor_P=0.0;
                if(spp_conf_.gnssC.ion_opt==ION_IF){
                    cor_P=sat_info->cor_if_P[0];
                }
                else{
                    cor_P=sat_info->cor_P[f];
                }

                r=GeoDist(sat_info->brd_pos,rover_xyz,sig_vec);
                if(SatElAz(rover_blh,sig_vec,sat_info->el_az)<=0.0){
                    continue;
                }
                if(sat_info->el_az[0]*R2D<C.gnssC.ele_min){
                    continue;
                }

                if(sat_info->sat.sat_.sys==SYS_BDS&&sat_info->sat.sat_.prn<19){
                    gnss_err_corr_.BD2MultipathModel(sat_info);
                }

                gnss_err_corr_.ion_model_.InitSatInfo(sat_info,&rover_blh);
                gnss_err_corr_.ion_model_.GetKlobIon();
                gnss_err_corr_.ion_model_.UpdateSatInfo();
                gnss_err_corr_.trp_model_.InitSatInfo(sat_info,&rover_blh);
                gnss_err_corr_.trp_model_.GetSaasTrp(0.7, nullptr, nullptr);
                gnss_err_corr_.trp_model_.UpdateSatInfo();

                double sag_err=gnss_err_corr_.SagnacCorr(sat_info->brd_pos,rover_xyz);
                double rec_clk=x[para_.ParIndexClk(SYS_INDEX_GPS)];
                double sat_clk=sat_info->brd_clk[0]*CLIGHT;
                double trp_err=sat_info->trp_dry_delay[0]+sat_info->trp_wet_delay[0];
                double ion_err=sat_info->ion_delay[0];
                double cod_bia=sat_info->code_bias[f];

                omc=cor_P-(r+sag_err+(rec_clk-CLIGHT*sat_info->brd_clk[0])+sat_info->trp_dry_delay[0]+sat_info->trp_wet_delay[0]+sat_info->ion_delay[0]);
                meas_var=GnssMeasVar(C,GNSS_OBS_CODE,*sat_info)+sat_info->brd_eph_var+sat_info->trp_var+sat_info->ion_var;
                omcs.push_back(omc);
                meas_var_vec.push_back(meas_var);

                int npos=para_.PvaParNum(),idx=0;
                for(int j=0;j<num_full_x_;j++) H.push_back(j<3?-sig_vec[j]:(j==3)?1.0:0.0);
                if(sys==SYS_BDS) {idx=npos+SYS_INDEX_BDS;omc-=x[idx];omcs[num_L_]-=x[idx];H[idx+num_L_*num_full_x_]=1.0;sys_mask[SYS_INDEX_BDS]++;}
                else if(sys==SYS_GAL) {idx=npos+SYS_INDEX_GAL;omc-=x[idx];omcs[num_L_]-=x[idx];H[idx+num_L_*num_full_x_]=1.0;sys_mask[SYS_INDEX_GAL]++;}
                else if(sys==SYS_GLO) {idx=npos+SYS_INDEX_GLO;omc-=x[idx];omcs[num_L_]-=x[idx];H[idx+num_L_*num_full_x_]=1.0;sys_mask[SYS_INDEX_GLO]++;}
                else if(sys==SYS_QZS) {idx=npos+SYS_INDEX_QZS;omc-=x[idx];omcs[num_L_]-=x[idx];H[idx+num_L_*num_full_x_]=1.0;sys_mask[SYS_INDEX_QZS]++;}
                else sys_mask[SYS_INDEX_GPS]++;
                num_L_++;

                char buff[MAX_BUFF]={'\0'};
                sprintf(buff,"%s omc=%12.4f, var=%7.3f el=%3.1f dtr=%14.3f dts=%14.3f trp=%6.3f ion=%6.3f cbias=%6.3f obs=%14.3f range=%14.3f",
                        sat_info->sat.sat_.id.c_str(),omc,meas_var,sat_info->el_az[0]*R2D,sys==SYS_GPS?rec_clk:x[idx],sat_clk,trp_err,ion_err,cod_bia,cor_P,r+sag_err);
                LOG(DEBUG)<<buff;
            }
            num_valid_sat_++;
        }

        for(int i=0;i<NSYS;i++){
            if(sys_mask[i]) continue;
            omcs.push_back(0.0);
            for(int j=0;j<num_full_x_;j++) H.push_back(j==i+3?1.0:0.0);
            meas_var_vec.push_back(0.01);
            num_L_++;
        }

        omc_L_=Map<VectorXd>(omcs.data(),num_L_);
        H_=Map<MatrixXd>(H.data(),num_full_x_,num_L_);
//        cout<<omc_L_<<endl;
//        cout<<H_.transpose()<<endl;
        Map<VectorXd> var_vec(meas_var_vec.data(),num_L_);
        R_=var_vec.asDiagonal();
        H.clear();omcs.clear();meas_var_vec.clear();

        return num_valid_sat_;
    }

    bool cSppSolver::Estimator(tPPPLibConf C) {
        bool stat;
        num_valid_sat_=0;
        ppplib_sol_.stat=SOL_NONE;

        VectorXd x=full_x_;
        MatrixXd Px=full_Px_;
        for(int i=0;i<iter_;i++){

            GnssObsRes(0,spp_conf_,x.data());
            if(num_valid_sat_<4){
                return false;
            }

            stat=lsq_.Adjustment(omc_L_,H_,R_,x,Px,num_L_,num_full_x_);

            if(stat){
                full_x_=x;
                full_Px_=Px;
                break;
            }

        }
    }

    void cSppSolver::InitSolver(tPPPLibConf C) {
        para_=cParSetting(C);
        gnss_err_corr_.InitGnssErrCorr(C,&nav_);
        out_=new cOutSol(spp_conf_);
        out_->ref_sols_=ref_sols_;
    }

    bool cSppSolver::SolverProcess(tPPPLibConf C) {
        InitSolver(C);
        for(int i=0;i<rover_obs_.epoch_num;i++){
            epoch_idx_+=1;
            epoch_sat_obs_=rover_obs_.GetGnssObs().at(i);
            SolverEpoch();
            out_->WriteSol(ppplib_sol_,epoch_idx_);
        }
    }

    bool cSppSolver::SolverEpoch() {
        UpdateGnssObs(spp_conf_);
        CorrGnssObs();
        gnss_err_corr_.eph_model_.EphCorr(epoch_sat_info_collect_);
        Estimator(spp_conf_);
        SolutionUpdate();
    }

    bool cSppSolver::SolutionUpdate() {
        int sat_no=0;

        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_no=epoch_sat_info_collect_.at(i).sat.sat_.no;
            previous_sat_info_[sat_no-1]=epoch_sat_info_collect_.at(i);
        }

        ppplib_sol_.t_tag=epoch_sat_obs_.obs_time;
        ppplib_sol_.stat=SOL_SPP;
        int num_pva=para_.PvaParNum();
        for(int i=0;i<num_pva;i++) ppplib_sol_.pos[i]=full_x_[i];
        int num_clk=para_.RecClkParNum();
        for(int i=num_pva;i<num_clk;i++) ppplib_sol_.clk_error[i-num_pva]=full_x_[i];

        sol_collect_.push_back(ppplib_sol_);

        epoch_sat_info_collect_.clear();
    }

    cPppSolver::cPppSolver() {}

    cPppSolver::cPppSolver(tPPPLibConf C) {ppp_conf_=C;}

    cPppSolver::~cPppSolver() {}

    void cPppSolver::InitSolver(tPPPLibConf C) {
        cReadGnssPreEph clk_reader(C.fileC.clk,nav_);
        clk_reader.Reading(1);

        cReadGnssPreEph orb_reader(C.fileC.sp3[1], nav_);
        for(int i=0;i<3;i++){
            if(C.fileC.sp3[i].empty()) continue;
            orb_reader.file_=C.fileC.sp3[i];
            orb_reader.Reading(0);
        }

        cReadGnssErp erp_reader(C.fileC.erp,nav_);
        erp_reader.Reading();

        cReadGnssAntex atx_reader(C.fileC.atx,nav_);
        atx_reader.Reading();
        atx_reader.AlignAntPar2Sat(C,*rover_obs_.GetStartTime(),nav_.sta_paras,nav_.sat_ant,nav_.rec_ant);

        cReadGnssOcean blq_reader(C.fileC.blq, nav_,"JFNG",REC_ROVER);
        blq_reader.Reading();

        para_=cParSetting(C);
        gnss_err_corr_.InitGnssErrCorr(C,&nav_);

        tPPPLibConf spp_conf=C;
        spp_conf.mode=MODE_SPP;
        spp_conf.gnssC.ion_opt=ION_KLB;
        spp_conf.gnssC.frq_opt=FRQ_SINGLE;
        spp_solver_=new cSppSolver(spp_conf);
        spp_solver_->spp_conf_=spp_conf;
        spp_solver_->nav_=nav_;
        spp_solver_->ref_sols_=ref_sols_;
        spp_solver_->InitSolver(spp_conf);
    }

    bool cPppSolver::SolverProcess(tPPPLibConf C) {
        InitSolver(C);
        for(int i=0;i<rover_obs_.epoch_num;i++){
            epoch_idx_+=1;
            epoch_sat_obs_=rover_obs_.GetGnssObs().at(i);
            SolverEpoch();

        }
    }

    bool cPppSolver::SolverEpoch() {
        spp_solver_->epoch_sat_obs_=epoch_sat_obs_;
        spp_solver_->SolverEpoch();
        spp_solver_->out_->WriteSol(spp_solver_->ppplib_sol_,epoch_idx_);
    }

    cPpkSolver::cPpkSolver() {}

    cPpkSolver::cPpkSolver(tPPPLibConf C) {ppk_conf_=C;}

    cPpkSolver::~cPpkSolver() {}

    void cPpkSolver::InitSolver(tPPPLibConf C) {
        cReadGnssObs base_reader(C.fileC.base,nav_,base_obs_,REC_BASE);
        base_reader.SetGnssSysMask(SYS_GPS);
        base_reader.Reading();

        para_=cParSetting(C);
        gnss_err_corr_.InitGnssErrCorr(C,&nav_);

        tPPPLibConf spp_conf=C;
        spp_conf.mode=MODE_SPP;
        spp_conf.gnssC.ion_opt=ION_KLB;
        spp_conf.gnssC.frq_opt=FRQ_SINGLE;
        spp_solver_=new cSppSolver(spp_conf);
        spp_solver_->spp_conf_=spp_conf;
        spp_solver_->nav_=nav_;
        spp_solver_->InitSolver(spp_conf);
    }

    bool cPpkSolver::SolverProcess(tPPPLibConf C) {
        InitSolver(C);
        for(int i=0;i<rover_obs_.epoch_num;i++){
            epoch_idx_+=1;
            epoch_sat_obs_=rover_obs_.GetGnssObs().at(i);
            SolverEpoch();
        }
    }

    bool cPpkSolver::SolverEpoch() {
        spp_solver_->epoch_sat_obs_=epoch_sat_obs_;
        spp_solver_->SolverEpoch();
        spp_solver_->out_->WriteSol(spp_solver_->ppplib_sol_,epoch_idx_);
    }

    cFusionSolver::cFusionSolver() {}

    cFusionSolver::cFusionSolver(tPPPLibConf C) {fs_conf_=C;}

    cFusionSolver::~cFusionSolver() {}

    bool cFusionSolver::InputImuData(int ws) {
        int i;

        if(imu_index_<0||imu_index_>imu_data_.data_.size()) return false;

        // prepare imu data for static detect
        for(i=imu_index_;i>=0&&i<ws&&i<imu_data_.data_.size();i++){
            imu_data_zd_.push_back(imu_data_.data_.at(i));
        }

        cur_imu_data_=imu_data_.data_.at(imu_index_++);
        return true;
    }

    bool cFusionSolver::MatchGnssObs() {
        double sow1,sow2;
        int i,week=0,wod=0,info=false;

        for(i=rover_idx_-100<0?0:rover_idx_-10;rover_idx_<rover_obs_.GetGnssObs().size();i++){
            sow1=rover_obs_.GetGnssObs().at(i).obs_time.Time2Gpst(&week,&wod,SYS_GPS);
            sow2=cur_imu_data_.t_tag_.Time2Gpst(nullptr, nullptr,SYS_GPS);
            if(fabs(sow1-sow2)<DTTOL){
                rover_idx_=i;info=true;
                break;
            }
            else if((sow1-sow2)>2.0*DTTOL){
                info=false;break;
            }
        }

        return info;
    }

    void cFusionSolver::InitSolver(tPPPLibConf C) {
        para_=cParSetting(C);
        gnss_err_corr_.InitGnssErrCorr(C,&nav_);

        if(fs_conf_.mode_opt==MODE_OPT_SPP){
            tPPPLibConf spp_conf=C;
            spp_conf.mode=MODE_SPP;
            gnss_solver_=new cSppSolver(spp_conf);
            gnss_solver_->nav_=nav_;
            gnss_solver_->ref_sols_=ref_sols_;
            gnss_solver_->InitSolver(spp_conf);
        }
        else if(fs_conf_.mode_opt==MODE_OPT_PPP){
            gnss_solver_=new cPppSolver(C);
            gnss_solver_->nav_=nav_;
            gnss_solver_->ref_sols_=ref_sols_;
            gnss_solver_->InitSolver(C);
        }
        else if(fs_conf_.mode_opt==MODE_OPT_PPK){
            gnss_solver_=new cPpkSolver(C);
            gnss_solver_->nav_=nav_;
            gnss_solver_->ref_sols_=ref_sols_;
            gnss_solver_->InitSolver(C);
        }

        cReadImu imu_reader(C.fileC.imu);
        imu_reader.Reading();
        imu_data_=*imu_reader.GetImus();
    }

    bool cFusionSolver::SolverProcess(tPPPLibConf C) {
        InitSolver(C);

//        int gnss_flag=false,ins_align=false;
//        while(InputImuData(5)){
//
//            if(fs_conf_.mode!=MODE_INS){
//                gnss_flag=MatchGnssObs();
//                if(gnss_flag){
//                    // ins-gnss coupled
//                    epoch_sat_obs_=rover_obs_.GetGnssObs().at(rover_idx_);
//                    if(!ins_align){
//
//                    }
//
//                }else{
//                    // ins mechanization
//                }
//            }
//            else{
//                ins_mech_.InsMechanization(pre_imu_data_,cur_imu_data_,sol_collect_.back());
//            }
//
//        }

        for(int i=0;i<rover_obs_.epoch_num;i++){
            epoch_idx_+=1;
            epoch_sat_obs_=rover_obs_.GetGnssObs().at(i);
            SolverEpoch();
        }
    }

    bool cFusionSolver::SolverEpoch() {
        gnss_solver_->epoch_idx_=epoch_idx_;
        gnss_solver_->epoch_sat_obs_=epoch_sat_obs_;
        gnss_solver_->SolverEpoch();
        gnss_solver_->out_->WriteSol(gnss_solver_->ppplib_sol_,epoch_idx_);
    }
}