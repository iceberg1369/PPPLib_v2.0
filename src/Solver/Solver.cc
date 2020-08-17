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

    void cSolver::InitEpochSatInfo(vector<tSatInfoUnit> &sat_infos) {
        for(int j=0;j<MAX_SAT_NUM;j++) for(int k=0;k<MAX_GNSS_USED_FRQ_NUM;k++){
                if(epoch_idx_==1&&k==0) previous_sat_info_[j].stat=SAT_NO_USE;
                previous_sat_info_[j].outc[k]++;
                previous_sat_info_[j].lock[k]++;
            }

        for(int j=0;j<sat_infos.size();j++){
            for(int k=0;k<MAX_GNSS_USED_FRQ_NUM;k++){
                sat_infos.at(j).outc[k]=previous_sat_info_[sat_infos.at(j).sat.sat_.no-1].outc[k];
                sat_infos.at(j).lock[k]=previous_sat_info_[sat_infos.at(j).sat.sat_.no-1].lock[k];
            }
            for(int k=0;k<2;k++){
                sat_infos.at(j).sm_mw[k]=previous_sat_info_[sat_infos.at(j).sat.sat_.no-1].sm_mw[k];
                sat_infos.at(j).mw_idx[k]=previous_sat_info_[sat_infos.at(j).sat.sat_.no-1].mw_idx[k];
                sat_infos.at(j).var_mw[k]=previous_sat_info_[sat_infos.at(j).sat.sat_.no-1].var_mw[k];
                sat_infos.at(j).gf[k]=previous_sat_info_[sat_infos.at(j).sat.sat_.no-1].gf[k];
                sat_infos.at(j).phase_wp=previous_sat_info_[sat_infos.at(j).sat.sat_.no-1].phase_wp;
            }
        }
    }

    void cSolver::UpdateSatInfo(vector<tSatInfoUnit> &sat_infos) {
        for(int i=0;i<sat_infos.size();i++){
            previous_sat_info_[sat_infos.at(i).sat.sat_.no-1]=sat_infos.at(i);
        }

    }

    void cSolver::UpdateGnssObs(tPPPLibConf C,tEpochSatUnit& epoch_sat_obs,RECEIVER_INDEX rec) {
        int sys,i;
        int *frqs;

        for(i=0;i<epoch_sat_obs.sat_num;i++){

            tSatInfoUnit sat_info={0};
            sys=epoch_sat_obs.epoch_data.at(i).sat.sat_.sys;
            sat_info.t_tag=epoch_sat_obs.obs_time;
            sat_info.sat=epoch_sat_obs.epoch_data.at(i).sat;
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

            gnss_obs_operator_.ReAlignObs(C,sat_info,epoch_sat_obs.epoch_data.at(i),0,frqs[0]);

            if(C.gnssC.frq_opt==FRQ_DUAL){
                gnss_obs_operator_.ReAlignObs(C,sat_info,epoch_sat_obs.epoch_data.at(i),1,frqs[1]);
            }
            else if(C.gnssC.frq_opt==FRQ_TRIPLE){
                gnss_obs_operator_.ReAlignObs(C,sat_info,epoch_sat_obs.epoch_data.at(i),1,frqs[1]);
                gnss_obs_operator_.ReAlignObs(C,sat_info,epoch_sat_obs.epoch_data.at(i),2,frqs[2]);
            }

            rec==REC_ROVER?epoch_sat_info_collect_.push_back(sat_info):base_sat_info_collect_.push_back(sat_info);
        }
    }

    void cSolver::CorrGnssObs(tPPPLibConf C) {
        Vector3d rr(full_x_[0],full_x_[1],full_x_[2]);

        tSatInfoUnit *sat_info= nullptr;
        double pcv_dants[MAX_GNSS_USED_FRQ_NUM]={0},dantr[MAX_GNSS_USED_FRQ_NUM]={0};
        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);
//            if(sat_info->el_az[0]*R2D<C.gnssC.ele_min) continue;
            gnss_err_corr_.cbia_model_.InitSatInfo(sat_info, nullptr);
            gnss_err_corr_.cbia_model_.GetCodeBias();
            gnss_err_corr_.cbia_model_.UpdateSatInfo();

            if(C.mode==MODE_PPP){
                if(sat_info->el_az[0]*R2D<C.gnssC.ele_min) continue;
                gnss_err_corr_.ant_model_.SatPcvCorr(sat_info,rr, pcv_dants);
                gnss_err_corr_.ant_model_.RecAntCorr(sat_info, dantr,REC_ROVER);
                gnss_err_corr_.PhaseWindUp(*sat_info,rr);
            }

            for(int j=0;j<MAX_GNSS_USED_FRQ_NUM;j++){
                if(sat_info->raw_P[j]==0.0) continue;
                sat_info->cor_P[j]=sat_info->raw_P[j]-sat_info->code_bias[j]-dantr[j]-pcv_dants[j];
                if(sat_info->raw_L[j]==0.0) continue;
                sat_info->cor_L[j]=sat_info->raw_L[j]*sat_info->lam[j]-dantr[j]-pcv_dants[j]-sat_info->phase_wp*sat_info->lam[j];
            }

            if(sat_info->sat.sat_.sys==SYS_BDS&&sat_info->sat.sat_.prn<=18){
                gnss_err_corr_.BD2MultipathModel(sat_info);
                for(int j=0;j<3;j++) sat_info->cor_P[j]-=sat_info->bd2_mp[j];
            }
            if(C.gnssC.ion_opt==ION_IF)
                gnss_obs_operator_.MakeGnssObsComb(C,COMB_IF,sat_info,previous_sat_info_[sat_info->sat.sat_.no-1]);
        }
    }

    void cSolver::InitSolver(tPPPLibConf C) {
        para_=cParSetting(C);
    }

    bool cSolver::SolverProcess(tPPPLibConf C) {}

    bool cSolver::SolverEpoch() {}

    bool cSolver::Estimator(tPPPLibConf C) {}

    bool cSolver::SolutionUpdate() {}

    static void InitP(double unc,double unc0,int row_s,int col_s,MatrixXd& P){
        double q=unc==0.0?SQR(unc0):SQR(unc);
        Vector3d vec(q,q,q);
        P.block<3,3>(row_s,col_s)=vec.asDiagonal();
    }

    void cSolver::InitFullPx(tPPPLibConf C) {
        full_Px_=MatrixXd::Zero(num_full_x_,num_full_x_);
        Vector3d vec(0,0,0);
        if(para_.NumPos()>0){
            InitP(C.insC.init_pos_unc,UNC_POS,0,0,full_Px_);
        }
        if(para_.NumVel()>0){
            InitP(C.insC.init_vel_unc,UNC_VEL,3,3,full_Px_);
        }
        if(para_.NumAtt()>0){
            InitP(C.insC.init_att_unc,UNC_ATT,6,6,full_Px_);
        }
        if(para_.NumBa()>0){
            InitP(C.insC.init_ba_unc,UNC_BA,9,9,full_Px_);
        }
        if(para_.NumBg()>0){
            InitP(C.insC.init_bg_unc,UNC_BG,12,12,full_Px_);
        }
    }

    void cSolver::InitX(double xi, double var, int idx,double *x,double *p) {
        x[idx]=xi;
        for(int j=0;j<num_full_x_;j++){
//            full_Px_(idx,j)=full_Px_(j,idx)=idx==j?var:0.0;
            p[idx+j*num_full_x_]=p[j+idx*num_full_x_]=idx==j?var:0.0;
        }
    }

    static void SetPsd(double psd,double dt,int row_s,int col_s,MatrixXd& Q){
        Vector3d vec(psd*dt,psd*dt,psd*dt);
        Q.block<3,3>(row_s,col_s)=vec.asDiagonal();
    }

    Eigen::MatrixXd cSolver::InitQ(tPPPLibConf C,double dt) {
        Eigen::MatrixXd Q;
        Q=MatrixXd::Zero(num_full_x_,num_full_x_);
        SetPsd(C.insC.psd_acce,dt,3,3,Q);
        SetPsd(C.insC.psd_gyro,dt,6,6,Q);
        SetPsd(C.insC.psd_ba,dt,9,9,Q);
        SetPsd(C.insC.psd_bg,dt,12,12,Q);

        return Q;
    }

    cSppSolver::cSppSolver() {
        spp_conf_.mode=MODE_SPP;
        spp_conf_.gnssC.ele_min=10.0;
        spp_conf_.gnssC.frq_opt=FRQ_SINGLE;
        spp_conf_.gnssC.trp_opt=TRP_SAAS;
        spp_conf_.gnssC.ion_opt=ION_KLB;
        spp_conf_.estimator=SOLVE_LSQ;

        para_=cParSetting();
        num_full_x_=para_.GetPPPLibPar(spp_conf_);
        full_x_=VectorXd::Zero(num_full_x_);
        full_Px_=MatrixXd::Zero(num_full_x_,num_full_x_);
    }

    cSppSolver::cSppSolver(tPPPLibConf conf) {
        spp_conf_=conf;
        para_=cParSetting(spp_conf_);
        num_full_x_=para_.GetPPPLibPar(conf);
        full_x_=VectorXd::Zero(num_full_x_);
        full_Px_=MatrixXd::Zero(num_full_x_,num_full_x_);
    }

    cSppSolver::~cSppSolver() {}

    void cSppSolver::CorrDoppler(tSatInfoUnit &sat_info, Vector3d &rover_xyz,int f) {
        tSatInfoUnit sat_info0;
        Vector3d rover_blh=Xyz2Blh(rover_xyz);
        Vector3d sat_pos=sat_info.brd_pos,sat_vel=sat_info.brd_vel;
        sat_info0.brd_pos=sat_pos-sat_vel*1.0;
        Vector3d sig_vec0;
        GeoDist(sat_info0.brd_pos,rover_xyz,sig_vec0);
        SatElAz(rover_blh,sig_vec0,sat_info0.el_az);
        gnss_err_corr_.trp_model_.InitSatInfo(&sat_info0,&rover_blh);
        gnss_err_corr_.trp_model_.GetSaasTrp(0.7, nullptr, nullptr);
        gnss_err_corr_.trp_model_.UpdateSatInfo();
        gnss_err_corr_.ion_model_.InitSatInfo(&sat_info0,&rover_blh);
        gnss_err_corr_.ion_model_.UpdateSatInfo();
        double trp_delta=((sat_info.trp_dry_delay[0]+sat_info.trp_wet_delay[0])-(sat_info0.trp_dry_delay[0]+sat_info0.trp_wet_delay[0])/1.0);
        double ion_delta=((sat_info.ion_delay[0]-sat_info0.ion_delay[0])/1.0);
        sat_info.cor_D[f]=sat_info.raw_D[f];
    }

    Vector3d cSppSolver::SatVelSagnacCorr(const Vector3d &sat_vel, const double tau) {
        Vector3d vel_corr(0,0,0);
        double a=OMGE_GPS*tau;

        vel_corr[0]=sat_vel[0]+a*sat_vel[1];
        vel_corr[1]=sat_vel[1]-a*sat_vel[0];
        vel_corr[2]=sat_vel[2];
        return vel_corr;
    }

    int cSppSolver::DopplerRes(tPPPLibConf C,MatrixXd& H_mat, MatrixXd& R_mat,VectorXd& L,VectorXd& x,Vector3d rover_xyz) {
        vector<double>H,omcs,meas_var_vec;
        int sys,sys_mask[NSYS]={0},num_doppler=0;
        double omc,r,meas_var;
        tSatInfoUnit* sat_info= nullptr;
        Vector3d rover_blh=Xyz2Blh(rover_xyz),sig_vec;
        Matrix3d Cen=CalcCen(rover_blh,COORD_ENU);

        int num_used_frq=para_.GetGnssUsedFrqs();

        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);
            if(sat_info->sat.sat_.no==4) continue;
            if(sat_info->sat.sat_.no==55) continue;
            sys=sat_info->sat.sat_.sys;
//            if(sys==SYS_BDS&&(1<=sat_info->sat.sat_.prn&&sat_info->sat.sat_.prn<=5||sat_info->sat.sat_.prn>18)) continue;
//            if(sys!=SYS_GPS) continue; // only use GPS dopppler for velocity estimate
            if(sat_info->stat!=SAT_USED) continue;

            LOG_IF(i==0,DEBUG)<<"DOPPLER RESIDUAL"<<"("<<epoch_idx_<<")"<<": "<<sat_info->t_tag.GetTimeStr(1)
                              <<setprecision(12)<<" "<<"VX="<<x[0]<<" "<<"VY="<<x[1]<<" "<<"VZ="<<x[2];

            for(int f=0;f<num_used_frq;f++){
                double doppler=sat_info->raw_D[f];
                double lam=sat_info->lam[f];
                if(doppler==0.0||lam==0.0) continue;
                CorrDoppler(*sat_info,rover_xyz,f);
                Vector3d sat_pos=sat_info->brd_pos;
                Vector3d sat_vel=sat_info->brd_vel;
                Vector3d s_r=(sat_pos-rover_xyz);
                double tau=s_r.norm()/CLIGHT;
                Vector3d sat_vel_corr=SatVelSagnacCorr(sat_vel,tau);
                Vector3d rover_vel(x[0],x[1],x[2]);
                Vector3d e=s_r/s_r.norm();
                Vector3d relative_vel=sat_vel_corr-rover_vel;
                double rate=e.dot(relative_vel);

#if 0
                double cos_el=cos(sat_info->el_az[0]);
                Vector3d a(0.0,0.0,0.0);
                a[0]=sin(sat_info->el_az[1])*cos_el;
                a[1]=cos(sat_info->el_az[1])*cos_el;
                a[2]=sin(sat_info->el_az[0]);
                Vector3d e=Cen.transpose()*a;

                int idx_vel=para_.IndexPos();
                Vector3d vs(0,0,0);
                for(int j=idx_vel;j<idx_vel+3;j++) vs[j-idx_vel]=sat_info->brd_vel[j-idx_vel]-x[j];
                double rate=vs.dot(e)+OMGE_GPS/CLIGHT*(sat_info->brd_vel[1]*rover_xyz[0]+sat_info->brd_pos[1]*x[idx_vel]
                                                       -sat_info->brd_vel[0]*rover_xyz[1]-sat_info->brd_pos[0]*x[idx_vel+1]);
#endif

                int idx_clk_drift=para_.IndexClk(SYS_INDEX_GPS);
                double a=CLIGHT*sat_info->brd_clk[1];
                omc=lam*sat_info->cor_D[f]-(rate+x[idx_clk_drift]-CLIGHT*sat_info->brd_clk[1]);
                meas_var=GnssMeasVar(C,GNSS_OBS_DOPPLER,*sat_info);

                omcs.push_back(omc);
                meas_var_vec.push_back(meas_var);

                for(int j=0;j<num_full_x_;j++) H.push_back(j<3?-e[j]:(j==idx_clk_drift)?1.0:0.0);
                int idx;
                if(sys==SYS_BDS) {idx=idx_clk_drift+SYS_INDEX_BDS;omc-=x[idx];omcs[num_doppler]-=x[idx];H[idx+num_doppler*num_full_x_]=1.0;sys_mask[SYS_INDEX_BDS]++;}
                else if(sys==SYS_GAL) {idx=idx_clk_drift+SYS_INDEX_GAL;omc-=x[idx];omcs[num_doppler]-=x[idx];H[idx+num_doppler*num_full_x_]=1.0;sys_mask[SYS_INDEX_GAL]++;}
                else if(sys==SYS_GLO) {idx=idx_clk_drift+SYS_INDEX_GLO;omc-=x[idx];omcs[num_doppler]-=x[idx];H[idx+num_doppler*num_full_x_]=1.0;sys_mask[SYS_INDEX_GLO]++;}
                else if(sys==SYS_QZS) {idx=idx_clk_drift+SYS_INDEX_QZS;omc-=x[idx];omcs[num_doppler]-=x[idx];H[idx+num_doppler*num_full_x_]=1.0;sys_mask[SYS_INDEX_QZS]++;}
                else sys_mask[SYS_INDEX_GPS]++;

                num_doppler++;

                char buff[MAX_BUFF]={'\0'};
                sprintf(buff,"%s omc=%12.4f, var=%7.3f el=%3.1f clk_drift=%14.3f sat_clk_drift=%14.3f doppler=%10.3f rate=%10.3f",
                        sat_info->sat.sat_.id.c_str(),omc,meas_var,sat_info->el_az[0]*R2D,sys==SYS_GPS?x[idx_clk_drift]:x[idx],sat_info->brd_clk[1]*CLIGHT,-lam*doppler,rate);
                LOG(DEBUG)<<buff;
            }
        }

        int idx_clk=para_.IndexClk(SYS_INDEX_GPS);
        for(int i=0;i<NSYS;i++){
            if(sys_mask[i]) continue;
            omcs.push_back(0.0);
            for(int j=0;j<num_full_x_;j++) H.push_back(j==i+idx_clk?1.0:0.0);
            meas_var_vec.push_back(0.01);
            num_doppler++;
        }

        L=Map<VectorXd>(omcs.data(),num_doppler);
        H_mat=Map<MatrixXd>(H.data(),num_full_x_,num_doppler);
        Map<VectorXd> var_vec(meas_var_vec.data(),num_doppler);
        R_mat=var_vec.asDiagonal();
        H.clear();omcs.clear();meas_var_vec.clear();

        return num_doppler;
    }

    void cSppSolver::EstDopVel(Vector3d rover_xyz) {
        VectorXd L,x(num_full_x_);
        MatrixXd H,R,Q;
        int nv,stat=0;

        for(int i=0;i<num_full_x_;i++) x[i]=0.0;
        for(int i=0;i<iter_;i++){
            if((nv=DopplerRes(spp_conf_,H,R,L,x,rover_xyz))<4){
                break;
            }

//            cout<<H.transpose()<<endl;
//            cout<<R<<endl;
//            cout<<L<<endl;
            stat=lsq_.Adjustment(L,H,R,x,Q,nv,num_full_x_);

            if(stat){
                for(int j=0;j<3;j++) ppplib_sol_.vel[j]=x[j];
                break;
            }
        }
    }

    double cSppSolver::Dops() {
        tSatInfoUnit *sat_info=nullptr;
        MatrixXd H_mat(4,num_valid_sat_),Q,P;
        vector<double>H(4*num_valid_sat_,0.0);
        int n=0;

        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);
            if(sat_info->stat!=SAT_USED) continue;
            double cos_el=cos(sat_info->el_az[0]);
            double sin_el=sin(sat_info->el_az[0]);
            H[4*n]=cos_el*sin(sat_info->el_az[1]);
            H[1+4*n]=cos_el*cos(sat_info->el_az[1]);
            H[2+4*n]=sin_el;
            H[3+4*n++]=1.0;
        }
        if(n<4) return 0.0;
        H_mat=Map<MatrixXd>(H.data(),4,n);
        Q=H_mat*H_mat.transpose();
        P=Q.inverse();
        ppplib_sol_.dops[0]=SQRT(P.trace());
        ppplib_sol_.dops[1]=SQRT(P(0,0)+P(1,1)+P(2,2));
        ppplib_sol_.dops[2]=SQRT(P(0,0)+P(1,1));
        ppplib_sol_.dops[2]=SQRT(P(2,2));

        return ppplib_sol_.dops[1];
    }

    bool cSppSolver::ValidateSol(tPPPLibConf C) {
        tSatInfoUnit* sat_info= nullptr;

        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);
            if(sat_info->stat==SAT_USED&&sat_info->el_az[0]<C.gnssC.ele_min*D2R){
                sat_info->stat=SAT_LOW_EL;
                continue;
            }
        }

//        for(int i=0;i<epoch_sat_info_collect_.size();i++){
//            sat_info=&epoch_sat_info_collect_.at(i);
//            if(sat_info->stat==SAT_USED){
//                sat_info->lock[0]++;
//                sat_info->outc[0]=0;
//            }
//        }

        string buff;
        int num_no_used=0;
        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);
            if(sat_info->stat!=SAT_USED&&sat_info->stat!=SAT_SLIP){
                num_no_used++;
                if(num_no_used==1) buff=epoch_sat_info_collect_[0].t_tag.GetTimeStr(1)+" SATELLITE NO USED: "+sat_info->sat.sat_.id+"("+kGnssSatStatStr[sat_info->stat+1]+") ";
                else{
                    buff+=sat_info->sat.sat_.id+"("+kGnssSatStatStr[sat_info->stat+1]+") ";
                }
            }
        }
        LOG_IF(num_no_used>0,DEBUG)<<buff;

        double vv=lsq_.v_.dot(lsq_.v_);
        if(num_L_>num_full_x_&&vv>kChiSqr[num_L_-num_full_x_-1]){
            LOG(WARNING)<<epoch_sat_info_collect_[0].t_tag.GetTimeStr(1)<<" SPP SOLVE FAILED, CHI-SQR OVERRUN vv: "<<vv<<" THRESHOLD: "<<kChiSqr[num_L_-num_full_x_-1];
            return false;
        }

        double pdop=Dops();

        if(pdop>C.gnssC.max_pdop){
            LOG(WARNING)<<epoch_sat_info_collect_[0].t_tag.GetTimeStr(1)<<" SPP SOLVE FAILED, PDOP OVERRUN pdop: "<<pdop<<" THRESHOLD: "<<C.gnssC.max_pdop;
            return false;
        }

        return true;
    }

    int cSppSolver::GnssObsRes(int post, tPPPLibConf C,double* x) {
        num_L_=0;
        num_valid_sat_=0;
        for(int i=0;i<NSYS;i++) sys_mask_[i]=0;

        vector<double>H,omcs,meas_var_vec;
        int sys;
        double omc,r,meas_var;
        bool have_large_res=false;
        tSatInfoUnit* sat_info= nullptr;
        Vector3d rover_xyz(x[0],x[1],x[2]);
        Vector3d rover_blh=Xyz2Blh(rover_xyz),sig_vec;

        int num_used_frq=para_.GetGnssUsedFrqs();
        if(num_used_frq<=0) return 0;

        vector<int>sat_idx;
        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);

            LOG_IF(i==0,DEBUG)<<"SINGLE POINT POSITION RESIDUAL"<<"("<<epoch_idx_<<"-"<<post<<")"<<": "<<sat_info->t_tag.GetTimeStr(1)
                                       <<setprecision(12)<<" "<<"X="<<rover_xyz[0]<<" "<<"Y="<<rover_xyz[1]<<" "<<"Z="<<rover_xyz[2];

            sys=sat_info->sat.sat_.sys;
            if(sat_info->stat!=SAT_USED) continue;

            for(int f=0;f<num_used_frq;f++){
                double cor_P=0.0;
                if(spp_conf_.gnssC.ion_opt==ION_IF){
                    cor_P=sat_info->cor_if_P[0];
                }
                else{
                    cor_P=sat_info->cor_P[f];
                }

                if(cor_P==0.0){
                    sat_info->stat=SAT_NO_PR;
                    continue;
                }

                r=GeoDist(sat_info->brd_pos,rover_xyz,sig_vec);
                if(SatElAz(rover_blh,sig_vec,sat_info->el_az)<=0.0){
                    continue;
                }
                double el=sat_info->el_az[0]*R2D;
                if(sat_info->el_az[0]*R2D<C.gnssC.ele_min){
                    continue;
                }

                if(sat_info->sat.sat_.sys==SYS_BDS&&sat_info->sat.sat_.prn<19){
                    gnss_err_corr_.BD2MultipathModel(sat_info);
                }

                gnss_err_corr_.ion_model_.InitSatInfo(sat_info,&rover_blh);
                gnss_err_corr_.ion_model_.GetIonError();
                gnss_err_corr_.ion_model_.UpdateSatInfo();
                gnss_err_corr_.trp_model_.InitSatInfo(sat_info,&rover_blh);
                gnss_err_corr_.trp_model_.GetSaasTrp(0.7, nullptr, nullptr);
                gnss_err_corr_.trp_model_.UpdateSatInfo();

                double sag_err=gnss_err_corr_.SagnacCorr(sat_info->brd_pos,rover_xyz);
                double rec_clk=x[para_.IndexClk(SYS_INDEX_GPS)];
                double sat_clk=sat_info->brd_clk[0]*CLIGHT;
                double trp_err=sat_info->trp_dry_delay[0]+sat_info->trp_wet_delay[0];
                double ion_err=sat_info->ion_delay[0];
                double cod_bia=sat_info->code_bias[f];

                omc=cor_P-(r+sag_err+(rec_clk-CLIGHT*sat_info->brd_clk[0])+sat_info->trp_dry_delay[0]+sat_info->trp_wet_delay[0]+sat_info->ion_delay[0]);
                if(epoch_idx_>10&&post==0&&fabs(omc)>5.0) have_large_res=true;
                double a=GnssMeasVar(C,GNSS_OBS_CODE,*sat_info);
                meas_var=GnssMeasVar(C,GNSS_OBS_CODE,*sat_info)+sat_info->brd_eph_var+sat_info->trp_var+sat_info->ion_var;
                omcs.push_back(omc);
                meas_var_vec.push_back(meas_var);

                int idx_clk=para_.IndexClk(SYS_INDEX_GPS),idx=0;
                for(int j=0;j<num_full_x_;j++) H.push_back(j<3?-sig_vec[j]:(j==idx_clk)?1.0:0.0);
                if(sys==SYS_BDS) {idx=idx_clk+SYS_INDEX_BDS;omc-=x[idx];omcs[num_L_]-=x[idx];H[idx+num_L_*num_full_x_]=1.0;sys_mask_[SYS_INDEX_BDS]++;}
                else if(sys==SYS_GAL) {idx=idx_clk+SYS_INDEX_GAL;omc-=x[idx];omcs[num_L_]-=x[idx];H[idx+num_L_*num_full_x_]=1.0;sys_mask_[SYS_INDEX_GAL]++;}
                else if(sys==SYS_GLO) {idx=idx_clk+SYS_INDEX_GLO;omc-=x[idx];omcs[num_L_]-=x[idx];H[idx+num_L_*num_full_x_]=1.0;sys_mask_[SYS_INDEX_GLO]++;}
                else if(sys==SYS_QZS) {idx=idx_clk+SYS_INDEX_QZS;omc-=x[idx];omcs[num_L_]-=x[idx];H[idx+num_L_*num_full_x_]=1.0;sys_mask_[SYS_INDEX_QZS]++;}
                else sys_mask_[SYS_INDEX_GPS]++;
                num_L_++;

                char buff[MAX_BUFF]={'\0'};
                sprintf(buff,"%s omc=%12.4f, var=%7.3f el=%3.1f dtr=%14.3f dts=%14.3f trp=%6.3f ion=%6.3f cbias=%6.3f obs=%14.3f range=%14.3f",
                        sat_info->sat.sat_.id.c_str(),omc,meas_var,sat_info->el_az[0]*R2D,sys==SYS_GPS?rec_clk:x[idx],sat_clk,trp_err,ion_err,cod_bia,cor_P,r+sag_err);
                LOG(DEBUG)<<buff;

                if(f==0){
                    sat_idx.push_back(i);
                    num_valid_sat_++;
                }

            }
        }

        // prior outliers detecting using Median Absolute Deviation method
#if 1
        if(epoch_idx_>=10&&post==0&&have_large_res){
            double mean_omcs=VectorMean(omcs);
            vector<double>new_omcs;
            vector<double>large_omcs;
            vector<int>larger_idx;
            for(int i=0;i<omcs.size();i++){
                new_omcs.push_back(fabs(omcs[i]-mean_omcs));
            }
            double mean_new_omcs=VectorMean(new_omcs)*0.7;
            for(int i=0;i<omcs.size();i++){
                double aa=mean_omcs-3*mean_new_omcs;
                double bb=mean_omcs+3*mean_new_omcs;
                if(omcs[i]<mean_omcs-3*mean_new_omcs||omcs[i]>mean_omcs+3*mean_new_omcs){
//                    LOG(DEBUG)<<epoch_sat_info_collect_[0].t_tag.GetTimeStr(1)<<" "<<epoch_sat_info_collect_[sat_idx[i]].sat.sat_.id<<" DETECTING OUTIERS, omc="<<omcs[i];
//                    epoch_sat_info_collect_[sat_idx[i]].stat=SAT_PRI_RES_C;
                    large_omcs.push_back(omcs[i]);
                    larger_idx.push_back(i);
                }
            }

            if(large_omcs.size()>0){
                auto max_omc=max_element(large_omcs.begin(),large_omcs.end());
                int idx_max_omc=distance(begin(large_omcs),max_omc);
                epoch_sat_info_collect_[sat_idx[larger_idx[idx_max_omc]]].stat=SAT_PRI_RES_C;
                LOG(DEBUG)<<epoch_sat_info_collect_[0].t_tag.GetTimeStr(1)<<" "<<epoch_sat_info_collect_[sat_idx[larger_idx[idx_max_omc]]].sat.sat_.id<<" DETECTING OUTIERS, omc="<<large_omcs[idx_max_omc];
            }
            new_omcs.clear();large_omcs.clear();larger_idx.clear();
        }
#endif

        int idx_clk=para_.IndexClk(SYS_INDEX_GPS);
        for(int i=0;i<NSYS;i++){
            if(sys_mask_[i]) continue;
            omcs.push_back(0.0);
            for(int j=0;j<num_full_x_;j++) H.push_back(j==i+idx_clk?1.0:0.0);
            meas_var_vec.push_back(0.01);
            num_L_++;
        }

        omc_L_=Map<VectorXd>(omcs.data(),num_L_);
        H_=Map<MatrixXd>(H.data(),num_full_x_,num_L_);
        Map<VectorXd> var_vec(meas_var_vec.data(),num_L_);
        R_=var_vec.asDiagonal();
        H.clear();omcs.clear();meas_var_vec.clear();

        return post==0?have_large_res:num_valid_sat_<=0;
    }

    bool cSppSolver::Estimator(tPPPLibConf C) {
        bool stat;
        bool valid_flag=false;
        ppplib_sol_.stat=SOL_NONE;
        tSatInfoUnit* sat_info= nullptr;

        VectorXd x=full_x_;
        MatrixXd Px=full_Px_;
        for(int i=0;i<iter_;i++){

            num_valid_sat_=0;
            if(GnssObsRes(i,spp_conf_,x.data())) continue;
            if(num_valid_sat_<4){
                LOG(WARNING)<<"SPP NO ENOUGH VALID SATELLITE "<<num_valid_sat_;
                return false;
            }

            stat=lsq_.Adjustment(omc_L_,H_,R_,x,Px,num_L_,num_full_x_);

            if(stat){
                full_x_=x;
                full_Px_=Px;
                valid_flag = ValidateSol(C);
                if(valid_flag){

                    ppplib_sol_.stat=SOL_SPP;
                }
                else{
                    ppplib_sol_.stat=SOL_NONE;
                }
                break;
            }
            if(i>=iter_){
                LOG(WARNING)<<"SPP ITER OVERRUN";
            }

        }

        if(valid_flag&&C.gnssC.use_doppler){
            Vector3d rover_xyz(x[0],x[1],x[2]);
            EstDopVel(rover_xyz);
        }

        return valid_flag;
    }

    void cSppSolver::InitSolver(tPPPLibConf C) {
        para_=cParSetting(C);
        gnss_err_corr_.InitGnssErrCorr(C,&nav_);
        out_=new cOutSol(spp_conf_);
        out_->ref_sols_=ref_sols_;
    }

    bool cSppSolver::SolverProcess(tPPPLibConf C) {
        char buff[MAX_BUFF]={'\0'};
        InitSolver(C);
        for(int i=0;i<rover_obs_.epoch_num;i++){

            epoch_idx_+=1;
            epoch_sat_obs_=rover_obs_.GetGnssObs().at(i);
            LOG(DEBUG)<<"START SOLVING: "<<i+1<<"th EPOCH, ROVER SATELLITE NUMBER: "<<epoch_sat_obs_.sat_num;

            UpdateGnssObs(spp_conf_,epoch_sat_obs_,REC_ROVER);
            if(SolverEpoch()){
                epoch_ok_++;
                SolutionUpdate();
                sprintf(buff,"%s SPP SOLVE SUCCESS POS: %14.3f %14.3f %14.3f VEL: %6.3f %6.3f %6.3f PDOP: %3.1f TOTAL SAT: %3d USED SAT: %3d",
                        ppplib_sol_.t_tag.GetTimeStr(1).c_str(),ppplib_sol_.pos[0],ppplib_sol_.pos[1],ppplib_sol_.pos[2],ppplib_sol_.vel[0],ppplib_sol_.vel[1],ppplib_sol_.vel[2],
                        ppplib_sol_.dops[1],epoch_sat_obs_.sat_num,num_valid_sat_);
                LOG(DEBUG)<<buff;

                out_->WriteSol(ppplib_sol_,previous_sat_info_[80],epoch_idx_);
                epoch_sat_info_collect_.clear();
            }
            else{
                epoch_fail_++;
                epoch_sat_info_collect_.clear();
            }
        }
        LOG(INFO)<<"TOTAL EPOCH: "<<rover_obs_.epoch_num<<", SOLVE SUCCESS EPOCH: "<<epoch_ok_<<", SOLVE FAILED EPOCH: "<<epoch_fail_;
        cout<<sol_collect_.size();
    }

    bool cSppSolver::SolverEpoch() {
        gnss_err_corr_.eph_model_.EphCorr(epoch_sat_info_collect_);
        CorrGnssObs(spp_conf_);

        if(Estimator(spp_conf_)){
            ppplib_sol_.stat=SOL_SPP;
            ppplib_sol_.t_tag=epoch_sat_info_collect_[0].t_tag;
            return true;
        }
        else{
            ppplib_sol_.stat=SOL_NONE;
            ppplib_sol_.t_tag=epoch_sat_info_collect_[0].t_tag;
            return false;
        }
    }

    bool cSppSolver::SolutionUpdate() {
        int num_pos=para_.NumPos();
        for(int i=0;i<num_pos;i++) ppplib_sol_.pos[i]=full_x_[i];
        int num_clk=para_.NumClk();
        for(int i=para_.IndexClk(SYS_INDEX_GPS);i<para_.IndexClk(SYS_INDEX_GPS)+num_clk;i++) ppplib_sol_.clk_error[i-para_.IndexClk(SYS_INDEX_GPS)]=full_x_[i];

//        sol_collect_.push_back(ppplib_sol_);
    }

    cPppSolver::cPppSolver() {}

    cPppSolver::cPppSolver(tPPPLibConf C) {
        ppp_conf_=C;
        para_=cParSetting(ppp_conf_);
        num_full_x_=para_.GetPPPLibPar(ppp_conf_);
        full_x_=VectorXd::Zero(num_full_x_);
        full_Px_=MatrixXd::Zero(num_full_x_,num_full_x_);
    }

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

        cReadGnssOcean blq_reader(C.fileC.blq, nav_,"HARB",REC_ROVER);
        blq_reader.Reading();

        para_=cParSetting(C);
        gnss_err_corr_.InitGnssErrCorr(C,&nav_);

        out_=new cOutSol(ppp_conf_);
        out_->ref_sols_=ref_sols_;

        tPPPLibConf spp_conf=C;
        spp_conf.mode=MODE_SPP;
        spp_conf.gnssC.ion_opt=ION_KLB;
        spp_conf.gnssC.trp_opt=TRP_SAAS;
        spp_conf.gnssC.frq_opt=FRQ_SINGLE;
        spp_conf.gnssC.eph_opt=EPH_BRD;
        spp_solver_=new cSppSolver(spp_conf);
        spp_solver_->spp_conf_=spp_conf;
        spp_solver_->nav_=nav_;
        spp_solver_->ref_sols_=ref_sols_;
        spp_solver_->InitSolver(spp_conf);

    }

    bool cPppSolver::SolverProcess(tPPPLibConf C) {
        char buff[MAX_BUFF]={'\0'};
        InitSolver(C);

        for(int i=0;i<rover_obs_.epoch_num;i++){

            epoch_sat_obs_=rover_obs_.GetGnssObs().at(i);
            LOG(DEBUG)<<"START PPP SOLVING "<<i+1<<"th EPOCH, ROVER SATELLITE NUMBER "<<epoch_sat_obs_.sat_num;

            epoch_idx_+=1;
            UpdateGnssObs(C,epoch_sat_obs_,REC_ROVER);
            InitEpochSatInfo(epoch_sat_info_collect_);


            if(SolverEpoch()){
                SolutionUpdate();
                sprintf(buff,"%s PPP SOLVE SUCCESS POS: %14.3f %14.3f %14.3f VEL: %6.3f %6.3f %6.3f PDOP: %3.1f TOTAL SAT: %3d USED SAT: %3d",
                        ppplib_sol_.t_tag.GetTimeStr(1).c_str(),ppplib_sol_.pos[0],ppplib_sol_.pos[1],ppplib_sol_.pos[2],ppplib_sol_.vel[0],ppplib_sol_.vel[1],ppplib_sol_.vel[2],
                        ppplib_sol_.dops[1],epoch_sat_obs_.sat_num,num_valid_sat_);
                LOG(DEBUG)<<buff;
                if(epoch_idx_>100)
                   out_->WriteSol(ppplib_sol_,previous_sat_info_[0],epoch_idx_);
            }

            UpdateSatInfo(epoch_sat_info_collect_);
            epoch_sat_info_collect_.clear();
        }

        LOG(INFO)<<" TOTAL EPOCH: "<<rover_obs_.epoch_num<<", SOLVE SUCCESS EPOCH: "<<epoch_ok_<<", SOLVE FAILED EPOCH: "<<epoch_fail_;
    }

    bool cPppSolver::SolverEpoch() {
        InitSppSolver();
        if(spp_solver_->SolverEpoch()){

            Spp2Ppp();
            LOG(DEBUG)<<"PPP-SPP SOLVE SUCCESS ROVER, SPP POS "<<spp_solver_->ppplib_sol_.pos.transpose()<<" DOPPLER  VEL "<<spp_solver_->ppplib_sol_.vel.transpose();

            if(ppp_conf_.gnssC.eph_opt==EPH_PRE){
                gnss_err_corr_.eph_model_.EphCorr(epoch_sat_info_collect_);
            }

            CorrGnssObs(ppp_conf_);
            long t1=clock();
            PppCycSlip(ppp_conf_);
            StateTimeUpdate(ppp_conf_);
            long t2=clock();
            double t=(double)(t2-t1)/CLOCKS_PER_SEC;
//            cout<<t<<endl;

            if(Estimator(ppp_conf_)){
                long t2=clock();
                double t=(double)(t2-t1)/CLOCKS_PER_SEC;
                epoch_ok_++;
                LOG(DEBUG)<<epoch_sat_obs_.obs_time.GetTimeStr(1)<<" PPP SOLVE SUCCESS ";
                return true;
            }
            else{
                epoch_fail_++;
                ppplib_sol_=spp_solver_->ppplib_sol_;
                LOG(WARNING)<<epoch_sat_obs_.obs_time.GetTimeStr(1)<<" PPP SOLVE FAILED ";
                return false;
            }
        }
        else{
            epoch_fail_++;
            ppplib_sol_.stat=SOL_NONE;
            ppplib_sol_.t_tag=epoch_sat_info_collect_[0].t_tag;
            LOG(DEBUG)<<epoch_sat_obs_.obs_time.GetTimeStr(1)<<" PPP-SPP SOLVE FAILED ";
            spp_solver_->epoch_sat_info_collect_.clear();
            return false;
        }
    }

    bool cPppSolver::Estimator(tPPPLibConf C) {

        // residuals
        VectorXd x=full_x_;
        MatrixXd Px=full_Px_;

        int iter;
        for(iter=0;iter<max_iter_;iter++){

            if(!GnssObsRes(0,C,x.data())){
                LOG(WARNING)<<epoch_sat_info_collect_[0].t_tag.GetTimeStr(1)<<" MAKE PPP PRIOR RESIDUAL ERROR";
                return false;
            }

            kf_.Adjustment(omc_L_,H_,R_,x,Px,num_L_,num_full_x_);


            if(GnssObsRes(iter+1,C,x.data())) continue;
            else{
                full_x_=x;
                full_Px_=Px;
                for(int j=0;j<epoch_sat_info_collect_.size();j++){
                    if(epoch_sat_info_collect_.at(j).stat!=SAT_USED) continue;
                    for(int k=0;k<3;k++){
                        epoch_sat_info_collect_.at(j).outc[k]=0;
                        epoch_sat_info_collect_.at(j).lock[k]++;
                    }
                }
                return true;
            }

            if(iter>max_iter_){
                LOG(DEBUG)<<epoch_sat_info_collect_[0].t_tag.GetTimeStr(1)<<" PPP ITERATION OVERRUN";
                return false;
                break;
            }

        }


    }

    bool cPppSolver::SolutionUpdate() {
        ppplib_sol_.stat=SOL_PPP;
        ppplib_sol_.t_tag=epoch_sat_info_collect_[0].t_tag;
        int num_pos=para_.NumPos();
        for(int i=0;i<num_pos;i++) ppplib_sol_.pos[i]=full_x_[i];
        int num_clk=para_.NumClk();
        for(int i=para_.IndexClk(SYS_INDEX_GPS);i<para_.IndexClk(SYS_INDEX_GPS)+num_clk;i++) ppplib_sol_.clk_error[i-para_.IndexClk(SYS_INDEX_GPS)]=full_x_[i];

//        sol_collect_.push_back(ppplib_sol_);
    }

    void cPppSolver::AmbUpdate(tPPPLibConf C,double tt) {
        if(para_.NumAmb()<=0) return;

        int ia;
        double dt=0.0;
        double bias[64]={0};
        int slip[64]={0};
        tSatInfoUnit* sat_info= nullptr;
        for(int f=0;f<para_.GetGnssUsedFrqs();f++){
            for(int i=0;i<MAX_SAT_NUM;i++){
                if(previous_sat_info_[i].outc[f]>5){
                    ia=para_.IndexAmb(f,i+1);
                    InitX(0.0,0.0,ia,full_x_.data(),full_Px_.data());
                }
            }

            for(int i=0;i<epoch_sat_info_collect_.size();i++){
                sat_info=&epoch_sat_info_collect_.at(i);
                ia=para_.IndexAmb(f,sat_info->sat.sat_.no);
                bias[i]=0.0;

                if(sat_info->stat!=SAT_USED&&sat_info->stat!=SAT_SLIP) continue;

                if(C.gnssC.ion_opt==ION_IF){
                    if(sat_info->cor_if_L[f]==0.0||sat_info->cor_if_P[f]==0.0) continue;
                    bias[i]=sat_info->cor_if_L[f]-sat_info->cor_if_P[f];
                    slip[i]=sat_info->stat==SAT_SLIP;
                }
                else if(sat_info->raw_L[f]!=0.0&&sat_info->raw_P[f]!=0.0){
                    slip[i]=sat_info->stat==SAT_SLIP;
                    bias[i]=sat_info->raw_L[f]*sat_info->lam[f]-sat_info->raw_P[f];
                }
                if(full_x_[ia]==0.0||slip[i]||bias[i]==0.0) continue;
            }

            for(int i=0;i<epoch_sat_info_collect_.size();i++){
                sat_info=&epoch_sat_info_collect_.at(i);
                ia=para_.IndexAmb(f,sat_info->sat.sat_.no);

                dt=spp_solver_->ppplib_sol_.t_tag.TimeDiff(previous_sat_info_[sat_info->sat.sat_.no-1].t_tag.t_);
                full_Px_(ia,ia)+=SQR(C.gnssC.ait_psd[0])*fabs(dt);

                if(bias[i]==0.0||(full_x_[ia]!=0.0&&!slip[i])) continue;

                InitX(bias[i],SQR(60.0),ia,full_x_.data(),full_Px_.data());
                LOG(DEBUG)<<epoch_sat_info_collect_.at(i).t_tag.GetTimeStr(1)<<" "<<epoch_sat_info_collect_.at(i).sat.sat_.id<<" L"<<f+1<<" AMBIGUITY INITIALIZED "<<bias[i];
            }
        }
    }

    void cPppSolver::IonUpdate(tPPPLibConf C,double tt) {
        if(para_.NumIon()<=0) return;

        int ii;
        tSatInfoUnit* sat_info= nullptr;
        double ion;
        for(int i=0;i<MAX_SAT_NUM;i++){
            ii=para_.IndexIon(i+1);
            if(full_x_[ii]!=0.0&&previous_sat_info_[i].outc[0]>120){
                full_x_[ii]=0.0;
            }
        }

        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);
            ii=para_.IndexIon(i+1);
            if(full_x_[ii]==0.0){
                ii=para_.IndexIon(sat_info->sat.sat_.no);
                if(sat_info->cor_P[0]==0.0||sat_info->cor_P[1]==0.0) continue;
                ion=(sat_info->cor_P[0]-sat_info->cor_P[1])/(1.0-SQR(sat_info->lam[1]-sat_info->lam[0]))/sat_info->ion_delay[1];
                InitX(ion,SQR(60.0),ii,full_x_.data(),full_Px_.data());
            }
            else{
                tt=spp_solver_->ppplib_sol_.t_tag.TimeDiff(previous_sat_info_[sat_info->sat.sat_.no-1].t_tag.t_);
                double sinel=sin(MAX(sat_info->el_az[0],10.0*D2R));
                full_Px_(ii,ii)+=SQR(C.gnssC.ait_psd[1]/sinel)*fabs(tt);
            }
        }
    }

    void cPppSolver::TrpUpdate(tPPPLibConf C,double tt) {
        if(para_.NumTrp()<=0) return;

        int it=para_.IndexTrp();
        if(full_x_[it]==0.0){
            full_x_[it]=0.175;
            full_Px_(it,it)=SQR(0.3);
            if(C.gnssC.trp_opt==TRP_EST_GRAD){
                full_x_[it+1]=full_x_[it+2]=1E-6;
                full_Px_(it+1,it+1)=full_Px_(it+2,it+2)=SQR(0.01);
            }
            return;
        }
        else{
            full_Px_(it,it)+=C.gnssC.ait_psd[2]*tt;
            if(C.gnssC.trp_opt==TRP_EST_GRAD){
                full_Px_(it+1,it+1)=full_Px_(it+1,it+1)+=C.gnssC.ait_psd[2]*0.1*tt;
            }
        }
    }

    void cPppSolver::ClkUpdate(tPPPLibConf C,double tt) {
        if(para_.NumClk()<=0) return;
        int ic=para_.IndexClk(SYS_INDEX_GPS);
        bool init=false;
        double psd=0.0001;

        if(SQR(full_x_[ic])+SQR(full_x_[ic+1])+SQR(full_x_[ic+2])+SQR(full_x_[ic+3])+SQR(full_x_[ic+4])<=0.0){
            init=true;
        }

        if(init){
            for(int i=0;i<NSYS;i++)
                InitX(spp_solver_->ppplib_sol_.clk_error[i],SQR(60.0),ic+i,full_x_.data(),full_Px_.data());
        }
        else{
            if(spp_solver_->ppplib_sol_.clk_error[SYS_INDEX_GPS]!=0.0){
                // GPS clock reinitialize
                InitX(spp_solver_->ppplib_sol_.clk_error[SYS_INDEX_GPS],SQR(60.0),ic,full_x_.data(),full_Px_.data());

                // BDS,GAL,GLO,QZS ISB modeling
                for(int i=1;i<NSYS;i++){
                    full_Px_(ic+i,ic+i)+=SQR(psd)*tt;
                }
                return ;
            }
            else if(ppplib_sol_.clk_error[SYS_INDEX_BDS]!=0.0){
                InitX(spp_solver_->ppplib_sol_.clk_error[SYS_INDEX_BDS],SQR(60.0),ic,full_x_.data(),full_Px_.data());
                for(int i=2;i<NSYS;i++){
                    full_Px_(ic+i,ic+i)+=SQR(psd)*tt;
                }
                return ;
            }
            else if(ppplib_sol_.clk_error[SYS_INDEX_GAL]!=0.0){
                InitX(spp_solver_->ppplib_sol_.clk_error[SYS_INDEX_GAL],SQR(60.0),ic,full_x_.data(),full_Px_.data());
                for(int i=3;i<NSYS;i++){
                    full_Px_(ic+i,ic+i)+=SQR(psd)*tt;
                }
                return ;
            }
            else if(ppplib_sol_.clk_error[SYS_INDEX_GLO]!=0.0){
                InitX(spp_solver_->ppplib_sol_.clk_error[SYS_INDEX_GLO],SQR(60.0),ic,full_x_.data(),full_Px_.data());
                for(int i=4;i<NSYS;i++){
                    full_Px_(ic+i,ic+i)+=SQR(psd)*tt;
                }
                return ;
            }
        }
    }

    void cPppSolver::PosUpdate(tPPPLibConf C) {
        int ip=para_.IndexPos();
        Eigen::Vector3d var(SQR(60.0),SQR(60.0),SQR(60.0));

        if((SQR(full_x_[0])+SQR(full_x_[1])+SQR(full_x_[2]))==0.0){
            for(int i=0;i<3;i++) full_x_[i]=spp_solver_->ppplib_sol_.pos[i];
            full_Px_.block<3,3>(0,0)=var.asDiagonal();
            return;
        }

        if(C.mode_opt==MODE_OPT_STATIC){
            // nothing
        }
        else if(C.mode_opt==MODE_OPT_KINE_SIM||C.mode_opt==MODE_OPT_KINEMATIC){
            for(int i=0;i<3;i++) full_x_[i]=spp_solver_->ppplib_sol_.pos[i];
            full_Px_.block<3,3>(0,0)=var.asDiagonal();
        }
    }


    void cPppSolver::StateTimeUpdate(tPPPLibConf C) {
        double tt=spp_solver_->ppplib_sol_.t_tag.TimeDiff(ppplib_sol_.t_tag.t_);

        //position
        PosUpdate(C);

        //clock
        ClkUpdate(C,tt);

        //dcb

        //ifb

        //glo ifcb

        //trp
        TrpUpdate(C,tt);

        //ion
        IonUpdate(C,tt);

        //amb
        AmbUpdate(C,tt);

    }

    void cPppSolver::InitSppSolver() {
        spp_solver_->epoch_idx_=epoch_idx_;
        spp_solver_->epoch_sat_info_collect_=epoch_sat_info_collect_;
    }

    void cPppSolver::Spp2Ppp() {
        spp_solver_->SolutionUpdate();
        epoch_sat_info_collect_.clear();
        epoch_sat_info_collect_=spp_solver_->epoch_sat_info_collect_;
        spp_solver_->epoch_sat_info_collect_.clear();
        for(int i=0;i<NSYS;i++) sys_mask_[i]=spp_solver_->sys_mask_[i];
    }

    void cPppSolver::PppCycSlip(tPPPLibConf C) {
        tSatInfoUnit* sat_info= nullptr;
        cTime t=ppplib_sol_.t_tag;
        double dt=C.gnssC.sample_rate;

        if(t.t_.long_time!=0.0) dt=epoch_sat_info_collect_[0].t_tag.TimeDiff(t.t_);

        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);

            if(C.gnssC.frq_opt>FRQ_SINGLE){
                gnss_obs_operator_.MwCycleSlip(C,C.gnssC.sample_rate,dt,sat_info, nullptr,previous_sat_info_[sat_info->sat.sat_.no-1].t_tag.t_);
                gnss_obs_operator_.GfCycleSlip(C,C.gnssC.sample_rate,dt,sat_info, nullptr);
            }
            gnss_obs_operator_.SmoothMw(C,sat_info, nullptr);
        }
    }

    int cPppSolver::GnssObsRes(int post, tPPPLibConf C, double *x) {
        num_L_=0;
        num_valid_sat_=0;
        for(int i=0;i<NSYS;i++) sys_mask_[i]=0;

        bool have_larger_res=false;
        char buff[MAX_BUFF]={'\0'};
        vector<double>H,omcs,meas_var_vec,frqs,idxs,types,larger_omcs;
        int sys,num_code=0,num_doppler=0;
        double omc,r,meas,meas_var,amb=0.0;
        tSatInfoUnit* sat_info= nullptr;
        Vector3d rover_xyz(x[0],x[1],x[2]);
        cTime t=epoch_sat_info_collect_[0].t_tag;
        Vector3d dr;
        LOG(DEBUG)<<epoch_sat_info_collect_[0].t_tag.GetTimeStr(1)<<(post==0?" PRIOR(":(" POST("))<<post<<")"<<" RESIDUAL, PRIOR ROVER COORDINATE "<<rover_xyz.transpose();

        if(C.gnssC.tid_opt>TID_OFF){
            gnss_err_corr_.tid_model_.TidCorr(t.Gpst2Utc(),rover_xyz,dr);
            rover_xyz+=dr;
        }

        Vector3d rover_blh=Xyz2Blh(rover_xyz),sig_vec;
        double dantr[MAX_GNSS_USED_FRQ_NUM]={0},pcv_dants[MAX_GNSS_USED_FRQ_NUM]={0};

        int num_used_frq=para_.GetGnssUsedFrqs();
        int num_used_obs_type=para_.GetNumObsType();
        if(num_used_frq<=0) return 0;

        double rec_clk,sat_clk,cbias,phw,trp_del,ion_del,ant_corr,alpha,beta;
        for(int i=0;i<epoch_sat_info_collect_.size();i++){

            sat_info=&epoch_sat_info_collect_.at(i);
//            if(epoch_idx_==734&&sat_info->sat.sat_.no==3) continue;

            if(sat_info->stat!=SAT_USED&&sat_info->stat!=SAT_SLIP){
                LOG(WARNING)<<sat_info->t_tag.GetTimeStr(1)<<" "<<sat_info->sat.sat_.id<<" EXCLUDE BY "<<kGnssSatStatStr[sat_info->stat+1]<<" el= "<<sat_info->el_az[0]*R2D;
                continue;
            }

            if((r=GeoDist(sat_info->pre_pos,rover_xyz,sig_vec))<=0.0){
                LOG(WARNING)<<sat_info->t_tag.GetTimeStr(1)<<" "<<sat_info->sat.sat_.id<<" S_R RANGE WRONG";
                sat_info->stat=SAT_NO_USE;
                continue;
            }

            if(SatElAz(rover_blh,sig_vec,sat_info->el_az)<C.gnssC.ele_min*D2R){
                LOG(WARNING)<<sat_info->t_tag.GetTimeStr(1)<<" "<<sat_info->sat.sat_.id<<" S_R RANGE WRONG";
                sat_info->stat=SAT_NO_USE;
                continue;
            }

            sat_info->sagnac=gnss_err_corr_.SagnacCorr(sat_info->pre_pos,rover_xyz);
            sat_info->shapiro=gnss_err_corr_.ShapiroCorr(sat_info->sat.sat_.sys,sat_info->pre_pos,rover_xyz);
            gnss_err_corr_.trp_model_.InitSatInfo(sat_info,&rover_blh);
            gnss_err_corr_.trp_model_.GetTrpError(0.0,x,para_.IndexTrp());
            gnss_err_corr_.trp_model_.UpdateSatInfo();
            gnss_err_corr_.ion_model_.InitSatInfo(sat_info,&rover_blh);
            gnss_err_corr_.ion_model_.GetIonError();
            gnss_err_corr_.ion_model_.UpdateSatInfo();

            int obs_type,frq;
            for(int f=0;f<num_used_frq*num_used_obs_type;f++){
                //f%num_used_obs_type==0, L, ==1 P, ==2 D,
                //f/num_used_obs_type==1, f1,==2 f2,==3 f3
                frq=f/num_used_obs_type;
                obs_type=f%num_used_obs_type;

                //ionosphere-free
                if(C.gnssC.ion_opt==ION_IF){
                    meas=obs_type==GNSS_OBS_CODE?sat_info->cor_if_P[frq]:(obs_type==GNSS_OBS_PHASE?sat_info->cor_if_L[frq]:sat_info->cor_D[frq]);
                }
                else{
                    meas=obs_type==GNSS_OBS_CODE?sat_info->cor_P[frq]:(obs_type==GNSS_OBS_PHASE?sat_info->cor_L[frq]:sat_info->cor_D[frq]);
                }

                if(meas==0.0){
                    sat_info->stat=GNSS_OBS_CODE?sat_info->stat=SAT_NO_CODE:sat_info->stat=SAT_NO_PHASE;
                    LOG(WARNING)<<sat_info->t_tag.GetTimeStr(1)<<" "<<sat_info->sat.sat_.id<<" EXCLUDE BY "<<kGnssSatStatStr[sat_info->stat+1];
                    continue;
                }

                amb=0.0;
                // position and clock
                int ic=para_.IndexClk(SYS_INDEX_GPS);
                if(!post) for(int j=0;j<num_full_x_;j++) H.push_back(j<3?-sig_vec[j]:(j==ic)?1.0:0.0);

                // tropospheric delay
                int it=para_.IndexTrp();
                if(!post){
                    H[it+num_L_*num_full_x_]=sat_info->trp_wet_delay[1];
                    if(C.gnssC.trp_opt==TRP_EST_GRAD){
                        H[it+1+num_L_*num_full_x_]=sat_info->trp_wet_delay[2];
                        H[it+2+num_L_*num_full_x_]=sat_info->trp_wet_delay[3];
                    }
                }
                //ionospheric delay
                if(!post){
                    if(C.gnssC.ion_opt>ION_IF2){
                        int ii=para_.IndexIon(sat_info->sat.sat_.no);
                        if(x[ii]==0.0){
                            LOG(WARNING)<<sat_info->t_tag.GetTimeStr(1)<<" "<<sat_info->sat.sat_.id<<" "<<"NO VALUE FOR IONOSPHERIC PARAMETER";
                            continue;
                        }
                        H[ii+num_L_*num_full_x_]=obs_type==GNSS_OBS_PHASE?(-1.0)*sat_info->ion_delay[1]:sat_info->ion_delay[1];
                    }
                }

                //Ambiguity
                if(obs_type==GNSS_OBS_PHASE){
                    int ia=para_.IndexAmb(frq,sat_info->sat.sat_.no);
                    amb=x[ia];
                    if(amb==0.0) {
                        LOG(WARNING)<<sat_info->t_tag.GetTimeStr(1)<<" "<<sat_info->sat.sat_.id<<" "<<"NO VALUE FOR AMBIGUITY PARAMETER";
                        continue;
                    }
                    if(!post) H[ia+num_L_*num_full_x_]=1.0;
                }

                rec_clk=x[para_.IndexClk(SYS_INDEX_GPS)];
                sat_clk=sat_info->pre_clk[0]*CLIGHT;
                trp_del=sat_info->trp_dry_delay[0]+sat_info->trp_wet_delay[0];
                ion_del=0.0;
                phw=0.0;
                cbias=0.0;
                alpha=SQR(sat_info->frq[0])/(SQR(sat_info->frq[0])-SQR(sat_info->frq[1]));
                beta=-SQR(sat_info->frq[1])/(SQR(sat_info->frq[0])-SQR(sat_info->frq[1]));

                if(obs_type==GNSS_OBS_CODE){
                    if(C.gnssC.ion_opt==ION_IF){
                        cbias=alpha*sat_info->code_bias[0]+beta*sat_info->code_bias[1];
                    }
                    else cbias=sat_info->code_bias[f];
                }
                if(obs_type==GNSS_OBS_PHASE){
                    if(C.gnssC.ion_opt==ION_IF){
                        phw=alpha*sat_info->phase_wp*sat_info->lam[0]+beta*sat_info->phase_wp*sat_info->lam[1];
                    }
                    else phw=sat_info->phase_wp*sat_info->lam[f];
                }

                omc=meas-(r+sat_info->sagnac+rec_clk-sat_clk+trp_del+ion_del+amb-sat_info->shapiro);
                meas_var=GnssMeasVar(C,obs_type==0?GNSS_OBS_CODE:(obs_type==1?GNSS_OBS_PHASE:GNSS_OBS_DOPPLER),*sat_info)+sat_info->trp_var;

                if(obs_type==GNSS_OBS_CODE){
                    sprintf(buff,"%4d el=%3.1f var=%9.5f omc=%7.4f cor_obs=%14.3f range=%14.3f rec_clk=%12.3f sat_clk=%12.3f trp=%7.3f ion=%7.3f  dant=%7.3f shaprio=%7.3f cbias=%7.3f",
                            epoch_idx_,sat_info->el_az[0]*R2D,meas_var,omc,meas-dantr[f]-pcv_dants[f],r+sat_info->sagnac,rec_clk,sat_clk,trp_del,ion_del,dantr[f]+pcv_dants[f],sat_info->shapiro,cbias);
                }
                else if(obs_type==GNSS_OBS_PHASE){
                    sprintf(buff,"%4d el=%3.1f var=%9.5f omc=%7.4f cor_obs=%14.3f range=%14.3f rec_clk=%12.3f sat_clk=%12.3f trp=%7.3f ion=%7.3f  dant=%7.3f shaprio=%7.3f amb  =%7.3f phw=%7.3f",
                            epoch_idx_,sat_info->el_az[0]*R2D,meas_var,omc,meas-dantr[f]-pcv_dants[f]-phw,r+sat_info->sagnac,rec_clk,sat_clk,trp_del,ion_del,dantr[f]+pcv_dants[f],sat_info->shapiro,amb,sat_info->phase_wp);
                }

                LOG(DEBUG)<<sat_info->sat.sat_.id<<" "<<(C.gnssC.ion_opt==ION_IF?((obs_type==GNSS_OBS_CODE?"IF_P":"IF_L")):((obs_type==GNSS_OBS_CODE?"P":"L")))<<frq+1<<" "<<buff;

                if(!post&&omc>C.gnssC.max_prior){
                    if(obs_type==GNSS_OBS_CODE) sat_info->stat=SAT_PRI_RES_C;
                    else sat_info->stat=SAT_PRI_RES_P;
                    LOG(WARNING)<<sat_info->t_tag.GetTimeStr(1)<<" "<<sat_info->sat.sat_.id<<" DETECTING OUTLIER IN PRIOR RESIDUALS";
                    continue;
                }

                if(obs_type==GNSS_OBS_PHASE&&!post){
                    if(fabs(omc-omcs.back())>3.0){
                        sat_info->stat=SAT_LAR_CP_DIFF;
                        LOG(WARNING)<<sat_info->t_tag.GetTimeStr(1)<<"("<<epoch_idx_<<")"<<" "<<sat_info->sat.sat_.id<<" CODE AND PHASE PRIOR RESIDUAL SIGNIFICANT DIFFERENCE code_res="<<omcs.back()<<" phase_res="<<omc;
                        continue;
                    }
                }

                omcs.push_back(omc);
                meas_var_vec.push_back(meas_var);

                if(post&&fabs(omc)>sqrt(meas_var)*10.0){
                    have_larger_res=true;
                    larger_omcs.push_back(omc);
                    idxs.push_back(i);
                    frqs.push_back(frq);
                    types.push_back(obs_type);
                    LOG(DEBUG)<<sat_info->t_tag.GetTimeStr(1)<<"("<<epoch_idx_<<") "<<sat_info->sat.sat_.id<<" "<<(obs_type==GNSS_OBS_CODE?"P":"L")
                                <<frq+1<<" LARGER POST RESIDUAL res="<<omc<<" thres="<<sqrt(meas_var)*8.0;
                }

                if(!post){
                    if(obs_type==GNSS_OBS_CODE)  sat_info->prior_res_P[frq]=omc;
                    if(obs_type==GNSS_OBS_PHASE) sat_info->prior_res_L[frq]=omc;
                }
                else{
                    if(obs_type==GNSS_OBS_CODE)  sat_info->post_res_P[frq]=omc;
                    if(obs_type==GNSS_OBS_PHASE) sat_info->post_res_L[frq]=omc;
                }

                num_L_++;
                if(frq==0) num_valid_sat_++;
            }
        }

        if(!post){
            omc_L_=Map<VectorXd>(omcs.data(),num_L_);
            H_=Map<MatrixXd>(H.data(),num_full_x_,num_L_);
            Map<VectorXd> var_vec(meas_var_vec.data(),num_L_);
            R_=var_vec.asDiagonal();
        }

        if(post&&have_larger_res&&larger_omcs.size()>0){
            for(int i=0;i<larger_omcs.size();i++) larger_omcs[i]=fabs(larger_omcs[i]);
            auto max_omc=max_element(larger_omcs.begin(),larger_omcs.end());
            int idx_max_omc=distance(begin(larger_omcs),max_omc);
            epoch_sat_info_collect_[idxs[idx_max_omc]].stat=types[idx_max_omc]==GNSS_OBS_CODE?SAT_POS_RES_C:SAT_POS_RES_P;
            LOG(WARNING)<<epoch_sat_info_collect_[idxs[idx_max_omc]].t_tag.GetTimeStr(1)<<" "<<epoch_sat_info_collect_[idxs[idx_max_omc]].sat.sat_.id
                      <<" "<<(types[idx_max_omc]==GNSS_OBS_CODE?"P":"L")<<frqs[idx_max_omc]+1<<" EXCLUDED BY POST RESIDUAL res="<<larger_omcs[idx_max_omc];
        }

        H.clear();omcs.clear();meas_var_vec.clear();types.clear();frqs.clear();idxs.clear();larger_omcs.clear();

        return post?have_larger_res:num_valid_sat_;
    }

    cPpkSolver::cPpkSolver() {}

    cPpkSolver::cPpkSolver(tPPPLibConf C) {
        ppk_conf_=C;
        para_=cParSetting(ppk_conf_);
        num_full_x_=para_.GetPPPLibPar(ppk_conf_);
        full_x_=VectorXd::Zero(num_full_x_);
        full_Px_=MatrixXd::Zero(num_full_x_,num_full_x_);
    }

    cPpkSolver::~cPpkSolver() {}

    void cPpkSolver::InitSolver(tPPPLibConf C) {
        cReadGnssObs base_reader(C.fileC.base,nav_,base_obs_,REC_BASE);
        base_reader.SetGnssSysMask(SYS_GPS);
        base_reader.Reading();

        para_=cParSetting(C);
        gnss_err_corr_.InitGnssErrCorr(C,&nav_);
        out_=new cOutSol(ppk_conf_);
        out_->ref_sols_=ref_sols_;

        tPPPLibConf spp_conf=C;
        spp_conf.mode=MODE_SPP;
        spp_conf.gnssC.ion_opt=ION_KLB;
        spp_conf.gnssC.trp_opt=TRP_SAAS;
        spp_conf.gnssC.frq_opt=FRQ_SINGLE;
        spp_solver_=new cSppSolver(spp_conf);
        spp_solver_->spp_conf_=spp_conf;
        spp_solver_->nav_=nav_;
        spp_solver_->ref_sols_=ref_sols_;
        spp_solver_->InitSolver(spp_conf);
    }

    bool cPpkSolver::SolverProcess(tPPPLibConf C) {

        InitSolver(C);

        Vector3d blh(34.220254297*D2R,117.143996963*D2R,36.051);
        base_xyz_=Blh2Xyz(blh);

        for(int i=0;i<rover_obs_.epoch_num;i++){

            epoch_sat_obs_=rover_obs_.GetGnssObs().at(i);
            LOG(DEBUG)<<"START PPK SOLVING "<<i+1<<"th EPOCH, ROVER SATELLITE NUMBER "<<epoch_sat_obs_.sat_num;

            if(MatchBaseObs(epoch_sat_obs_.obs_time)){

                LOG(DEBUG)<<"MATCH BASE STATION OBSERVATIONS, BASE SATELLITE NUMBER "<<base_epoch_sat_obs_.sat_num;

                epoch_idx_+=1;
                UpdateGnssObs(C,base_epoch_sat_obs_,REC_BASE);
                UpdateGnssObs(C,epoch_sat_obs_,REC_ROVER);
                InitEpochSatInfo(epoch_sat_info_collect_);

                if(SolverEpoch()){
                    SolutionUpdate();
                    out_->WriteSol(ppplib_sol_,previous_sat_info_[31],epoch_idx_);
                }
            }
            else{
                ppplib_sol_.stat=SOL_NONE;
                ppplib_sol_.t_tag=epoch_sat_obs_.obs_time;
                epoch_fail_++;
                LOG(WARNING)<<"MATCH BASE STATION OBSERVATIONS FAILED";
            }

            UpdateSatInfo(epoch_sat_info_collect_);
            epoch_sat_info_collect_.clear();
            base_sat_info_collect_.clear();
            rover_res.clear();base_res.clear();
        }

        LOG(INFO)<<"TOTAL EPOCH: "<<rover_obs_.epoch_num<<", SOLVE SUCCESS EPOCH: "<<epoch_ok_<<", SOLVE FAILED EPOCH: "<<epoch_fail_;
    }

    bool cPpkSolver::SolverEpoch() {

        InitSppSolver();
        if(spp_solver_->SolverEpoch()){
            Spp2Ppk();

            LOG(DEBUG)<<"PPK-SPP SOLVE SUCCESS, SPP POS "<<spp_solver_->ppplib_sol_.pos.transpose()<<" DOPPLER  VEL "<<spp_solver_->ppplib_sol_.vel.transpose();

            if(ppk_conf_.gnssC.eph_opt==EPH_PRE){
                gnss_err_corr_.eph_model_.EphCorr(epoch_sat_info_collect_);
            }

            if(Estimator(ppk_conf_)){
                epoch_ok_++;
                LOG(DEBUG)<<epoch_sat_obs_.obs_time.GetTimeStr(1)<<" PPK SOLVE SUCCESS";
                return true;
            }else{
                epoch_fail_++;
                ppplib_sol_=spp_solver_->ppplib_sol_;
                LOG(WARNING)<<epoch_sat_obs_.obs_time.GetTimeStr(1)<<" PPK SOLVE FAILED";
                return false;
            }

        }
        else{
            epoch_fail_++;
            ppplib_sol_.stat=SOL_NONE;
            ppplib_sol_.t_tag=epoch_sat_info_collect_[0].t_tag;
            spp_solver_->epoch_sat_info_collect_.clear();
            LOG(WARNING)<<epoch_sat_obs_.obs_time.GetTimeStr(1)<<" PPK-SPP SOLVE FAILED";
            return false;
        }

    }

    bool cPpkSolver::Estimator(tPPPLibConf C) {

        // select common satellites
        vector<int>ir,ib,cmn_sat_no;
        SelectCmnSat(C,ir,ib,cmn_sat_no);
        // cycle slip
//        PpkCycleSlip(C,ir,ib,cmn_sat_no);
        // state time_update
        StateTimeUpdate(C,ir,ib,cmn_sat_no);
        // zero residual for base and rover
        if(!GnssZeroRes(C,REC_BASE,ib,full_x_.data())){
            return false;
        }

        VectorXd x=full_x_;
        MatrixXd Px=full_Px_;
        Vector3d rover_xyz(full_x_[0],full_x_[1],full_x_[2]);
        for(int iter=0;iter<1;iter++){
            if(!GnssZeroRes(C,REC_ROVER,ir,x.data())){
                break;
            }

            if(!GnssDdRes(0,C,ir,ib,cmn_sat_no,x.data())){
                LOG(WARNING)<<epoch_sat_info_collect_[0].t_tag.GetTimeStr(1)<<" "<<"MAKE DOUBLE DIFFERENCE RESIDUAL FAILED";
                return false;
            }

            if(!kf_.Adjustment(omc_L_,H_,R_,x,Px,num_L_,num_full_x_)){
                ppplib_sol_.stat=SOL_SPP;
                return false;
            }

            if(GnssZeroRes(C,REC_ROVER,ir,x.data())){
                GnssDdRes(0,C,ir,ib,cmn_sat_no,x.data());
                full_x_=x;
                full_Px_=Px;
                ppplib_sol_.stat=SOL_PPK;
                for(int j=0;j<epoch_sat_info_collect_.size();j++){
                    if(epoch_sat_info_collect_.at(j).stat!=SAT_USED) continue;
                    for(int k=0;k<3;k++){
                        epoch_sat_info_collect_.at(j).outc[k]=0;
                        epoch_sat_info_collect_.at(j).lock[k]++;
                    }
                }
            }
            else{
                ppplib_sol_.stat=SOL_SPP;
                return false;
            }
        }

        //PPK-AR
        if(ppplib_sol_.stat==SOL_PPK){
//            ppplib_sol_.stat=SOL_PPK_AR;
        }
        else{
            ppplib_sol_.stat=SOL_PPK;

        }

        ir.clear();ib.clear();cmn_sat_no.clear();
    }

    bool cPpkSolver::SolutionUpdate() {
        if(ppplib_sol_.stat==SOL_SPP) return false;
        else if(ppplib_sol_.stat==SOL_PPK){
            ppplib_sol_.t_tag=epoch_sat_info_collect_[0].t_tag;
            int num_pos=para_.NumPos();
            for(int i=0;i<num_pos;i++) ppplib_sol_.pos[i]=full_x_[i];
        }
        else if(ppplib_sol_.stat==SOL_PPK_AR){

        }
        for(int i=0;i<3;i++) spp_solver_->full_x_[i]=full_x_[i];
        sol_collect_.push_back(ppplib_sol_);
    }

    void cPpkSolver::InitSppSolver() {
        spp_solver_->epoch_idx_=epoch_idx_;
        spp_solver_->epoch_sat_info_collect_=epoch_sat_info_collect_;
    }

    void cPpkSolver::Spp2Ppk() {
        spp_solver_->SolutionUpdate();
        epoch_sat_info_collect_.clear();
        epoch_sat_info_collect_=spp_solver_->epoch_sat_info_collect_;
        spp_solver_->epoch_sat_info_collect_.clear();
    }

    bool cPpkSolver::GnssZeroRes(tPPPLibConf C, RECEIVER_INDEX rec,vector<int>sat_idx,double* x) {

        vector<tSatInfoUnit> *sat_collect;
        vector<double> *res;

        sat_collect=rec==REC_ROVER?&epoch_sat_info_collect_:&base_sat_info_collect_;
        res=rec==REC_ROVER?&rover_res:&base_res;
        double omc,r,meas;
        tSatInfoUnit* sat_info= nullptr;

        Vector3d rec_xyz;
        if(rec==REC_ROVER){
            rec_xyz<<x[0],x[1],x[2];
        }
        else{
            rec_xyz=base_xyz_;
        }

        Vector3d rover_blh=Xyz2Blh(rec_xyz);

        int num_used_frq=para_.GetGnssUsedFrqs();
        int num_used_obs_type=para_.GetNumObsType();
        if(num_used_frq<=0) return false;
        int nf=num_used_frq*num_used_obs_type;

        rover_res.resize(sat_idx.size()*num_used_frq*num_used_obs_type,0.0);
        base_res.resize(sat_idx.size()*num_used_frq*num_used_obs_type,0.0);

        for(int i=0;i<sat_idx.size();i++){

            sat_info=&sat_collect->at(sat_idx[i]);
            if(sat_info->stat!=SAT_USED&&sat_info->stat!=SAT_SLIP){
                LOG(DEBUG)<<sat_info->t_tag.GetTimeStr(1)<<" "<<(rec==REC_BASE?"BASE":"ROVER")<<" "<<sat_info->sat.sat_.id<<" NO USED("<<kGnssSatStatStr[sat_info->stat+1]<<")";
                continue;
            }

            if((r=GeoDist(sat_info->brd_pos,rec_xyz,sat_info->sig_vec))<=0.0){
                LOG(DEBUG)<<sat_info->t_tag.GetTimeStr(1)<<" "<<(rec==REC_BASE?"BASE":"ROVER")<<" "<<sat_info->sat.sat_.id<<" NO USED";
                continue;
            }

            if(SatElAz(rover_blh,sat_info->sig_vec,sat_info->el_az)<C.gnssC.ele_min*D2R){
                LOG(DEBUG)<<sat_info->t_tag.GetTimeStr(1)<<" "<<(rec==REC_BASE?"BASE":"ROVER")<<" "<<sat_info->sat.sat_.id<<" LOW ELE";
                continue;
            }

            r+=gnss_err_corr_.SagnacCorr(sat_info->brd_pos,rec_xyz);
//            gnss_err_corr_.trp_model_.InitSatInfo(sat_info,&rover_blh);
//            gnss_err_corr_.trp_model_.GetTrpError(0.0,x,para_.IndexTrp());
//            gnss_err_corr_.trp_model_.UpdateSatInfo();
//            gnss_err_corr_.ion_model_.InitSatInfo(sat_info,&rover_blh);
//            gnss_err_corr_.ion_model_.GetIonError();
//            gnss_err_corr_.ion_model_.UpdateSatInfo();
//            gnss_err_corr_.ant_model_.SatPcvCorr(sat_info,rover_blh, nullptr);
//            gnss_err_corr_.ant_model_.RecAntCorr(sat_info, nullptr,REC_ROVER);

            int obs_code,frq;
            for(int f=0;f<num_used_frq*num_used_obs_type;f++){
                //f%num_used_obs_type==0, L, ==1 P, ==2 D,
                //f/num_used_obs_type==1, f1,==2 f2,==3 f3
                frq=f>=num_used_frq?f-num_used_frq:f;
                obs_code=f<num_used_frq?0:1;

                //ionosphere-free
                if(C.gnssC.ion_opt==ION_IF){
//                    meas=obs_type==GNSS_OBS_CODE?sat_info->cor_if_P[frq]:(obs_type==GNSS_OBS_PHASE?sat_info->cor_if_L[frq]:sat_info->cor_D[frq]);
                    meas=obs_code?sat_info->cor_if_P[frq]:sat_info->cor_if_L[frq];
                }
                    //uncombined and undifference
                else{
                    meas=obs_code?sat_info->raw_P[frq]:sat_info->raw_L[frq]*sat_info->lam[frq];
//                    meas=obs_type==GNSS_OBS_CODE?sat_info->raw_P[frq]:(obs_type==GNSS_OBS_PHASE?sat_info->raw_L[frq]*sat_info->lam[frq]:sat_info->cor_D[frq]);
                }

                if(meas==0.0) {
                    LOG(DEBUG)<<sat_info->t_tag.GetTimeStr(1)<<" "<<(rec==REC_BASE?"BASE":"ROVER")<<" "<<sat_info->sat.sat_.id<<" MISSING "<<(obs_code?"P":"L")<<frq+1;
                    continue;
                }

                double sat_clk=C.gnssC.eph_opt==EPH_BRD?sat_info->brd_clk[0]*CLIGHT:sat_info->pre_clk[0]*CLIGHT;
                double trp_del=sat_info->trp_dry_delay[0]+sat_info->trp_wet_delay[0];
                double ion_del=sat_info->ion_delay[0];
                omc=meas-(r-sat_clk);
                res->at(i*num_used_frq*num_used_obs_type+f)=omc;
            }
        }

        res= nullptr;
        return true;
    }

    int cPpkSolver::GnssDdRes(int post, tPPPLibConf C,vector<int>ir,vector<int>ib,vector<int>cmn_sat_no, double *x) {
        num_L_=0;
        char buff[MAX_BUFF]={'\0'};

        int num_used_frq=para_.GetGnssUsedFrqs();
        int num_used_obs_type=para_.GetNumObsType();
        if(num_used_frq<=0) return false;

        int nf=num_used_frq*num_used_obs_type,ref_sat_idx,j;
        int frq,obs_code,nobs[NSYS][nf],iobs=0;
        double omc;
        vector<double> omcs,H,Ri,Rj,R;
        tSatInfoUnit* sat_info= nullptr, *ref_sat_info= nullptr;

        for(int i=0;i<NSYS;i++){
            for(int k=0;k<nf;k++){
                nobs[i][k]=0;
            }
        }

        for(int isys=0;isys<NSYS;isys++){

            for(int ifrq=0;ifrq<nf;ifrq++){

                frq=ifrq>=num_used_frq?ifrq-num_used_frq:ifrq;
                obs_code=ifrq<num_used_frq?0:1;

                for(j=0,ref_sat_idx=-1;j<ir.size();j++){
                    int sys=epoch_sat_info_collect_.at(ir[j]).sat.sat_.sys_idx;
                    if(sys!=isys) continue;
                    if(!ValidObs(j,num_used_frq,ifrq)) continue;
                    if(ref_sat_idx<0||epoch_sat_info_collect_.at(ir[ref_sat_idx]).el_az[0]<epoch_sat_info_collect_.at(ir[j]).el_az[0]) ref_sat_idx=j;
                }
                if(ref_sat_idx<0) continue;
                LOG(DEBUG)<<epoch_sat_info_collect_[0].t_tag.GetTimeStr(1)<<" "<<kGnssSysStr[isys+1]<<" MAKE DOUBLE DIFFERENCE, "<<" REFERENCE SATELLITE "<<epoch_sat_info_collect_[ir[ref_sat_idx]].sat.sat_.id<<" "<<epoch_sat_info_collect_[ir[ref_sat_idx]].el_az[0]*R2D;

                for(int isat=0;isat<cmn_sat_no.size();isat++){
                    if(isat==ref_sat_idx) continue;
                    sat_info=&epoch_sat_info_collect_[ir[isat]];
                    ref_sat_info=&epoch_sat_info_collect_[ir[ref_sat_idx]];
                    if(sat_info->stat!=SAT_USED) continue;
                    int sys=epoch_sat_info_collect_.at(ir[isat]).sat.sat_.sys_idx;
                    if(sys!=isys) continue;
                    if(!ValidObs(isat,num_used_frq,ifrq)){
//                        LOG(DEBUG)<<epoch_sat_info_collect_.at(ir[isat]).sat.sat_.id<<" MISSING "<<(obs_code?"P":"L")<<frq+1;
                        sat_info->stat=SAT_NO_USE;
                        continue;
                    }

                    omc=(rover_res[ref_sat_idx*nf+ifrq]-base_res[ref_sat_idx*nf+ifrq])-(rover_res[isat*nf+ifrq]-base_res[isat*nf+ifrq]);
                    omcs.push_back(omc);

                    if(!post){
                        for(int i=0;i<num_full_x_;i++) H.push_back(i<3?(-ref_sat_info->sig_vec[i]+sat_info->sig_vec[i]):0.0);
                    }
                    if(C.gnssC.ion_opt>=ION_EST){

                    }
                    if(C.gnssC.trp_opt>=TRP_EST_WET){

                    }
                    double ambi=0.0,ambj=0.0;
                    if(!obs_code){
                        int ia=para_.IndexAmb(frq,sat_info->sat.sat_.no);
                        int ref_ia=para_.IndexAmb(frq,ref_sat_info->sat.sat_.no);
                        if(C.gnssC.ion_opt!=ION_IF){
                            ambi=ref_sat_info->lam[frq]*x[ref_ia];ambj=sat_info->lam[frq]*x[ia];
                            omcs.back()-=ambi-ambj;
                            if(!post){
                                H[ref_ia+num_full_x_*num_L_]=ref_sat_info->lam[frq];
                                H[ia+num_full_x_*num_L_]=-sat_info->lam[frq];
                            }
                        }
                        else{
                            omcs.back()-=x[ref_ia]-x[ia];
                            if(!post){
                                H[ref_ia+num_full_x_*num_L_]=1.0;
                                H[ia+num_full_x_*num_L_]=-1.0;
                            }
                        }
                    }

                    sprintf(buff,"(%8.4f - %8.4f)-(%8.4f - %8.4f)-(%8.4f-%8.4f)=%8.4f",rover_res[ref_sat_idx*nf+ifrq],base_res[ref_sat_idx*nf+ifrq],rover_res[isat*nf+ifrq],base_res[isat*nf+ifrq],ambi,ambj,omcs.back());
                    LOG(DEBUG)<<(obs_code?"P":"L")<<frq+1<<": (ROVER_"<<ref_sat_info->sat.sat_.id<<"-BASE_"<<ref_sat_info->sat.sat_.id<<")-(ROVER_"<<sat_info->sat.sat_.id<<"-BASE_"<<sat_info->sat.sat_.id<<")-amb"
                              <<" = "<<buff;

                    if(!post){
                        if(obs_code){
                            sat_info->prior_res_P[frq]=omcs.back();
                        }
                        else if(!obs_code){
                            sat_info->prior_res_L[frq]=omcs.back();
                        }
                    }
                    else{
                        if(obs_code){
                            sat_info->post_res_P[frq]=omcs.back();
                        }
                        else if(!obs_code){
                            sat_info->post_res_L[frq]=omcs.back();
                        }
                    }

                    Ri.push_back(GnssMeasVar(C,(obs_code?GNSS_OBS_CODE:GNSS_OBS_PHASE),*ref_sat_info));
                    Rj.push_back(GnssMeasVar(C,(obs_code?GNSS_OBS_CODE:GNSS_OBS_PHASE),*sat_info));

                    num_L_++;
                    nobs[isys][iobs]++;
                }
                iobs++;
            }
        }

        R.resize(num_L_*num_L_,0.0);
        for(int isys=0,nt=0;isys<NSYS;isys++){
            for(int f=0;f<nf;f++){
                for(int j=0;j<nobs[isys][f];j++){
                    for(int k=0;k<nobs[isys][f];k++){
                        R[nt+k+(nt+j)*num_L_]=j==k?Ri[nt+j]+Rj[nt+j]:Ri[nt+j];
                    }
                }
                nt+=nobs[isys][f];
            }
        }


        H_=Map<MatrixXd>(H.data(),num_full_x_,num_L_);
        R_=Map<MatrixXd>(R.data(),num_L_,num_L_);
        omc_L_=Map<VectorXd>(omcs.data(),num_L_);

        H.clear();Ri.clear();Rj.clear();R.clear();omcs.clear();
        return num_L_;
    }

    bool cPpkSolver::ValidObs(int i, int nf, int f) {
        return (rover_res[i*nf*2+f]!=0.0&&base_res[i*nf*2+f]!=0.0&&(f<nf||rover_res[f-nf+i*nf*2]!=0.0&&base_res[f-nf+i*nf*2]!=0.0));
    }

    bool cPpkSolver::MatchBaseObs(cTime t) {
        double sow1,sow2;
        int i,week=0,wod=0,info=false;

        for(i=base_idx_-100<0?0:base_idx_-10;base_idx_<base_obs_.GetGnssObs().size();i++){
            sow1=base_obs_.GetGnssObs().at(i).obs_time.Time2Gpst(&week,&wod,SYS_GPS);
            sow2=t.Time2Gpst(nullptr, nullptr,SYS_GPS);
            if(fabs(sow1-sow2)<DTTOL){
                base_idx_=i;info=true;
                base_epoch_sat_obs_=base_obs_.GetGnssObs().at(base_idx_);
                break;
            }
            else if((sow1-sow2)>2.0*DTTOL){
                info=false;break;
            }
        }

        return info;
    }

    int cPpkSolver::SelectCmnSat(tPPPLibConf C, vector<int> &ir, vector<int> &ib, vector<int> &cmn_sat_no) {
        string buff;
        tSatInfoUnit rover,base;
        for(int i=0,j=0;i<epoch_sat_info_collect_.size()&&j<base_sat_info_collect_.size();i++,j++){
            rover=epoch_sat_info_collect_.at(i);
            base=base_sat_info_collect_.at(j);
//            if(rover.stat!=SAT_USED||base.stat!=SAT_USED) continue;
            if(rover.sat.sat_.no<base.sat.sat_.no) j--;
            else if(rover.sat.sat_.no>base.sat.sat_.no) i--;
            else if(rover.el_az[0]>=C.gnssC.ele_min*D2R){
                cmn_sat_no.push_back(rover.sat.sat_.no);
                ir.push_back(i);
                ib.push_back(j);
            }
        }

        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            buff+=epoch_sat_info_collect_.at(i).sat.sat_.id+" ";
        }
        LOG(DEBUG)<<"PPK ROVER STATION OBSERVED SATELLITES: "<<epoch_sat_info_collect_.size()<<" "<<buff;
        buff.clear();

//        vector<tSatInfoUnit> sat_infos;
//        for(int i=0;i<ir.size();i++){
//            sat_infos.push_back(epoch_sat_info_collect_[ir[i]]);
//        }
//        epoch_sat_info_collect_.clear();
//        epoch_sat_info_collect_=sat_infos;
//        sat_infos.clear();

        for(int i=0;i<base_sat_info_collect_.size();i++){
            buff+=base_sat_info_collect_.at(i).sat.sat_.id+" ";
        }
        LOG(DEBUG)<<"PPK BASE STATION OBSERVED SATELLITES : "<<base_sat_info_collect_.size()<<" "<<buff;
        buff.clear();

//        for(int i=0;i<ib.size();i++){
//            sat_infos.push_back(base_sat_info_collect_[ib[i]]);
//        }
//        base_sat_info_collect_.clear();
//        base_sat_info_collect_=sat_infos;
//        sat_infos.clear();


        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            buff+=epoch_sat_info_collect_.at(i).sat.sat_.id+" ";
        }

        LOG(DEBUG)<<"PPK ROVER AND BASE COMMON SATELLITE  : "<<epoch_sat_info_collect_.size()<<" "<<buff;

        for(int i=0;i<cmn_sat_no.size();i++){
            base_sat_info_collect_.at(ib[i]).brd_pos=epoch_sat_info_collect_.at(ir[i]).brd_pos;
            base_sat_info_collect_.at(ib[i]).brd_clk=epoch_sat_info_collect_.at(ir[i]).brd_clk;
            base_sat_info_collect_.at(ib[i]).pre_pos=epoch_sat_info_collect_.at(ir[i]).pre_pos;
            base_sat_info_collect_.at(ib[i]).pre_clk=epoch_sat_info_collect_.at(ir[i]).pre_clk;
        }

        return ir.size();
    }

    void cPpkSolver::PpkCycleSlip(tPPPLibConf C,vector<int>& iu,vector<int>& ib,vector<int>& cmn_sat_no) {

        tSatInfoUnit* sat_info= nullptr;
        tSatInfoUnit* base_sat= nullptr;
        cTime t=ppplib_sol_.t_tag;
        double dt=C.gnssC.sample_rate;

        if(t.t_.long_time!=0.0) dt=epoch_sat_info_collect_[0].t_tag.TimeDiff(t.t_);

        for(int i=0;i<cmn_sat_no.size();i++){
            sat_info=&epoch_sat_info_collect_.at(iu[i]);
            base_sat=&base_sat_info_collect_.at(ib[i]);

            if(C.gnssC.frq_opt>FRQ_SINGLE){
                gnss_obs_operator_.MwCycleSlip(C,C.gnssC.sample_rate,dt,sat_info, base_sat,previous_sat_info_[sat_info->sat.sat_.no-1].t_tag.t_);
                gnss_obs_operator_.GfCycleSlip(C,C.gnssC.sample_rate,dt,sat_info, base_sat);
            }
            gnss_obs_operator_.SmoothMw(C,sat_info, base_sat);
        }

#if 1
        for(int i=0;i<epoch_sat_info_collect_.size();i++){
            sat_info=&epoch_sat_info_collect_.at(i);

            if(C.gnssC.frq_opt>FRQ_SINGLE){
                gnss_obs_operator_.MwCycleSlip(C,C.gnssC.sample_rate,dt,sat_info, nullptr,previous_sat_info_[sat_info->sat.sat_.no-1].t_tag.t_);
                gnss_obs_operator_.GfCycleSlip(C,C.gnssC.sample_rate,dt,sat_info, nullptr);
            }

            gnss_obs_operator_.SmoothMw(C,sat_info, nullptr);
        }
#endif

    }

    void cPpkSolver::StateTimeUpdate(tPPPLibConf C,vector<int>& ir,vector<int>& ib,vector<int>& cmn_sat_no) {
        double tt=spp_solver_->ppplib_sol_.t_tag.TimeDiff(ppplib_sol_.t_tag.t_);
        //position
        PosUpdate(C);
        //tropospheric delay
        TrpUpdate(C,tt);
        //ionospheric delay
        IonUpdate(C,tt);
        //ambiguity
        AmbUpdate(C,tt,ir,ib,cmn_sat_no);
    }

    void cPpkSolver::PosUpdate(tPPPLibConf  C) {
        if(para_.NumPos()<=0) return;

        Vector3d q(SQR(30),SQR(30),SQR(30));
        int ip=para_.IndexPos();
        if((SQR(full_x_[ip])+SQR(full_x_[ip+1])+SQR(full_x_[ip+1]))==0.0){
            for(int i=0;i<3;i++) InitX(spp_solver_->ppplib_sol_.pos[i],SQR(30.0),ip+i,full_x_.data(),full_Px_.data());
        }else{
            if(C.mode_opt==MODE_OPT_STATIC) return;
            if(C.mode_opt==MODE_OPT_KINE_SIM||C.mode_opt==MODE_OPT_KINEMATIC){
                for(int i=0;i<3;i++) InitX(spp_solver_->ppplib_sol_.pos[i],SQR(30.0),ip+i,full_x_.data(),full_Px_.data());
            }
        }
    }

    void cPpkSolver::TrpUpdate(tPPPLibConf C, double tt) {
        if(para_.NumTrp()<=0) return;
    }

    void cPpkSolver::IonUpdate(tPPPLibConf C, double tt) {
        if(para_.NumIon()<=0) return;
    }

    void cPpkSolver::AmbUpdate(tPPPLibConf C, double tt,vector<int>& ir,vector<int>& ib,vector<int>& cmn_sat_no) {
        if(para_.NumAmb()<=0) return;

        int i,j,reset,ia,slip,rejc;
        double offset;
        for(int f=0;f<para_.GetGnssUsedFrqs();f++){

            for(i=0;i<MAX_SAT_NUM;i++){
                ia=para_.IndexAmb(f,i+1);
                reset=previous_sat_info_[i].outc[f]>C.gnssC.max_out;
                if(C.gnssC.ar_mode==AR_INST&&full_x_[ia]!=0.0){
                    InitX(0.0,0.0,ia,full_x_.data(),full_Px_.data());
                }
                else if(reset&&full_x_[ia]!=0.0){
                    InitX(0.0,0.0,ia,full_x_.data(),full_Px_.data());
                    previous_sat_info_[i].outc[f]=0;
                }
                if(C.gnssC.ar_mode!=AR_INST&&reset){

                }
            }

            for(i=0;i<cmn_sat_no.size();i++){
                ia=para_.IndexAmb(f,cmn_sat_no[i]);
                full_Px_(ia,ia)+=SQR(C.gnssC.ait_psd[0])*fabs(tt);
            }

            double cp,pr,amb,com_offset;
            vector<double>bias;
            for(i=0,j=0,offset=0.0;i<cmn_sat_no.size();i++){
                if(C.gnssC.ion_opt!=ION_IF){
                    cp=gnss_obs_operator_.GnssSdObs(epoch_sat_info_collect_[ir[i]],base_sat_info_collect_[ib[i]],f,GNSS_OBS_PHASE);
                    pr=gnss_obs_operator_.GnssSdObs(epoch_sat_info_collect_[ir[i]],base_sat_info_collect_[ib[i]],f,GNSS_OBS_CODE);
                    if(cp==0.0||pr==0.0){
                        if(cp==0.0){
                            LOG(WARNING)<<epoch_sat_info_collect_[ir[i]].t_tag.GetTimeStr(1)<<" "<<epoch_sat_info_collect_[ir[i]].sat.sat_.id<<" MISSING L"<<f+1<<" FOR INITIALIZING AMBIGUITY";
//                            epoch_sat_info_collect_[ir[i]].stat=SAT_NO_CP;
                        }
                        if(pr==0.0){
                            LOG(WARNING)<<epoch_sat_info_collect_[ir[i]].t_tag.GetTimeStr(1)<<" "<<epoch_sat_info_collect_[ir[i]].sat.sat_.id<<" MISSING P"<<f+1<<" FOR INITIALIZING AMBIGUITY";
//                            epoch_sat_info_collect_[ir[i]].stat=SAT_NO_PR;
                        }
                        bias.push_back(0.0);
                        continue;
                    }
                    amb=cp*epoch_sat_info_collect_[ir[i]].lam[f]-pr;
                    bias.push_back(amb);

                }
                else{

                }

                ia=para_.IndexAmb(f,cmn_sat_no[i]);
                if(full_x_[ia]!=0.0){
                    offset+=amb-full_x_[ia]*epoch_sat_info_collect_[ir[i]].lam[f];
                    j++;
                }
            }

            com_offset=j>0?offset/j:0.0;

            for(i=0;i<cmn_sat_no.size();i++){
                ia=para_.IndexAmb(f,cmn_sat_no[i]);
                if(bias[i]==0.0||full_x_[ia]!=0.0) continue;
                InitX((bias[i]-com_offset)/epoch_sat_info_collect_.at(ir[i]).lam[f],SQR(30.0),ia,full_x_.data(),full_Px_.data());
                LOG(DEBUG)<<epoch_sat_info_collect_.at(ir[i]).t_tag.GetTimeStr(1)<<" "<<epoch_sat_info_collect_.at(ir[i]).sat.sat_.id<<" L"<<f+1<<" AMBIGUITY INITIALIZED "<<(bias[i]-com_offset)/epoch_sat_info_collect_[ir[i]].lam[f];
            }
            bias.clear();
        }
    }

    cFusionSolver::cFusionSolver() {}

    cFusionSolver::cFusionSolver(tPPPLibConf C) {
        fs_conf_=C;
        num_full_x_=para_.GetPPPLibPar(C);
        full_x_=VectorXd::Zero(num_full_x_);
    }

    cFusionSolver::~cFusionSolver() {}

    bool cFusionSolver::InputImuData(int ws) {
        int i;

        if(imu_index_++<0||imu_index_>=imu_data_.data_.size()) return false;

        // prepare imu data for static detect
        for(i=imu_index_;i>=0&&i<ws&&i<imu_data_.data_.size();i++){
            imu_data_zd_.push_back(imu_data_.data_.at(i));
        }

        cur_imu_info_.t_tag=imu_data_.data_.at(imu_index_).t_tag;

        if(imu_index_==1) cur_imu_info_.dt=1.0/fs_conf_.insC.sample_rate;
        else cur_imu_info_.dt=cur_imu_info_.t_tag.TimeDiff(pre_imu_info_.t_tag.t_);

        if(imu_data_.data_format_==IMU_FORMAT_INCR){
            cur_imu_info_.raw_gyro=imu_data_.data_.at(imu_index_).gyro/cur_imu_info_.dt;
            cur_imu_info_.raw_acce=imu_data_.data_.at(imu_index_).acce/cur_imu_info_.dt;
        }

        return true;
    }

    bool cFusionSolver::MatchGnssObs() {
        double sow1,sow2;
        int i,week=0,wod=0,info=false;

        for(i=rover_idx_-100<0?0:rover_idx_-10;rover_idx_<rover_obs_.GetGnssObs().size();i++){
            sow1=rover_obs_.GetGnssObs().at(i).obs_time.Time2Gpst(&week,&wod,SYS_GPS);
            sow2=cur_imu_info_.t_tag.Time2Gpst(nullptr, nullptr,SYS_GPS);
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

    double cFusionSolver::Vel2Yaw(Vector3d vn) {
        return atan2(vn[1],fabs(vn[0])<1E-4?1E-4:vn[0]);
    }

    Vector3d cFusionSolver::Pos2Vel(tSolInfoUnit &sol1, tSolInfoUnit &sol2) {
        Vector3d vel={0,0,0};
        double t=sol1.t_tag.TimeDiff(sol2.t_tag.t_);

        vel=(sol1.pos-sol2.pos)/t;
        return vel;
    }

    bool cFusionSolver::GnssSol2Ins(Vector3d re,Vector3d ve) {
        if(re.norm()==0.0||ve.norm()==0.0) return false;
        Vector3d blh=Xyz2Blh(re),rn,rpy(0,0,0);
        Vector3d wiee(0,0,OMGE_GPS);
        Matrix3d Cne;
        Cne=CalcCen(blh,COORD_NED).transpose();
        pre_imu_info_.rn=blh;
        pre_imu_info_.vn=Cne.transpose()*ve;

        rpy[2]=Vel2Yaw(pre_imu_info_.vn);
        pre_imu_info_.Cbn=Euler2RotationMatrix(rpy);
        pre_imu_info_.Cbe=Cne*pre_imu_info_.Cbn;

        // lever correct
        pre_imu_info_.re=re-pre_imu_info_.Cbe*fs_conf_.insC.lever;
        Matrix3d T=VectorSkew(cur_imu_info_.raw_gyro*cur_imu_info_.dt);
        Matrix3d Omge=VectorSkew(wiee);
        pre_imu_info_.ve=ve-pre_imu_info_.Cbe*T*fs_conf_.insC.lever+Omge*pre_imu_info_.Cbe*fs_conf_.insC.lever;

        return true;
    }

    bool cFusionSolver::InsAlign() {
        int num_sols=5;
        static vector<tSolInfoUnit>sols;
        gnss_solver_->epoch_sat_obs_=epoch_sat_obs_;
        if(gnss_solver_->SolverEpoch()) {
            LOG(INFO)<<"USING GNSS TO ALIGN INS("<<sols.size()<<"): "<<gnss_solver_->ppplib_sol_.pos.transpose();
            sols.push_back(gnss_solver_->ppplib_sol_);
        }
        else return false;

        if(sols.size()>num_sols){
            for(int i=0;i<sols.size()-1;i++){
                if(sols[i+1].t_tag.TimeDiff(sols[i].t_tag.t_)>fs_conf_.gnssC.sample_rate){
                    return false;
                }
            }

            Vector3d ve;
            if(sols.back().vel.norm()==0.0){
                ve=Pos2Vel(sols.back(),*(sols.end()-2));
            }
            else ve=sols.back().vel;

            if(GnssSol2Ins(sols.back().pos,ve)){
                LOG(INFO)<<"INS INITIALIZATION OK";
                LOG(INFO)<<"INIT POSITION: "<<pre_imu_info_.re.transpose();
                LOG(INFO)<<"INIT VELOCITY: "<<pre_imu_info_.ve.transpose();
                LOG(INFO)<<"INIT ATTITUTE: "<<RotationMatrix2Euler(pre_imu_info_.Cbe).transpose()*R2D;
                return true;
            }
        }
        return false;
    }

    void cFusionSolver::InitSolver(tPPPLibConf C) {
        para_=cParSetting(C);
        gnss_err_corr_.InitGnssErrCorr(C,&nav_);
        out_=new cOutSol(C);
        out_->ref_sols_=ref_sols_;
        InitFullPx(C);

        if(fs_conf_.mode_opt==MODE_OPT_SPP){
            tPPPLibConf spp_conf=C;
            spp_conf.mode=MODE_SPP;
            spp_conf.mode_opt=MODE_OPT_KINEMATIC;
            gnss_solver_=new cSppSolver(spp_conf);
            gnss_solver_->nav_=nav_;
            gnss_solver_->ref_sols_=ref_sols_;
            gnss_solver_->InitSolver(spp_conf);
        }
        else if(fs_conf_.mode_opt==MODE_OPT_PPP){
            tPPPLibConf ppp_conf=C;
            ppp_conf.mode=MODE_PPP;
            ppp_conf.gnssC.frq_opt=FRQ_DUAL;
            gnss_solver_=new cPppSolver(ppp_conf);
            gnss_solver_->nav_=nav_;
            gnss_solver_->ref_sols_=ref_sols_;
            gnss_solver_->InitSolver(C);
        }
        else if(fs_conf_.mode_opt==MODE_OPT_PPK){
            tPPPLibConf ppk_conf=C;
            ppk_conf.mode=MODE_PPK;
            gnss_solver_=new cPpkSolver(C);
            gnss_solver_->nav_=nav_;
            gnss_solver_->ref_sols_=ref_sols_;
            gnss_solver_->InitSolver(C);
        }

        cReadImu imu_reader(C.fileC.imu);
        imu_reader.SetImu(C.insC.imu_type,C.insC.coord_type,C.insC.data_format);
        imu_reader.Reading();
        imu_data_=*imu_reader.GetImus();
    }

    bool cFusionSolver::SolverProcess(tPPPLibConf C) {
        InitSolver(C);
        tSatInfoUnit sat_info;

        int gnss_flag=false,ins_align=false;
        int mech_idx=0;
        while(InputImuData(5)){
            if(fs_conf_.mode!=MODE_INS){
                gnss_flag=MatchGnssObs();
                if(gnss_flag){
                    epoch_sat_obs_=rover_obs_.GetGnssObs().at(rover_idx_);
                    if(!ins_align){
                        ins_align=InsAlign();
                        pre_imu_info_.t_tag=cur_imu_info_.t_tag;
                        pre_imu_info_.raw_gyro=cur_imu_info_.raw_gyro;
                        pre_imu_info_.raw_acce=cur_imu_info_.raw_acce;
                        pre_imu_info_.cor_gyro=cur_imu_info_.raw_gyro;
                        pre_imu_info_.cor_acce=cur_imu_info_.raw_acce;
                        continue;
                    }

                    if(C.mode==MODE_IGLC) LooseCouple(C);
                    else if(C.mode==MODE_IGTC) TightCouple(C);

                    pre_imu_info_=cur_imu_info_;

                }else{
                    if(!ins_align){
                        pre_imu_info_=cur_imu_info_;
                        continue;
                    }

                    LOG(DEBUG)<<"INS MECHANIZATION - ("<<++mech_idx<<"):";
                    LOG(DEBUG)<<"   "<<"GYRO VALUE: "<<pre_imu_info_.cor_gyro.transpose();
                    LOG(DEBUG)<<"   "<<"ACCE VALUE: "<<pre_imu_info_.cor_acce.transpose();
                    LOG(DEBUG)<<"   "<<"Bg:         "<<pre_imu_info_.bg.transpose();
                    LOG(DEBUG)<<"   "<<"Ba:         "<<pre_imu_info_.ba.transpose();
                    LOG(DEBUG)<<"   "<<"POSITION:   "<<pre_imu_info_.re.transpose();
                    LOG(DEBUG)<<"   "<<"VELOCITY:   "<<pre_imu_info_.ve.transpose();
                    LOG(DEBUG)<<"   "<<"ATTITUDE:   "<<RotationMatrix2Euler(pre_imu_info_.Cbe).transpose();
                    ins_mech_.InsMechanization(C,pre_imu_info_,cur_imu_info_);
                    LOG(DEBUG)<<"INS MECHANIZATION + ("<<+mech_idx<<"):";
                    LOG(DEBUG)<<"   "<<"GYRO VALUE: "<<cur_imu_info_.cor_gyro.transpose();
                    LOG(DEBUG)<<"   "<<"ACCE VALUE: "<<cur_imu_info_.cor_acce.transpose();
                    LOG(DEBUG)<<"   "<<"POSITION:   "<<cur_imu_info_.re.transpose();
                    LOG(DEBUG)<<"   "<<"VELOCITY:   "<<cur_imu_info_.ve.transpose();
                    LOG(DEBUG)<<"   "<<"ATTITUDE:   "<<RotationMatrix2Euler(cur_imu_info_.Cbe).transpose();

                    StateTimeUpdate();
                    pre_imu_info_=cur_imu_info_;
                }
            }
            else{

            }
        }
    }

    bool cFusionSolver::SolverEpoch() {
        gnss_solver_->epoch_idx_=epoch_idx_;
        gnss_solver_->epoch_sat_obs_=epoch_sat_obs_;
        if(gnss_solver_->SolverEpoch()){
            gnss_solver_->out_->WriteSol(gnss_solver_->ppplib_sol_,previous_sat_info_[9],epoch_idx_);
        }
    }

    bool cFusionSolver::SolutionUpdate() {

    }

    void cFusionSolver::CloseLoopState() {
        int ip=para_.IndexPos();
        cur_imu_info_.re[0]-=full_x_[ip+0];
        cur_imu_info_.re[1]-=full_x_[ip+1];
        cur_imu_info_.re[2]-=full_x_[ip+2];

        int iv=para_.IndexVel();
        cur_imu_info_.ve[0]-=full_x_[iv+0];
        cur_imu_info_.ve[1]-=full_x_[iv+1];
        cur_imu_info_.ve[2]-=full_x_[iv+2];

        int ia=para_.IndexAtt();
        Vector3d att(full_x_[ia],full_x_[ia+1],full_x_[ia+2]);
        Matrix3d T=Matrix3d::Identity()-VectorSkew(att);
        cur_imu_info_.Cbe=T*cur_imu_info_.Cbe;

        int iba=para_.IndexBa();
        cur_imu_info_.ba[0]+=full_x_[iba+0];
        cur_imu_info_.ba[1]+=full_x_[iba+1];
        cur_imu_info_.ba[2]+=full_x_[iba+2];

        int ibg=para_.IndexBg();
        cur_imu_info_.bg[0]+=full_x_[ibg+0];
        cur_imu_info_.bg[1]+=full_x_[ibg+1];
        cur_imu_info_.bg[2]+=full_x_[ibg+2];
    }

    void cFusionSolver::StateTimeUpdate() {
        double dt=cur_imu_info_.dt;
        int nx=para_.GetInsTransParNum(fs_conf_);
        MatrixXd F,P,Q;
        F=MatrixXd::Zero(nx,nx);
        P=full_Px_.block<15,15>(0,0);

        F=ins_mech_.StateTransferMat(fs_conf_,pre_imu_info_,cur_imu_info_,nx);
        Q=InitQ(fs_conf_,dt);

        P=F*P*F.transpose()+Q;
        full_Px_.block<15,15>(0,0)=P;

        for(int i=0;i<nx;i++) full_x_[i]=1E-20;
    }

    void cFusionSolver::RemoveLever(const Vector3d &ins_re, const Vector3d &ins_ve, Vector3d &lever, Vector3d &gnss_re,
                                    Vector3d &gnss_ve) {
        Matrix3d Cbe=cur_imu_info_.Cbe;
        Vector3d wiee(0.0,0.0,OMGE_GPS);
        Vector3d T=Cbe*lever;

        // position correction
        gnss_re=ins_re+T;

        //velocity correction
        Vector3d omge=cur_imu_info_.cor_gyro,W;
        T=Cbe*VectorSkew(omge)*lever;
        W=VectorSkew(wiee)*Cbe*lever;
        gnss_ve=ins_ve+T-W;
    }

    int cFusionSolver::BuildLcHVR(int post,tPPPLibConf C,Vector3d ins_re,Vector3d ins_ve,double *meas_pos,double *meas_vel) {
        Vector3d gnss_re,gnss_ve;
        Matrix3d Cbe=cur_imu_info_.Cbe;
        double omc=0.0;
        vector<double>omcs;
        num_L_=0;

        if(!meas_pos&&meas_vel) return 0;
        if(meas_pos) num_L_+=3;
        if(meas_vel) num_L_+=3;
        H_=MatrixXd::Zero(num_L_,num_full_x_);
        R_=MatrixXd::Zero(num_L_,num_L_);

        RemoveLever(ins_re,ins_ve,C.insC.lever,gnss_re,gnss_ve);

        if(meas_pos){
            for(int i=0;i<3;i++){
                omc=meas_pos[i]-gnss_re[i];
                omcs.push_back(omc);
            }

            if(!post){
                // position to position jacobian
                H_.block<3,3>(0,0)=-1.0*Matrix3d::Identity();

                // position to attitude jacobian
                H_.block<3,3>(0,6)=VectorSkew(Cbe*C.insC.lever);

                Vector3d q(2.5,2.5,2.5);
                R_.block<3,3>(0,0)=q.asDiagonal();
            }
        }

        if(meas_vel){
            for(int i=0;i<3;i++){
                omc=meas_vel[i]-gnss_ve[i];
                omcs.push_back(omc);
            }

            if(!post){
                // velocity to velocity jacobian
                H_.block<3,3>(3,3)=-1.0*Matrix3d::Identity();

                // velocity to attitude jacobian
                Vector3d wiee(0.0,0.0,OMGE_GPS);
                Vector3d T,W;
                T=Cbe*VectorSkew(cur_imu_info_.cor_gyro)*C.insC.lever;
                W=VectorSkew(wiee)*Cbe*C.insC.lever;
                H_.block<3,3>(3,6)=VectorSkew(T-W);

                // velocity to bg jacobian
                H_.block<3,3>(3,12)=Cbe*(VectorSkew(C.insC.lever));

                Vector3d q(0.1,0.1,0.1);
                R_.block<3,3>(3,3)=q.asDiagonal();
            }
        }

        omc_L_=Map<VectorXd>(omcs.data(),num_L_);
        omcs.clear();

        return num_L_;
    }

    bool cFusionSolver::LcFilter(tPPPLibConf C) {
        ppplib_sol_.stat=gnss_solver_->ppplib_sol_.stat;
        ppplib_sol_.pos=gnss_solver_->ppplib_sol_.pos;
        ppplib_sol_.vel=gnss_solver_->ppplib_sol_.vel;

        bool stat=false;
        Vector3d ins_re=cur_imu_info_.re,ins_ve=cur_imu_info_.ve;
        double *meas_pos= nullptr,*meas_vel= nullptr;

        if(ppplib_sol_.pos.norm()!=0.0) meas_pos=ppplib_sol_.pos.data();
        if(ppplib_sol_.vel.norm()!=0.0) meas_vel=ppplib_sol_.vel.data();

        if(BuildLcHVR(0,C,ins_re,ins_ve,meas_pos,meas_vel)){

            cout<<full_x_<<endl;
            cout<<full_Px_<<endl;
            kf_.Adjustment(omc_L_,H_,R_,full_x_,full_Px_,num_L_,num_full_x_);
            cout<<full_x_<<endl;
            CloseLoopState();
        }

    }

    bool cFusionSolver::LooseCouple(tPPPLibConf C) {
        epoch_idx_++;

        gnss_solver_->epoch_idx_=epoch_idx_;
        gnss_solver_->epoch_sat_obs_=epoch_sat_obs_;
        if(gnss_solver_->SolverEpoch()){
            LcFilter(C);
            ppplib_sol_.pos=cur_imu_info_.re;
            ppplib_sol_.vel=cur_imu_info_.ve;
            out_->WriteSol(ppplib_sol_,previous_sat_info_[0],epoch_idx_);
        }

    }

    bool cFusionSolver::TightCouple(tPPPLibConf C) {

    }
}