//
// Created by cc on 7/23/20.
//

#ifndef PPPLIB_SOLVER_H
#define PPPLIB_SOLVER_H

#include "GnssFunc.h"
#include "GnssErrorModel.h"
#include "InsFunc.h"

namespace PPPLib{

    typedef struct {
        double P1,P2,P3;
        double L1,L2,L3;
        double f1,f2,f3;
        double D;
    }tCorrGnssMeasUnit;

    class cSolver {
    public:
        cSolver();
        virtual ~cSolver();

    public:
        void MakeObsComb(tPPPLibConf C);
        void OutGnssObs(tSatInfoUnit sat_info,int f);
        void ReAlignObs(tSatInfoUnit& sat_info, tSatObsUnit sat_obs,int f,int frq_idx);
        void UpdateGnssObs(tPPPLibConf C);
        virtual void GnssErrCorr(Vector3d blh,tSatInfoUnit& sat_info);
        virtual int GnssObsRes(int post,tPPPLibConf C);
        virtual void InitSolver();
        virtual bool SolverProcess();
        virtual bool Estimator(tPPPLibConf C);
        virtual bool SolutionUpdate();

    public:
        tNav nav_;
        cGnssObs rover_obs_;
        vector<tSolInfoUnit> sol_collect_;
        tSatInfoUnit previous_sat_info_[MAX_SAT_NUM];
        tEpochSatUnit epoch_sat_obs_;
        vector<tSatInfoUnit> epoch_sat_info_collect_;
    };

    class cSppSolver:public cSolver {
    public:
        cSppSolver();
        cSppSolver(tPPPLibConf conf);
        ~cSppSolver();

    public:
        int GnssObsRes(int post,tPPPLibConf C) override;
        void GnssErrCorr(Vector3d blh,tSatInfoUnit& sat_info) override;
        void InitSolver() override;
        bool SolverProcess() override;
        bool Estimator(tPPPLibConf C) override;
        bool SolutionUpdate() override;

    private:
        void CorrGnssObs();

    private:
        int iter_=1;
        tPPPLibConf spp_conf_;
        cParSetting spp_para_;
        cGnssErrCorr spp_err_corr_;

    public:
        int num_full_x_,num_zip_x_;
        vector<double> full_x_,full_Px,zip_x_,zip_Px;
        tSolInfoUnit spp_sols_;
    };

//    class cPppSolver:public cSolver {
//    public:
//        cPppSolver();
//        ~cPppSolver();
//
//    public:
//        bool SolverStart() override;
//
//    private:
//        cSppSolver spp_solver_;
//        tPPPLibConf ppp_conf_;
//        cGnssError ppp_err_model_;
//
//    public:
//        tSolInfoUnit ppp_sols_;
//    };
//
//    class cPpkSolver:public cSolver {
//    public:
//        cPpkSolver();
//        ~cPpkSolver();
//
//    private:
//        cGnssObs base_obs_;
//        cSppSolver spp_solver_;
//        cGnssError ppk_err_model_;
//        tPPPLibConf ppk_conf_;
//
//    public:
//        tSolInfoUnit ppk_sols_;
//    };
//
//    class cFusionSolver:public cSolver {
//    public:
//        cFusionSolver();
//        ~cFusionSolver();
//
//    private:
//        cImuData imu_data_;
//    };
}




#endif //PPPLIB_SOLVER_H
