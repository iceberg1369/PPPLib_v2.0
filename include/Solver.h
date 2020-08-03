//
// Created by cc on 7/23/20.
//

#ifndef PPPLIB_SOLVER_H
#define PPPLIB_SOLVER_H

#include "GnssFunc.h"
#include "GnssErrorModel.h"
#include "InsFunc.h"
#include "AdjFunc.h"
#include "OutSol.h"

namespace PPPLib{

    class cSolver {
    public:
        cSolver();
        virtual ~cSolver();

    public:
        void UpdateGnssObs(tPPPLibConf C);
        virtual int GnssObsRes(int post,tPPPLibConf C,double* x);

        virtual void InitSolver(tPPPLibConf C);
        virtual bool SolverProcess(tPPPLibConf C);
        virtual bool SolverEpoch();
        virtual bool Estimator(tPPPLibConf C);
        virtual bool SolutionUpdate();

    public:
        cGnssObsOperator gnss_obs_operator_;
        cGnssErrCorr gnss_err_corr_;
        cParSetting para_;
        cLsqAdjuster lsq_;
        cOutSol *out_;

    public:
        int epoch_idx_;
        tNav nav_;
        cGnssObs rover_obs_;
        vector<tSolInfoUnit> ref_sols_;

        tEpochSatUnit epoch_sat_obs_;
        vector<tSatInfoUnit> epoch_sat_info_collect_;

        tSolInfoUnit ppplib_sol_;
        vector<tSolInfoUnit> sol_collect_;
        tSatInfoUnit previous_sat_info_[MAX_SAT_NUM];

    public:
        int num_full_x_,num_zip_x_;
        VectorXd full_x_;
        MatrixXd full_Px_;
        int num_valid_sat_;
        int num_L_;
        VectorXd omc_L_;
        MatrixXd H_;
        MatrixXd R_;
    };

    class cSppSolver:public cSolver {
    public:
        cSppSolver();
        cSppSolver(tPPPLibConf conf);
        ~cSppSolver();

    public:
        void CorrGnssObs();
        int GnssObsRes(int post,tPPPLibConf C,double* x) override;
        void InitSolver(tPPPLibConf C) override;
        bool SolverProcess(tPPPLibConf C) override;
        bool SolverEpoch() override;
        bool Estimator(tPPPLibConf C) override;
        bool SolutionUpdate() override;

    private:
        int iter_=10;
    public:
        tPPPLibConf spp_conf_;
    };

    class cPppSolver:public cSolver {
    public:
        cPppSolver();
        cPppSolver(tPPPLibConf C);
        ~cPppSolver();

    public:
        void InitSolver(tPPPLibConf C) override;
        bool SolverProcess(tPPPLibConf C) override;
        bool SolverEpoch() override;

    private:
        tPPPLibConf ppp_conf_;

    public:
        cSppSolver *spp_solver_;
    };

    class cPpkSolver:public cSolver {
    public:
        cPpkSolver();
        cPpkSolver(tPPPLibConf C);
        ~cPpkSolver();

    public:
        void InitSolver(tPPPLibConf C) override;
        bool SolverProcess(tPPPLibConf C) override;
        bool SolverEpoch() override;

    private:
        cGnssObs base_obs_;
        cSppSolver *spp_solver_;
        tPPPLibConf ppk_conf_;
    };

    class cFusionSolver:public cSolver {
    public:
        cFusionSolver();
        cFusionSolver(tPPPLibConf C);
        ~cFusionSolver();

    private:
        bool InputImuData(int ws);
        bool MatchGnssObs();

    public:
        void InitSolver(tPPPLibConf C) override;
        bool SolverProcess(tPPPLibConf C) override;
        bool SolverEpoch() override;

    private:
        cInsMech ins_mech_;
        cSolver *gnss_solver_;
        tPPPLibConf fs_conf_;

    private:
        cImuData imu_data_;

    private:
        int imu_index_=0;
        int rover_idx_=0;
        tImuDataUnit cur_imu_data_={0};
        tImuDataUnit pre_imu_data_={0};
        vector<tImuDataUnit> imu_data_zd_;
    };
}




#endif //PPPLIB_SOLVER_H
