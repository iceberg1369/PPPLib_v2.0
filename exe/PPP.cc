//
// Created by cc on 8/2/20.
//

#include "ReadFiles.h"
#include "Solver.h"
#include "PlotFunc.h"
INITIALIZE_EASYLOGGINGPP

using namespace PPPLib;

int main(int argc, char** argv){
    string logini_path = SetLogConfPath("");
    int log_level = SetLogLevel(1);
    InitLog(argc,argv,logini_path, log_level);

    tPPPLibConf C;

#if 0
    C.fileC.rover="../data/harb3350.19o";
    C.fileC.brd="../data/brdm3350.19p";
    C.fileC.cbias="../data/CAS0MGXRAP_20193350000_01D_01D_DCB.BSX";
    C.fileC.sp3[1]="../data/wum20820.sp3";
    C.fileC.sp3[2]="../data/wum20821.sp3";
    C.fileC.clk="../data/wum20820.clk";
    C.fileC.atx="../data/igs14_2097.atx";
    C.fileC.blq="../data/ocnload.blq";
    C.fileC.erp="../data/wum20820.erp";
    C.gnssC.sample_rate=30.0;
#else
    C.fileC.rover="../data1/cpt00870.19o";
    C.fileC.brd="../data1/brdm0870.19p";
    C.fileC.cbias="../data1/CAS0MGXRAP_20190870000_01D_01D_DCB.BSX";
    C.fileC.ref="../data1/cpt00870.ref";
    C.fileC.sp3[0]="../data1/wum20463.sp3";
    C.fileC.sp3[1]="../data1/wum20464.sp3";
    C.fileC.sp3[2]="../data1/wum20465.sp3";
    C.fileC.clk="../data1/wum20464.clk";
    C.fileC.erp="../data1/wum20464.erp";
    C.fileC.atx="../data1/igs14_2097.atx";
    C.fileC.blq="../data1/ocnload.blq";
    C.gnssC.sample_rate=1.0;
#endif


    C.mode=MODE_PPP;
    C.mode_opt=MODE_OPT_KINEMATIC;
    C.dynamic=false;
    C.gnssC.nav_sys=SYS_GPS;
    C.gnssC.frq_opt=FRQ_DUAL;
    C.gnssC.ion_opt=ION_IF;
    C.gnssC.trp_opt=TRP_EST_WET;
    C.gnssC.eph_opt=EPH_PRE;
    C.gnssC.tid_opt=TID_SOLID;
    C.gnssC.ele_min=10.0;
    C.gnssC.max_pdop=30.0;
    C.gnssC.max_out=5;
    C.gnssC.max_prior=30.0;
    C.gnssC.cs_thres[0]=5.0;
    C.gnssC.cs_thres[1]=0.15;
    C.gnssC.gnss_frq[SYS_INDEX_GPS][0]=GPS_L1;C.gnssC.gnss_frq[SYS_INDEX_GPS][1]=GPS_L2;C.gnssC.gnss_frq[SYS_INDEX_GPS][2]=GPS_L5;
    C.gnssC.gnss_frq[SYS_INDEX_BDS][0]=BDS_B1I;C.gnssC.gnss_frq[SYS_INDEX_BDS][1]=BDS_B2I;C.gnssC.gnss_frq[SYS_INDEX_BDS][2]=BDS_B3I;
    C.gnssC.gnss_frq[SYS_INDEX_GAL][0]=GAL_E1;C.gnssC.gnss_frq[SYS_INDEX_GAL][1]=GAL_E5a;C.gnssC.gnss_frq[SYS_INDEX_GAL][2]=GAL_E5b;
    C.gnssC.gnss_frq[SYS_INDEX_GLO][0]=GLO_G1;C.gnssC.gnss_frq[SYS_INDEX_GLO][1]=GLO_G2;
    C.gnssC.gnss_frq[SYS_INDEX_QZS][0]=QZS_L1;C.gnssC.gnss_frq[SYS_INDEX_QZS][1]=QZS_L2;C.gnssC.gnss_frq[SYS_INDEX_QZS][2]=QZS_L5;
    C.gnssC.gnss_frq[NSYS][0]=BDS_B1I;C.gnssC.gnss_frq[NSYS][1]=BDS_B3I;C.gnssC.gnss_frq[NSYS][2]=BDS_B1C;
    C.solC.out_err=true;
    C.solC.sol_coord=COORD_ENU;

    cSolver *solver= nullptr;
    solver=new cPppSolver(C);

    long t1=clock();

    if(!C.fileC.rover.empty()){
        cReadGnssObs rover_reader(C.fileC.rover,solver->nav_,solver->rover_obs_,REC_ROVER);
        rover_reader.SetGnssSysMask(C.gnssC.nav_sys);
        rover_reader.Reading();
    }

    if(!C.fileC.brd.empty()){
        cReadGnssBrdEph brd_reader(C.fileC.brd, solver->nav_);
        brd_reader.SetGnssSysMask(C.gnssC.nav_sys);
        brd_reader.Reading();
    }

    if(!C.fileC.cbias.empty()){
        cReadGnssCodeBias cbias_reader(C.fileC.cbias, solver->nav_);
        cbias_reader.Reading();
    }

    cReadRefSol *ref_reader= nullptr;
    if(C.mode_opt==MODE_OPT_KINEMATIC||C.mode>=MODE_IGLC){
        if(!C.fileC.ref.empty()&&C.solC.out_err){

            ref_reader=new cReadRefSol(C.fileC.ref,solver->ref_sols_);
            ref_reader->Reading();
        }
    }
    else if(C.mode_opt==MODE_OPT_KINE_SIM||C.mode_opt==MODE_OPT_STATIC){
        tSolInfoUnit ref;
        ref.pos<<5.08465761725042e+06,2.67032540511120e+06,-2.76848090433746e+06;
        ref.vel<<0,0,0;
        solver->ref_sols_.push_back(ref);
    }

    solver->SolverProcess(C);

    long t2=clock();
    double t=(double)(t2-t1)/CLOCKS_PER_SEC;
    cout<<t<<endl;
}
