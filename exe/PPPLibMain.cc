//
// Created by cc on 7/20/20.
//

#include "ReadFiles.h"
#include "Solver.h"
INITIALIZE_EASYLOGGINGPP

using namespace PPPLib;

int main(int argc, char** argv){
    string logini_path = SetLogConfPath("");
    int log_level = SetLogLevel(128);
    InitLog(argc,argv,logini_path, log_level);

    tPPPLibConf C;

    C.fileC.rover="../data/faa13350.19o";
    C.fileC.brd="../data/brdm3350.19p";
    C.fileC.clk="../data/wum20820.clk";
    C.fileC.sp3[1]="../data/wum20820.sp3";
    C.fileC.sp3[2]="../data/wum20821.sp3";
    C.fileC.cbias="../data/CAS0MGXRAP_20193350000_01D_01D_DCB.BSX";
    C.fileC.atx="../data/igs14_2097.atx";
    C.fileC.erp="../data/wum20820.erp";
    C.fileC.blq="../data/ocnload.blq";

    C.mode=MODE_SPP;
    C.gnssC.frq_opt=FRQ_DUAL;

    C.gnssC.gnss_frq[SYS_INDEX_GPS][0]=GPS_L1;C.gnssC.gnss_frq[SYS_INDEX_GPS][1]=GPS_L2;C.gnssC.gnss_frq[SYS_INDEX_GPS][2]=GPS_L5;
    C.gnssC.gnss_frq[SYS_INDEX_BDS][0]=BDS_B1I;C.gnssC.gnss_frq[SYS_INDEX_BDS][1]=BDS_B2I;C.gnssC.gnss_frq[SYS_INDEX_BDS][2]=BDS_B3I;
    C.gnssC.gnss_frq[SYS_INDEX_GAL][0]=GAL_E1;C.gnssC.gnss_frq[SYS_INDEX_GAL][1]=GAL_E5a;C.gnssC.gnss_frq[SYS_INDEX_GAL][2]=GAL_E5b;
    C.gnssC.gnss_frq[SYS_INDEX_GLO][0]=GLO_G1;C.gnssC.gnss_frq[SYS_INDEX_GLO][1]=GLO_G2;
    C.gnssC.gnss_frq[SYS_INDEX_QZS][0]=QZS_L1;C.gnssC.gnss_frq[SYS_INDEX_QZS][1]=QZS_L2;C.gnssC.gnss_frq[SYS_INDEX_QZS][2]=QZS_L5;
    C.gnssC.gnss_frq[NSYS][0]=BDS_B1I;C.gnssC.gnss_frq[NSYS][1]=BDS_B3I;C.gnssC.gnss_frq[NSYS][2]=BDS_B1C;

    cSolver *solver= nullptr;

    if(C.mode==MODE_SPP) solver=new cSppSolver(C);


    long t1=clock();
    cReadGnssObs rover_reader(C.fileC.rover,solver->nav_,solver->rover_obs_,REC_ROVER);
    rover_reader.SetGnssSysMask(SYS_BDS);
    rover_reader.Reading();

    cReadGnssBrdEph brd_reader(C.fileC.brd, solver->nav_);
    brd_reader.Reading();

    cReadGnssCodeBias cbias_reader(C.fileC.cbias, solver->nav_);
    cbias_reader.Reading();

    cReadGnssPreEph clk_reader(C.fileC.clk,solver->nav_);
    clk_reader.Reading(1);

    cReadGnssPreEph orb_reader(C.fileC.sp3[1], solver->nav_);
    for(int i=0;i<3;i++){
        if(C.fileC.sp3[i].empty()) continue;
        orb_reader.file_=C.fileC.sp3[i];
        orb_reader.Reading(0);
    }

    cReadGnssErp erp_reader(C.fileC.erp,solver->nav_);
    erp_reader.Reading();

    cReadGnssAntex atx_reader(C.fileC.atx,solver->nav_);
    atx_reader.Reading();

    cReadGnssOcean blq_reader(C.fileC.blq, solver->nav_,"JFNG",REC_ROVER);
    blq_reader.Reading();

    long t2=clock();
    double t=(double)(t2-t1)/CLOCKS_PER_SEC;
    cout<<t<<endl;

    solver->SolverProcess();
}
