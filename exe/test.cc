//
// Created by cc on 7/9/20.
//

#include <cmath>
#include "matplotlibcpp.h"
#include "ReadFiles.h"
#include "PlotFunc.h"

INITIALIZE_EASYLOGGINGPP

namespace plt=matplotlibcpp;
using namespace PPPLib;

int test_read_ref() {
    string ref_file="../data/cpt00870.ref";
    cReadRefSol ref_reader(ref_file);
    ref_reader.Reading();

    int a=1;
}

int test_read_ionex() {
    string ion_file="../data/CODG3350.19I";
    tNav nav;
    cReadGnssIonex ion_reader(ion_file,nav);
    ion_reader.Reading();
    int a=1;
}

int test_read_antex(){
    string atx_file="../data/igs14_2097.atx";
    tNav nav;
    cReadGnssAntex atx_reader(atx_file,nav);
    atx_reader.Reading();
}

int test_read_ocean(){
    string blq_file="../data/ocnload.blq";
    tNav nav;
    cReadGnssOcean blq_reader(blq_file,nav,"KIRU",REC_ROVER);
    blq_reader.Reading();
    int a=1;
}

int test_read_erp() {
    string erp_file="../data/igs19P2082.erp";
    tNav nav;
    cReadGnssErp erp_reader(erp_file,nav);
    erp_reader.Reading();
    int a=1;
}

int test_read_mgex_dcb() {
    string dcb_file="../data/CAS0MGXRAP_20193350000_01D_01D_DCB.BSX";
    tNav nav;
    cReadGnssCodeBias cbias_reader(dcb_file,nav);

    cbias_reader.Reading();

    int a=1;
}

int test_read_pre(){
    string sp3_file="../data/wum20820.sp3";
    string clk_file="../data/wum20820.clk";
    tNav nav;
    cReadGnssPreEph sp3_reader(sp3_file,nav);
    sp3_reader.Reading(0);

    cReadGnssPreEph clk_reader(clk_file, nav);
    clk_reader.Reading(1);
}

int test_read_brd(){
    string file="../data/brdm3350.19p";
    tNav nav;
    cReadGnssBrdEph brd_reader(file,nav);
    brd_reader.SetGnssSysMask(SYS_GPS|SYS_GLO);

    brd_reader.Reading();
}


int test_plot_sat_view(){
    string file="../data/jfng3350.19o";
    tNav nav;
    cReadGnssObs obs_reader(file,nav,REC_ROVER);
    double es[6]={2019,12,1,0,00,00};
    double ee[6]={2019,12,1,23,00,00};
    cTime ts(es),te(ee);

    obs_reader.SetGnssSysMask(SYS_ALL);
    obs_reader.SetGnssTimeSpan(&ts,&te);

    obs_reader.Reading();

    cPlotSat plt_sat(obs_reader.GetGnssData()->GetGnssObs());
//    plt_sat.PlotSatView();
    plt_sat.PlotEpochSatNum();

}

int test_read_obs(){
    string file="../data/jfng3350.19o";
    tNav nav;
    cReadGnssObs obs_reader(file,nav,REC_ROVER);
    double es[6]={2019,12,1,0,00,00};
    double ee[6]={2019,12,1,23,59,30};
    cTime ts(es),te(ee);

    obs_reader.SetGnssSysMask(SYS_BDS);
    obs_reader.SetGnssTimeSpan(&ts,&te);

    obs_reader.Reading();
}


int test_plot_imu(vector<tImuDataUnit>& imu) {
    cPlot pt;

    pt.PlotImuData(imu);
}


int test_read_imu(){
    string file="../data/cpt00870.imu";
    cReadImu imu_reader(file);
    double es[6]={2019,3,28,3,15,00};
    double ee[6]={2019,3,28,3,35,00};
    cTime ts(es),te(ee);

    imu_reader.SetImuTimeSpan(&ts,&te);
    imu_reader.SetImuType(IMU_NOVTEL_CPT);
    imu_reader.SetImuCoordType(IMU_COORD_ENU);

    imu_reader.Reading();

    test_plot_imu(imu_reader.GetImus()->data_);

    string out_path="../data/imu.txt";
    imu_reader.OutImu(out_path);

}

int main(int argc, char** argv){
    string logini_path = SetLogConfPath("");
    int log_level = SetLogLevel(128);
    InitLog(argc,argv,logini_path, log_level);
//    test_read_imu();
//    test_read_obs();
//    test_read_brd();
//    test_read_pre();
//    test_read_mgex_dcb();
//    test_read_erp();
//      test_read_ocean();
//      test_read_antex();
//      test_read_ionex();
      test_read_ref();
//    test_plot_sat_view();
}
