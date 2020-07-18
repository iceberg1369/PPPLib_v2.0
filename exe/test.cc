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

int test_plot_sat_view(){
    string file="../data/jfng3350.19o";
    tNav nav;
    ReadGnssObs obs_reader(file,nav,REC_ROVER);
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
    ReadGnssObs obs_reader(file,nav,REC_ROVER);
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
    ReadImu imu_reader(file);
//    cTime ts("20,28,0,0,0");
//    cTime te("2019,3,28,0,0,0");
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
    test_plot_sat_view();
}
