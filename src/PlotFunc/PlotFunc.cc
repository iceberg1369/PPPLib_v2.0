//
// Created by cc on 7/16/20.
//
#include "PlotFunc.h"

namespace plt=matplotlibcpp;
namespace PPPLib {

    cPlot::cPlot() {
        w_=400;h_=300;
        show_flag_=true;
    }

    cPlot::~cPlot() {}

    void cPlot::SetFigureSize(int width, int height) {
        w_=width;
        h_=height;
    }

    void cPlot::SetSavePath(string path) {path_=path;}

    void cPlot::SetPlotShow(bool flag) {show_flag_=flag;}

    void cPlot::PlotImuData(const vector<tImuDataUnit>& imus) {
//        int n=imus.size();
//        vector<double> t(n),ax(n),ay(n),az(n),gx(n),gy(n),gz(n);
//        for(int i=0;i<n;++i){
//            t.at(i)=i;
//            ax.at(i)=imus[i].acce_[0];
//            ay.at(i)=imus[i].acce_[1];
//            az.at(i)=imus[i].acce_[2];
//        }
//
////        plt::figure_size(w_, h_);
////        plt::subplot(3,1,1);
////        plt::named_plot("acce_x",t, ax);
////        plt::legend();
////        plt::subplot(3,1,2);
////        plt::named_plot("acce_y",t, ay);
////        plt::legend();
////        plt::subplot(3,1,3);
////        plt::named_plot("acce_z",t, az);
////        plt::legend();
//
//        if(show_flag_) plt::show();
//        else plt::save(path_);
    }

    cPlotSat::cPlotSat() {}

    cPlotSat::~cPlotSat() {}

    void cPlotSat::PlotSatView() {

    }

    void cPlotSat::PlotEpochSatNum() {

    }

    void cPlotSat::PlotSatMeas(GNSS_OBS type,int epoch,tSatInfoUnit sat_info) {

        static vector<double>x,t;

        double meas=sat_info.raw_D[0];
        if(meas==0.0)
            return;

        x.push_back(sat_info.raw_D[0]);
        t.push_back(epoch);

        if(epoch%10==0){
            plt::figure(3);
            plt::ion();
            plt::clf();
            plt::named_plot("meas",t,x);
            plt::pause(0.01);
        }

    }

    void cPlotSat::PlotMwAmb(tSatInfoUnit sat_info,int epoch) {
        static vector<double>mw,smw,t;
        mw.push_back(sat_info.raw_mw[0]);
        smw.push_back(sat_info.sm_mw[0]);
        t.push_back(epoch);

        if(epoch%10==0){
            plt::figure(2);
            plt::ion();
            plt::clf();
            plt::named_plot("mw", t, mw);
            plt::named_plot("smw", t, smw);

            plt::title("Position Error [m]");
            plt::legend();
            plt::pause(0.01);
        }
    }

    cPlotSol::cPlotSol() {}

    cPlotSol::~cPlotSol() {}

    void cPlotSol::PlotSolPos(tSolInfoUnit sol,int epoch) {

#if 0
        vector<double> delta_e,delta_n,delta_u,t;
        for(int i=0;i<sols.size();i++){
            delta_e.push_back(sols.at(i).pos[0]);
            delta_n.push_back(sols.at(i).pos[1]);
            delta_u.push_back(sols.at(i).pos[2]);
            t.push_back(i);
        }

        plt::figure_size(w_, h_);
//        plt::named_plot("x",t, delta_x);
//        plt::named_plot("y",t, delta_y);
//        plt::named_plot("z",t, delta_z);
        plt::scatter(t, delta_e);
        plt::scatter(t, delta_n);
        plt::scatter(t, delta_u);

        delta_e.clear();
        delta_n.clear();
        delta_u.clear();

        plt::show();
#endif
        static vector<double>x, y,z,t;
        x.push_back(sol.pos[0]);
        y.push_back(sol.pos[1]);
        z.push_back(sol.pos[2]);
        t.push_back(epoch);

        if(epoch%10==0){
            plt::figure(1);
            plt::ion();
            plt::clf();
            plt::named_plot("east", t, x);
            plt::named_plot("north", t, y);
//            plt::named_plot("up", t, z);

            // Add graph title
            plt::title("Position Error [m]");
            // Enable legend.
            plt::legend();
            // Display plot continuously
            plt::pause(0.01);
        }
    }

    void cPlotSol::PlotSolVel(tSolInfoUnit sol, int epoch) {
        static vector<double>vx, vy,vz,t;
        vx.push_back(sol.vel[0]);
        vy.push_back(sol.vel[1]);
        vz.push_back(sol.vel[2]);
        t.push_back(epoch);

        if(epoch%10==0){
            plt::figure(2);
            plt::ion();
            plt::clf();
            plt::named_plot("east", t, vx);
            plt::named_plot("north", t, vy);
            plt::named_plot("up", t, vz);

            // Add graph title
            plt::title("Velocity Error [m]");
            // Enable legend.
            plt::legend();
            // Display plot continuously
            plt::pause(0.01);
        }
    }

}