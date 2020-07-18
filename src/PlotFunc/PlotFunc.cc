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
        int n=imus.size();
        vector<double> t(n),ax(n),ay(n),az(n),gx(n),gy(n),gz(n);
        for(int i=0;i<n;++i){
            t.at(i)=i;
            ax.at(i)=imus[i].acce_[0];
            ay.at(i)=imus[i].acce_[1];
            az.at(i)=imus[i].acce_[2];
        }

        plt::figure_size(w_, h_);
        plt::subplot(3,1,1);
        plt::named_plot("acce_x",t, ax);
        plt::legend();
        plt::subplot(3,1,2);
        plt::named_plot("acce_y",t, ay);
        plt::legend();
        plt::subplot(3,1,3);
        plt::named_plot("acce_z",t, az);
        plt::legend();

        if(show_flag_) plt::show();
        else plt::save(path_);
    }

    cPlotSat::cPlotSat() {}

    cPlotSat::cPlotSat(vector<tEpochSatUnit> gnss_obs) {obs_=gnss_obs;}

    cPlotSat::~cPlotSat() {}

    void cPlotSat::PlotSatView() {
        int epoch_num=obs_.size(),i,j,sat_no;
        MatrixXd sat_view(epoch_num,MAX_SAT_NUM);
        sat_view<<MatrixXd::Zero(epoch_num,MAX_SAT_NUM);

        for(i=0;i<epoch_num;i++){
            for(j=0;j<obs_.at(i).sat_num;j++){
                sat_no=obs_.at(i).epoch_data.at(j).sat.sat_.no;
                sat_view(i,sat_no-1)=1;
            }
        }

        vector<int>no_zero_sat_no;
        for(i=0;i<MAX_SAT_NUM-1;i++){
            if(sat_view.col(i).mean()!=0.0) no_zero_sat_no.push_back(i+1);
        }

        MatrixXd no_zero_sat_view(epoch_num, no_zero_sat_no.size());
        for(i=0;i<no_zero_sat_no.size();i++){
            no_zero_sat_view.col(i)=sat_view.col(no_zero_sat_no.at(i)-1);
        }

        vector<int>t;
        for(i=1;i<epoch_num;i++) t.push_back(i);

//        plt::plot(t,no_zero_sat_view.col(0))
    }

    void cPlotSat::PlotEpochSatNum() {
        int epoch_num=obs_.size(),i,j,k;
        int sat_num[NSYS]={0};
        vector<int>epoch_sat_num[NSYS];
        vector<int>t;

        for(i=0;i<epoch_num;i++){
            t.push_back(i+1);
            for(j=0;j<obs_.at(i).sat_num;j++){
                switch(obs_.at(i).epoch_data.at(j).sat.sat_.sys){
                    case SYS_GPS: sat_num[SYS_INDEX_GPS]++; break;
                    case SYS_BDS: sat_num[SYS_INDEX_BDS]++; break;
                    case SYS_GAL: sat_num[SYS_INDEX_GAL]++; break;
                    case SYS_GLO: sat_num[SYS_INDEX_GLO]++; break;
                    case SYS_QZS: sat_num[SYS_INDEX_QZS]++; break;
                    case SYS_IRN: sat_num[SYS_INDEX_IRN]++; break;
                }
            }
            epoch_sat_num[SYS_INDEX_GPS].push_back(sat_num[SYS_INDEX_GPS]);
            epoch_sat_num[SYS_INDEX_BDS].push_back(sat_num[SYS_INDEX_BDS]);
            epoch_sat_num[SYS_INDEX_GAL].push_back(sat_num[SYS_INDEX_GAL]);
            epoch_sat_num[SYS_INDEX_GLO].push_back(sat_num[SYS_INDEX_GLO]);
            epoch_sat_num[SYS_INDEX_QZS].push_back(sat_num[SYS_INDEX_QZS]);
            epoch_sat_num[SYS_INDEX_IRN].push_back(sat_num[SYS_INDEX_IRN]);
            for(k=0;k<NSYS;k++) sat_num[k]=0;

        }

        string sys_flag[NSYS]={"GPS","BDS","GAL","GLO","QZS","IRN"};
        for(i=0;i<NSYS;i++){
            if(accumulate(begin(epoch_sat_num[i]),end(epoch_sat_num[i]),0.0)==0.0) continue;
            plt::named_plot(sys_flag[i],epoch_sat_num[i]);
            plt::legend();
        }
        plt::show();

    }



}