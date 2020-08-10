//
// Created by cc on 7/16/20.
//

#ifndef PPPLIB_PLOTFUNC_H
#define PPPLIB_PLOTFUNC_H

#include "CmnFunc.h"
#include "InsFunc.h"
#include "GnssFunc.h"
#include "matplotlibcpp.h"


namespace PPPLib {

    class cPlot{
    public:
        cPlot();
        ~cPlot();

    public:
        void SetFigureSize(int width,int height);
        void SetSavePath(string path);
        void SetPlotShow(bool flag);

        void PlotImuData(const vector<tImuDataUnit>& imus);

    public:
        double w_,h_;
        bool show_flag_;
        string path_;
    };

    class cPlotSat:public cPlot{
    public:
        cPlotSat();
        ~cPlotSat();

    private:
        void GetSatView();

    public:
        void PlotSatView();
        void PlotEpochSatNum();
        void PlotSatMeas(GNSS_OBS type,int epoch,tSatInfoUnit sat_info);
        void PlotMwAmb(tSatInfoUnit sat_info,int epoch);
    };

    class cPlotSol:public cPlot{
    public:
        cPlotSol();
        ~cPlotSol();

    public:
        void PlotSolPos(tSolInfoUnit sol,int epoch);
        void PlotSolVel(tSolInfoUnit sol,int epoch);
    };

}

#endif //PPPLIB_PLOTFUNC_H
