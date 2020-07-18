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

    private:
        double w_,h_;
        bool show_flag_;
        string path_;
    };

    class cPlotSat:public cPlot{
    public:
        cPlotSat();
        cPlotSat(vector<tEpochSatUnit>gnss_obs);
        ~cPlotSat();

    private:
        void GetSatView();

    public:
        void PlotSatView();
        void PlotEpochSatNum();

    private:
        vector<tEpochSatUnit> obs_;
    };

}

#endif //PPPLIB_PLOTFUNC_H
