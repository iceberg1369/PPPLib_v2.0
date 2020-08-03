//
// Created by cc on 8/3/20.
//

#ifndef PPPLIB_OUTSOL_H
#define PPPLIB_OUTSOL_H

#include "CmnFunc.h"
#include "PlotFunc.h"

namespace PPPLib {
    class cOutSol{
    public:
        cOutSol();
        cOutSol(tPPPLibConf C);
        cOutSol(tPPPLibConf C,vector<tSolInfoUnit>& ref_sols);
        ~cOutSol();

    private:
        int MatchRefSol(cTime sol_time);
        tSolInfoUnit CompareSol(tSolInfoUnit& sol,tSolInfoUnit& ref_sol);

    public:
        void WriteSol(tSolInfoUnit sol,int epoch);


    private:
        int ref_index_=0;
        tPPPLibConf C_;
        cPlotSol* plot_sol_;

    public:
        vector<tSolInfoUnit> ref_sols_;
    };
}


#endif //PPPLIB_OUTSOL_H
