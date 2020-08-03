//
// Created by cc on 7/26/20.
//

#ifndef PPPLIB_ADJFUNC_H
#define PPPLIB_ADJFUNC_H

#include "CmnFunc.h"

namespace PPPLib{

    class cAdjuster {
    public:
        cAdjuster();
        virtual ~cAdjuster();

    public:
        virtual int Adjustment(VectorXd L,const MatrixXd H,const MatrixXd R,VectorXd& X, MatrixXd& Px,int nl,int nx);

    public:
        VectorXd dx_;
        VectorXd v_;
    };

    class cLsqAdjuster:public cAdjuster{
    public:
        cLsqAdjuster();
        ~cLsqAdjuster();

    public:
        int Adjustment(VectorXd L,const MatrixXd H,const MatrixXd R,VectorXd& X, MatrixXd& Px,int nl,int nx);
    };

    class cKfAdjuster:public cAdjuster{
    public:
        cKfAdjuster();
        ~cKfAdjuster();

    public:
        int Adjustment(VectorXd L,const MatrixXd H,const MatrixXd R,VectorXd& X, MatrixXd& Px,int nl,int nx);
    };
}




#endif //PPPLIB_ADJFUNC_H
