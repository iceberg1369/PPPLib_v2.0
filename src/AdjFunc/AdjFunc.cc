//
// Created by cc on 7/26/20.
//

#include "AdjFunc.h"

namespace PPPLib {

    cAdjuster::cAdjuster() {}

    cAdjuster::~cAdjuster() {}

    int cAdjuster::Adjustment(VectorXd L, const MatrixXd H, const MatrixXd R, VectorXd &X, MatrixXd &Px, int nl, int nx) {}

    cLsqAdjuster::cLsqAdjuster() {}

    cLsqAdjuster::~cLsqAdjuster() {}

    int cLsqAdjuster::Adjustment(VectorXd L, const MatrixXd H, const MatrixXd R, VectorXd &X, MatrixXd &Px, int nl, int nx) {
        bool flag=true;
        /* dx=(APA)^-1*(APL) */
        Eigen::MatrixXd W=R.inverse();
        Eigen::MatrixXd HTWH=H*W*H.transpose();
        Eigen::MatrixXd HTWL=H*W*L;

        dx_=HTWH.inverse()*HTWL;
        for(int i=0;i<nx;i++) X[i]+=dx_[i];

        double n=dx_.norm();
        flag=dx_.norm()<1E-6;
        if(flag) v_=H.transpose()*dx_-L;

        return flag;
    }

    cKfAdjuster::cKfAdjuster() {}

    cKfAdjuster::~cKfAdjuster() {}

    int cKfAdjuster::Adjustment(VectorXd L, const MatrixXd H, const MatrixXd R, VectorXd &X, MatrixXd &Px, int nl, int nx) {
        Eigen::MatrixXd K=Px*H.transpose()*(H*Px*H.transpose()+R).inverse();
        dx_=K*L;
        Eigen::MatrixXd I=Eigen::MatrixXd::Identity(nx,nx);
        Px=(I-K*H)*Px*(I-K*H).transpose()+K*R*K.transpose();

        //保持方差正定
        Eigen::MatrixXd cov_fixed=(Px+Px.transpose())/2.0;
        Px=cov_fixed;

        for(int i=0;i<nx;i++) X[i]+=dx_[i];

        return true;
    }
}
