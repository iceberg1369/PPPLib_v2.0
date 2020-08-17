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

        flag=dx_.norm()<1E-4;
        if(flag) v_=H.transpose()*dx_-L;

        return flag;
    }

    cKfAdjuster::cKfAdjuster() {}

    cKfAdjuster::~cKfAdjuster() {}

    int cKfAdjuster::Adjustment(VectorXd L, const MatrixXd H, const MatrixXd R, VectorXd &X, MatrixXd &Px, int nl, int nx) {

        using Eigen::MatrixXd;
        using Eigen::VectorXd;

        vector<int>zip_idx;
        int i,j;

        for(i=0;i<nx;i++){
            if(X[i]!=0.0&&Px(i,i)>0.0) zip_idx.push_back(i);
        }
        MatrixXd H_,Px_;
        VectorXd X_;
        H_=MatrixXd::Zero(zip_idx.size(),nl);Px_=MatrixXd::Zero(zip_idx.size(),zip_idx.size());X_=VectorXd::Zero(zip_idx.size());

        for(i=0;i<zip_idx.size();i++){
            X_[i]=X[zip_idx[i]];
            for(j=0;j<zip_idx.size();j++) Px_(i,j)=Px(zip_idx[i],zip_idx[j]);
            for(j=0;j<nl;j++) H_(i,j)=H(zip_idx[i],j);
        }

        cout<<endl;

        cout<<Px_.transpose()<<endl<<endl;

        cout<<H_.transpose()<<endl<<endl;

        cout<<R<<endl<<endl;

        MatrixXd K=Px_*H_*(H_.transpose()*Px_*H_+R).inverse();
        dx_=K*L;
        MatrixXd I=MatrixXd::Identity(zip_idx.size(),zip_idx.size());
//        Px_=(I-K*H_.transpose())*Px_*(I-K*H_.transpose()).transpose()+K*R*K.transpose();
        Px_=(I-K*H_.transpose())*Px_;

        cout<<X_.transpose()<<endl<<endl;
        cout<<Px_.transpose()<<endl<<endl;


        for(int i=0;i<zip_idx.size();i++) X_[i]+=dx_[i];

        for(i=0;i<zip_idx.size();i++){
            X[zip_idx[i]]=X_[i];
            for(int j=0;j<zip_idx.size();j++) Px(zip_idx[i],zip_idx[j])=Px_(i,j);
        }

        zip_idx.clear();
        return true;
    }
}
