//
// Created by cc on 7/19/20.
//

#include "GnssErrorModel.h"

namespace PPPLib{

    cTrpDelayModel::cTrpDelayModel() {}

    cTrpDelayModel::~cTrpDelayModel() {}

    Vector2d cTrpDelayModel::GetSaasTrp(Vector3d blh,double el,double humi,Vector2d &sat_trp_dry,Vector4d &sat_trp_wet){
        blh_=blh;el_=el;
        SaasModel(humi);
        sat_trp_dry=slant_trp_dry_;
        sat_trp_wet=slant_trp_wet_;
        return zenith_trp_;
    }

    bool cTrpDelayModel::SaasModel(double humi) {
        const double temp0=15.0;
        double hgt,pres,temp,e,z;

        if(blh_[2]<-100.0||1E6<blh_[2]||el_<=0) return false;
        if(blh_[2]>=1.0/2.2557E-5) return false;
        hgt=blh_[2]<0.0?0.0:blh_[2];
        if(hgt>15000.0) hgt=15000.0;

        pres=1013.25*pow(1.0-2.2557E-5*hgt,5.2568);
        temp=temp0-6.5E-3*hgt+273.16;
        e=6.108*humi*exp((17.15*temp-4684.0)/(temp-38.45));
        z=PI/2.0-el_;
        zenith_trp_[0]=0.0022768*pres/(1.0-0.00266*cos(2.0*blh_[0])-0.00028*hgt/1E3);
        zenith_trp_[1]=0.0022770*(1255.0/temp+0.05)*e;
        double map=1.0/cos(z);
        slant_trp_dry_[0]=zenith_trp_[0]*map;
        slant_trp_dry_[1]=map;
        slant_trp_wet_[0]=zenith_trp_[1]*map;
        slant_trp_wet_[1]=map;
    }

    cIonDelayModel::cIonDelayModel() {}

    cIonDelayModel::~cIonDelayModel() {}

    cGnssErrorModel::cGnssErrorModel() {}

    cGnssErrorModel::cGnssErrorModel(PPPLib::tSatInfoUnit& sat_info) {sat_info_=sat_info;}

    cGnssErrorModel::~cGnssErrorModel() {}

    void cGnssErrorModel::ModelTrpError(cSat sat) {
        Vector2d trp_dry;
        Vector4d trp_wet;

//        trp_model_.GetSaasTrp()

    }


}