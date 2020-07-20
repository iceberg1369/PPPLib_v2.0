//
// Created by cc on 7/19/20.
//

#ifndef PPPLIB_GNSSERRORMODEL_H
#define PPPLIB_GNSSERRORMODEL_H

#include "GnssFunc.h"

using namespace PPPLib;

namespace PPPLib{

    class cTrpDelayModel {
    public:
        cTrpDelayModel();
        ~cTrpDelayModel();

    public:
        Vector2d GetSaasTrp(Vector3d blh,double el,double humi,Vector2d& sat_trp_dry, Vector4d& sat_trp_wet);

    private:
        bool SaasModel(double humi);
        Vector4d EstTrpWet(double humi);

    private:
        double el_;
        Vector3d blh_;
        Vector2d zenith_trp_={0,0};      // dry and wet
        Vector2d slant_trp_dry_={0,0};           //dry, map_dry
        Vector4d slant_trp_wet_={0,0,0,0}; //wet, map_wet,grand_e,grand_n
    };

    class cIonDelayModel {
    public:
        cIonDelayModel();
        ~cIonDelayModel();
    };

    class cGnssErrorModel {
    public:
        cGnssErrorModel();
        cGnssErrorModel(tSatInfoUnit& sat_info);
        ~cGnssErrorModel();

    public:
        void ModelGnssError();

    private:
        void ModelTrpError(cSat sat);

    private:
        tSatInfoUnit sat_info_;
        cTrpDelayModel trp_model_;
        cIonDelayModel ion_model_;
    };

}



#endif //PPPLIB_GNSSERRORMODEL_H
