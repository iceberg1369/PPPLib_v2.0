//
// Created by cc on 7/19/20.
//

#ifndef PPPLIB_GNSSERRORMODEL_H
#define PPPLIB_GNSSERRORMODEL_H

#include "GnssFunc.h"

using namespace PPPLib;

namespace PPPLib{

    class cGnssModel{
    public:
        cGnssModel();
        virtual ~cGnssModel();

    public:
        void InitErrModel(tPPPLibConf C,tNav nav);
        void InitSatInfo(tSatInfoUnit* sat_info,Vector3d* blh);
        virtual void UpdateSatInfo();

    public:
        tNav nav_;
        tSatInfoUnit* sat_info_;
        Vector3d blh_;
        tPPPLibConf PPPLibC_;
    };

    class cTrpDelayModel: public cGnssModel {
    public:
        cTrpDelayModel();
        cTrpDelayModel(Vector3d blh,tSatInfoUnit& sat_info);
        ~cTrpDelayModel();

    public:
        Vector2d GetSaasTrp(double humi,Vector2d* sat_trp_dry, Vector4d* sat_trp_wet);
        void UpdateSatInfo() override;

    private:
        bool SaasModel(double humi);
        Vector4d EstTrpWet(double humi);

    private:
        Vector2d zenith_trp_={0,0};                   //dry, wet
        Vector2d slant_trp_dry_={0,0};                //dry, map_dry
        Vector4d slant_trp_wet_={0,0,0,0};      //wet, map_wet,grand_e,grand_n
    };

    class cIonDelayModel: public cGnssModel {
    public:
        cIonDelayModel();
        cIonDelayModel(Vector3d blh,tSatInfoUnit& sat_info,tNav nav);
        ~cIonDelayModel();

    public:
        Vector2d GetKlobIon();
        void UpdateSatInfo() override;

    private:
        bool KlobModel();
        bool GimModel();
        bool IonFreeModel();
        bool IonEstModel();

    private:
        double* ion_para_;
        Vector2d ion_delay_;     // L1_ion/ion_map
    };

    class cCbiasModel:public cGnssModel {
    public:
        cCbiasModel();
        cCbiasModel(tNav nav,tSatInfoUnit& sat_info);
        ~cCbiasModel();

    public:
        void GetCodeBias();
        void UpdateSatInfo() override;

    private:
        void TgdModel();
        void BsxModel();

    private:
        double cbias_[NSYS+1][MAX_GNSS_FRQ_NUM];
    };

    class cGnssErrCorr {
    public:
        cGnssErrCorr();
        ~cGnssErrCorr();


    public:
        void InitGnssErrCorr(tPPPLibConf C, tNav nav);
        void BD2MultipathModel(tSatInfoUnit* sat_info);

    public:
        cCbiasModel    cbia_model_;
        cTrpDelayModel trp_model_;
        cIonDelayModel ion_model_;

    private:
        double bd2_mp_[3];
    };


}



#endif //PPPLIB_GNSSERRORMODEL_H
