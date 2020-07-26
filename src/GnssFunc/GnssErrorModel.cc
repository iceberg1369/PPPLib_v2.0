//
// Created by cc on 7/19/20.
//

#include "GnssErrorModel.h"

namespace PPPLib{

    cGnssModel::cGnssModel() {}

    cGnssModel::~cGnssModel() {}

    void cGnssModel::UpdateSatInfo() {}

    void cGnssModel::InitErrModel(tPPPLibConf C, tNav nav) {
        PPPLibC_=C;
        nav_=nav;
    }

    void cGnssModel::InitSatInfo(tSatInfoUnit *sat_info, Vector3d* blh) {
        sat_info_=sat_info;
        if(blh) blh_=*blh;
    }

    cTrpDelayModel::cTrpDelayModel() {}

    cTrpDelayModel::cTrpDelayModel(Vector3d blh, tSatInfoUnit &sat_info) {
        blh_=blh;
        sat_info_=&sat_info;
    }

    cTrpDelayModel::~cTrpDelayModel() {}

    Vector2d cTrpDelayModel::GetSaasTrp(double humi,Vector2d* sat_trp_dry,Vector4d* sat_trp_wet){
        SaasModel(humi);
        if(sat_trp_dry) sat_trp_dry=&slant_trp_dry_;
        if(sat_trp_wet) sat_trp_wet=&slant_trp_wet_;
        return zenith_trp_;
    }

    void cTrpDelayModel::UpdateSatInfo() {
        sat_info_->trp_dry_delay=slant_trp_dry_;
        sat_info_->trp_wet_delay=slant_trp_wet_;
    }

    bool cTrpDelayModel::SaasModel(double humi) {
        const double temp0=15.0;
        double hgt,pres,temp,e,z;
        double el=sat_info_->el_az[0];

        if(blh_[2]<-100.0||1E6<blh_[2]||el<=0) return false;
        if(blh_[2]>=1.0/2.2557E-5) return false;
        hgt=blh_[2]<0.0?0.0:blh_[2];
        if(hgt>15000.0) hgt=15000.0;

        pres=1013.25*pow(1.0-2.2557E-5*hgt,5.2568);
        temp=temp0-6.5E-3*hgt+273.16;
        e=6.108*humi*exp((17.15*temp-4684.0)/(temp-38.45));
        z=PI/2.0-el;
        zenith_trp_[0]=0.0022768*pres/(1.0-0.00266*cos(2.0*blh_[0])-0.00028*hgt/1E3);
        zenith_trp_[1]=0.0022770*(1255.0/temp+0.05)*e;
        double map=1.0/cos(z);
        slant_trp_dry_[0]=zenith_trp_[0]*map;
        slant_trp_dry_[1]=map;
        slant_trp_wet_[0]=zenith_trp_[1]*map;
        slant_trp_wet_[1]=map;
    }

    cIonDelayModel::cIonDelayModel() {}

    cIonDelayModel::cIonDelayModel(Vector3d blh, tSatInfoUnit &sat_info, tNav nav) {
        blh_=blh;
        sat_info_=&sat_info;
        ion_para_=nav.ion_para[SYS_GPS];
    }

    cIonDelayModel::~cIonDelayModel() {}

    Vector2d cIonDelayModel::GetKlobIon() {
        KlobModel();
        return ion_delay_;
    }

    void cIonDelayModel::UpdateSatInfo() {
        sat_info_->ion_delay=ion_delay_;
    }

    bool cIonDelayModel::KlobModel() {
        const double ion_default[]={ /* 2004/1/1 */
                0.1118E-07,-0.7451E-08,-0.5961E-07, 0.1192E-06,
                0.1167E+06,-0.2294E+06,-0.1311E+06, 0.1049E+07
        };
        double tt,psi,phi,lam,amp,per,x;
        double el=sat_info_->el_az[0];
        double az=sat_info_->el_az[1];
        int week;
        cTime obs_time=sat_info_->t_tag;

        if (blh_[2]<-1E3||el<=0) return false;

        VectorXd ion_par=VectorXd::Map(nav_.ion_para[SYS_INDEX_GPS],8);
        if(ion_par.norm()<=0.0) ion_par=VectorXd::Map(ion_default,8);

        /* earth centered angle (semi-circle) */
        psi=0.0137/(el/PI+0.11)-0.022;

        /* subionospheric latitude/longitude (semi-circle) */
        phi=blh_[0]/PI+psi*cos(az);
        // latitude boundary protection
        if (phi> 0.416) phi= 0.416;
        else if (phi<-0.416) phi=-0.416;
        lam=blh_[1]/PI+psi*sin(az)/cos(phi*PI);

        /* geomagnetic latitude (semi-circle) */
        phi+=0.064*cos((lam-1.617)*PI);

        /* local time (s) */
        tt=43200.0*lam+obs_time.Time2Gpst(&week, nullptr,SYS_GPS);
        tt-=floor(tt/86400.0)*86400.0; /* 0<=tt<86400 */

        /* slant factor */
        double map_ion=0.0;
        map_ion=sat_info_->ion_delay[1]=1.0+16.0*pow(0.53-el/PI,3.0);

        /* ionospheric delay */
        amp=ion_par[0]+phi*(ion_par[1]+phi*(ion_par[2]+phi*ion_par[3]));
        per=ion_par[4]+phi*(ion_par[5]+phi*(ion_par[6]+phi*ion_par[7]));
        amp=amp<0.0?0.0:amp;
        per=per<72000.0?72000.0:per;
        x=2.0*PI*(tt-50400.0)/per;

        // GPS L1
        double sion=CLIGHT*map_ion*(fabs(x)<1.57 ? 5E-9+amp*(1.0+x*x*(-0.5+x*x/24.0)):5E-9);

        double lam_G1=CLIGHT/FREQ_GPS_L1;
        ion_delay_[0]=sion*SQR(sat_info_->lam[0]/lam_G1);
        ion_delay_[1]=map_ion;

        return true;
    }

    bool cIonDelayModel::IonFreeModel() {
        ion_para_[0]=0.0;
        ion_delay_[1]=0.0;
    }

    cCbiasModel::cCbiasModel() {}

    cCbiasModel::cCbiasModel(tNav nav,tSatInfoUnit& sat_info) {
        nav_=nav;
        sat_info_=&sat_info;
    }

    cCbiasModel::~cCbiasModel() {}

    void cCbiasModel::GetCodeBias() {
        if(PPPLibC_.fileC.cbias.empty()){
            TgdModel();
            return;
        }
        BsxModel();
    }

    void cCbiasModel::UpdateSatInfo() {
        int sys=sat_info_->sat.sat_.sys;
        int sys_idx=sat_info_->sat.sat_.sys_idx;
        int *frqs=PPPLibC_.gnssC.gnss_frq[sys_idx];
        double *bias=cbias_[sys_idx];

        for(int i=0;i<MAX_GNSS_FRQ_NUM;i++){
            sat_info_->code_bias[i]=0.0;
        }

        if(sys==SYS_BDS&&sat_info_->sat.sat_.prn>18){
            frqs=PPPLibC_.gnssC.gnss_frq[NSYS];
            bias=cbias_[NSYS];

        }

        sat_info_->code_bias[0]=bias[frqs[0]];
        if(PPPLibC_.gnssC.frq_opt==FRQ_DUAL){
            sat_info_->code_bias[0]=bias[frqs[0]];
            sat_info_->code_bias[1]=bias[frqs[1]];
        }
        else if(PPPLibC_.gnssC.frq_opt==FRQ_TRIPLE){
            sat_info_->code_bias[0]=bias[frqs[0]];
            sat_info_->code_bias[1]=bias[frqs[1]];
            sat_info_->code_bias[2]=bias[frqs[2]];
        }
    }

    void cCbiasModel::TgdModel() {
        int i;
        double gamma=0.0;
        double tgd1=0.0,tgd2=0.0;

        for(i=0;i<NSYS+1;i++){
            for(int j=0;j<MAX_GNSS_FRQ_NUM;j++){
                cbias_[i][j]=0.0;
            }
        }

        for(i=0;i<nav_.brd_eph.size();i++){
            if(nav_.brd_eph.at(i).sat.sat_.no!=sat_info_->sat.sat_.no) continue;
            tgd1=CLIGHT*(nav_.brd_eph.at(i).tgd[0]);
            tgd2=CLIGHT*(nav_.brd_eph.at(i).tgd[1]);
            break;
        }
        if(sat_info_->sat.sat_.sys==SYS_GPS||sat_info_->sat.sat_.sys==SYS_QZS){
            gamma=SQR(FREQ_GPS_L1/FREQ_GPS_L2);
            cbias_[SYS_INDEX_GPS][0]=tgd1;                        // L1   tgd=dcb/(1.0-gamma)
            cbias_[SYS_INDEX_GPS][1]=tgd1*gamma;                  // L2
        }
        else if(sat_info_->sat.sat_.sys==SYS_GAL){
            gamma=SQR(FREQ_GAL_E5A/FREQ_GAL_E1);
            cbias_[SYS_INDEX_GAL][0]=gamma*tgd1;                   // E1
            cbias_[SYS_INDEX_GAL][1]=tgd1;                         // E5a
            cbias_[SYS_INDEX_GAL][2]=gamma*tgd1+(1.0-gamma)*tgd2;  // E5b
        }
        else if(sat_info_->sat.sat_.sys==SYS_BDS){
            if(sat_info_->sat.sat_.prn>18){
                cbias_[NSYS][0]=-tgd1;                             // BD3-B1I
                cbias_[NSYS][1]=0.0;
                cbias_[NSYS][2]=0.0;                               // BD3-B3I
            }
            else{
                cbias_[SYS_INDEX_BDS][0]=-tgd1;                    // BD2-B1I
                cbias_[SYS_INDEX_BDS][1]=-tgd2;                    // BD2-B2I
                cbias_[SYS_INDEX_BDS][2]=0.0;                      // BD2-B3I
            }
        }
    }

    void cCbiasModel::BsxModel() {
        int sys=sat_info_->sat.sat_.sys;
        int sat_no=sat_info_->sat.sat_.no;
        int sys_idx=sat_info_->sat.sat_.sys_idx;

        if(sys==SYS_BDS&&sat_info_->sat.sat_.prn>18) sys_idx=NSYS;

        for(int j=0;j<MAX_GNSS_FRQ_NUM;j++)
            cbias_[sys_idx][j]=0.0;

        if(sys==SYS_GPS){
            double DCB_p1p2=nav_.code_bias[sat_no-1][GPS_C1WC2W];
            double a=(SQR(FREQ_GPS_L1)-SQR(FREQ_GPS_L2));
            double alpha=SQR(FREQ_GPS_L1)/a;
            double beta=-SQR(FREQ_GPS_L2)/a;

            for(int i=0;i<MAX_GNSS_FRQ_NUM;i++){
                if(sat_info_->P_code[i]==GNSS_CODE_NONE){
                    cbias_[SYS_INDEX_GPS][i]=0.0;
                    continue;
                }
                if(i==0){
                    cbias_[SYS_INDEX_GPS][0]=beta*DCB_p1p2;     // L1
                    if(sat_info_->P_code[i]==GNSS_CODE_L1C) cbias_[SYS_INDEX_GPS][i]+=nav_.code_bias[sat_no-1][GPS_C1CC1W];
                }
                else if(i==1){
                    cbias_[SYS_INDEX_GPS][1]=-alpha*DCB_p1p2;   // L2
                    if(sat_info_->P_code[i]==GNSS_CODE_L2C) cbias_[SYS_INDEX_GPS][i]+=nav_.code_bias[sat_no-1][GPS_C2CC2W];
                    else if(sat_info_->P_code[i]==GNSS_CODE_L2S) cbias_[SYS_INDEX_GPS][i]-=nav_.code_bias[sat_no-1][GPS_C2WC2S];
                    else if(sat_info_->P_code[i]==GNSS_CODE_L2L) cbias_[SYS_INDEX_GPS][i]-=nav_.code_bias[sat_no-1][GPS_C2WC2L];
                    else if(sat_info_->P_code[i]==GNSS_CODE_L2X) cbias_[SYS_INDEX_GPS][i]-=nav_.code_bias[sat_no-1][GPS_C2WC2X];
                }
                else if(i==2){
                    double DCB_p1p3=0.0;
                    double beta_13=-SQR(FREQ_GPS_L5)/(SQR(FREQ_GPS_L1)-SQR(FREQ_GPS_L5));
                    if(sat_info_->P_code[i]==GNSS_CODE_L5Q) DCB_p1p3=nav_.code_bias[sat_no-1][GPS_C1CC5Q]-nav_.code_bias[sat_no-1][GPS_C1CC1W];
                    else if(sat_info_->P_code[i]==GNSS_CODE_L5X) DCB_p1p3=nav_.code_bias[sat_no-1][GPS_C1CC5X]-nav_.code_bias[sat_no-1][GPS_C1CC1W];
                    cbias_[SYS_INDEX_GPS][2]=beta_13*DCB_p1p2-DCB_p1p3;
                }
            }
        }
        else if(sys==SYS_BDS){
            if(PPPLibC_.gnssC.eph_opt==EPH_BRD){
                // base on b3
                if(sat_info_->sat.sat_.prn>18){
                    //BD3
                    double DCB_b1b3=nav_.code_bias[sat_no-1][BD3_C2IC6I];
                    for(int i=0;i<MAX_GNSS_FRQ_NUM;i++){
                        if(sat_info_->P_code[i]==GNSS_CODE_NONE){
                            cbias_[SYS_INDEX_BDS][i]=0.0;
                            continue;
                        }
                        if(i==0){
                            cbias_[NSYS][i]=DCB_b1b3;
                        }
                        else if(i==1){
                            cbias_[NSYS][i]=0.0;
                        }
                        else if(i==2){
                            cbias_[NSYS][i]=0.0;
                        }
                        else if(i==3){
                            //B1C
                            if(sat_info_->P_code[i]==GNSS_CODE_L1X)  DCB_b1b3=nav_.code_bias[sat_no-1][BD3_C1XC6I];
                            else if(sat_info_->P_code[i]==GNSS_CODE_L1P) DCB_b1b3=nav_.code_bias[sat_no-1][BD3_C1PC6I];
                            else if(sat_info_->P_code[i]==GNSS_CODE_L1D) DCB_b1b3=nav_.code_bias[sat_no-1][BD3_C1DC6I];
                            cbias_[NSYS][i]=DCB_b1b3;
                        }
                        else if(i==4){
                            //B2a
                            double DCB_b2b3=0.0;
                            if(sat_info_->P_code[i]==GNSS_CODE_L5X) DCB_b2b3=nav_.code_bias[sat_no-1][BD3_C1XC6I]-nav_.code_bias[sat_no-1][BD3_C1XC5X];
                            else if(sat_info_->P_code[i]==GNSS_CODE_L5P) DCB_b2b3=nav_.code_bias[sat_no-1][BD3_C1PC6I]-nav_.code_bias[sat_no-1][BD3_C1PC5P];
                            else if(sat_info_->P_code[i]==GNSS_CODE_L5D) DCB_b2b3=nav_.code_bias[sat_no-1][BD3_C1DC6I]-nav_.code_bias[sat_no-1][BD3_C1DC5D];
                            cbias_[NSYS][i]=DCB_b2b3;
                        }
                        else if(i==5){
                            //B2b
                            cbias_[NSYS][i]=0.0;
                        }
                    }
                }
                else{
                    //BD2
                    double DCB_b1b2=nav_.code_bias[sat_no-1][BD2_C2IC7I];
                    double DCB_b1b3=nav_.code_bias[sat_no-1][BD2_C2IC6I];
                    for(int i=0;i<MAX_GNSS_FRQ_NUM;i++){
                        if(sat_info_->P_code[i]==GNSS_CODE_NONE){
                            cbias_[SYS_INDEX_BDS][i]=0.0;
                            continue;
                        }
                        if(i==0){
                            cbias_[SYS_INDEX_BDS][i]=DCB_b1b3;
                        }
                        else if(i==1){
                            cbias_[SYS_INDEX_BDS][i]=DCB_b1b3-DCB_b1b2;
                        }
                        else if(i==2){
                            cbias_[SYS_INDEX_BDS][i]=0.0;
                        }
                    }
                }
            }
            else if(PPPLibC_.gnssC.eph_opt==EPH_PRE){
                if(PPPLibC_.gnssC.ac_opt==AC_COM){
                    // base on b1b2
                    for(int i=0;i<MAX_GNSS_FRQ_NUM;i++){
                        if(sat_info_->P_code[i]==GNSS_CODE_NONE){
                            cbias_[SYS_INDEX_BDS][i]=0.0;
                            continue;
                        }
                        if(sat_info_->sat.sat_.prn>18){
                            cbias_[SYS_INDEX_BDS][i]=0.0;
                            continue;
                        }
                        double DCB_b1b2=nav_.code_bias[sat_no-1][BD2_C2IC7I];
                        double DCB_b1b3=nav_.code_bias[sat_no-1][BD2_C2IC6I];
                        double a=(SQR(FREQ_BDS_B1)-SQR(FREQ_BDS_B2));
                        double alpha=SQR(FREQ_BDS_B1)/a;
                        double beta=-SQR(FREQ_BDS_B2)/a;
                        if(i==0){
                            cbias_[SYS_INDEX_BDS][i]=beta*DCB_b1b2;
                        }
                        else if(i==1){
                            cbias_[SYS_INDEX_BDS][i]=-alpha*DCB_b1b2;
                        }
                        else if(i==2){
                            double beta_13=-SQR(FREQ_BDS_B3)/(SQR(FREQ_BDS_B1)-SQR(FREQ_BDS_B3));
                            cbias_[SYS_INDEX_BDS][i]=beta_13*DCB_b1b2-DCB_b1b3;
                        }
                    }
                }
                else if(PPPLibC_.gnssC.ac_opt==AC_WUM||PPPLibC_.gnssC.ac_opt==AC_GBM){
                    // base on b1b3
                    double a=(SQR(FREQ_BDS_B1)-SQR(FREQ_BDS_B3));
                    double alpha=SQR(FREQ_BDS_B1)/a;
                    double beta=-SQR(FREQ_BDS_B3)/a;
                    for(int i=0;i<MAX_GNSS_FRQ_NUM;i++){
                        if(sat_info_->P_code[i]==GNSS_CODE_NONE){
                            cbias_[SYS_INDEX_BDS][i]=0.0;
                            continue;
                        }
                        if(sat_info_->sat.sat_.prn>18){
                            double DCB_b1b3=nav_.code_bias[sat_no-1][BD3_C2IC6I];
                            if(i==0){
                                cbias_[SYS_INDEX_BDS][i]=beta*DCB_b1b3;
                            }
                            else if(i==1){
                                cbias_[SYS_INDEX_BDS][i]=0.0;
                            }
                            else if(i==2){
                                cbias_[SYS_INDEX_BDS][i]=-alpha*DCB_b1b3;
                            }
                            else if(i==3){
                                //B1C
                                double beta_13=-SQR(FREQ_BDS_B1C)/(SQR(FREQ_BDS_B1)-SQR(FREQ_BDS_B3));
                                double DCB_b1b2=0.0;
                                if(sat_info_->P_code[i]==GNSS_CODE_L1X) DCB_b1b2=nav_.code_bias[sat_no-1][BD3_C1XC6I];
                                else if(sat_info_->P_code[i]==GNSS_CODE_L1P) DCB_b1b2=nav_.code_bias[sat_no-1][BD3_C1PC6I];
                                else if(sat_info_->P_code[i]==GNSS_CODE_L1D) DCB_b1b2=nav_.code_bias[sat_no-1][BD3_C1DC6I];
                                double dcb=DCB_b1b3-DCB_b1b2;
                                cbias_[SYS_INDEX_BDS][i]=beta_13*DCB_b1b3-dcb;
                            }
                            else if(i==4){
                                //B2a
                                double beta_13=-SQR(FREQ_BDS_B2A)/(SQR(FREQ_BDS_B1)-SQR(FREQ_BDS_B2A));
                                double DCB_b1b2=0.0;
                                if(sat_info_->P_code[i]==GNSS_CODE_L5X) DCB_b1b2=nav_.code_bias[sat_no-1][BD3_C1XC6I]-nav_.code_bias[sat_no-1][BD3_C1XC5X];
                                else if(sat_info_->P_code[i]==GNSS_CODE_L5P) DCB_b1b2=nav_.code_bias[sat_no-1][BD3_C1PC6I]-nav_.code_bias[sat_no-1][BD3_C1PC5P];
                                else if(sat_info_->P_code[i]==GNSS_CODE_L5D) DCB_b1b2=nav_.code_bias[sat_no-1][BD3_C1DC6I]-nav_.code_bias[sat_no-1][BD3_C1DC5D];
                                double dcb=DCB_b1b3-DCB_b1b2;
                                cbias_[SYS_INDEX_BDS][i]=beta_13*DCB_b1b3-dcb;
                            }
                            else if(i==5){
                                //B2b
                                cbias_[SYS_INDEX_BDS][i]=0.0;
                            }

                        }
                        else{
                            double DCB_b1b2=nav_.code_bias[sat_no-1][BD2_C2IC7I];
                            double DCB_b1b3=nav_.code_bias[sat_no-1][BD2_C2IC6I];
                            if(i==0){
                                cbias_[SYS_INDEX_BDS][i]=beta*DCB_b1b3;
                            }
                            else if(i==1){
                                double beta_13=-SQR(FREQ_BDS_B2)/(SQR(FREQ_BDS_B1)-SQR(FREQ_BDS_B2));
                                cbias_[SYS_INDEX_BDS][i]=beta_13*DCB_b1b3-DCB_b1b2;
                            }
                            else if(i==2){
                                cbias_[SYS_INDEX_BDS][i]=-alpha*DCB_b1b3;
                            }
                        }
                    }
                }
            }
        }
        else if(sys==SYS_GAL){
            double DCB_p1p2=nav_.code_bias[sat_no-1][GAL_C1CC5Q];
            double a=(SQR(FREQ_GAL_E1)-SQR(FREQ_GAL_E5A));
            double alpha=SQR(FREQ_GAL_E1)/a;
            double beta=-SQR(FREQ_GAL_E5A)/a;

            for(int i=0;i<MAX_GNSS_FRQ_NUM;i++){
                if(sat_info_->P_code[i]==GNSS_CODE_NONE){
                    cbias_[SYS_INDEX_GAL][i]=0.0;
                    continue;
                }
                if(i==0){
                    if(sat_info_->P_code[i]==GNSS_CODE_L1X) DCB_p1p2=nav_.code_bias[sat_no-1][GAL_C1XC5X];
                    cbias_[SYS_INDEX_GAL][i]=beta*DCB_p1p2;
                }
                else if(i==1){
                    if(sat_info_->P_code[i]==GNSS_CODE_L5X) DCB_p1p2=nav_.code_bias[sat_no-1][GAL_C1XC5X];
                    cbias_[SYS_INDEX_GAL][i]=-alpha*DCB_p1p2;
                }
                else if(i==2){
                    double DCB_p1p3=0.0;
                    double beta_13=-SQR(FREQ_GAL_E5B)/(SQR(FREQ_GAL_E1)-SQR(FREQ_GAL_E5B));
                    if(sat_info_->P_code[i]==GNSS_CODE_L7X){
                        DCB_p1p2=nav_.code_bias[sat_no-1][GAL_C1XC5X];
                        DCB_p1p3=nav_.code_bias[sat_no-1][GAL_C1XC7X];
                    }
                    else if(sat_info_->P_code[i]==GNSS_CODE_L7Q) DCB_p1p3=nav_.code_bias[sat_no-1][GAL_C1CC7Q];
                    cbias_[SYS_INDEX_GAL][i]=beta_13*DCB_p1p2-DCB_p1p3;
                }
            }
        }
        else if(sys==SYS_GLO){
            double DCB_p1p2=nav_.code_bias[sat_no-1][GLO_C1PC2P];
            double a=(SQR(FREQ_GLO_G1)-SQR(FREQ_GLO_G2));
            double alpha=SQR(FREQ_GLO_G1)/a;
            double beta=-SQR(FREQ_GLO_G2)/a;

            for(int i=0;i<MAX_GNSS_FRQ_NUM;i++){
                if(sat_info_->P_code[i]==GNSS_CODE_NONE){
                    cbias_[SYS_INDEX_GLO][i]=0.0;
                    continue;
                }
                if(i==0){
                    cbias_[SYS_INDEX_GLO][i]=beta*DCB_p1p2;
                    if(sat_info_->P_code[i]==GNSS_CODE_L1C) cbias_[SYS_INDEX_GLO][i]+=nav_.code_bias[sat_no-1][GLO_C1CC1P];
                }
                else if(i==1){
                    cbias_[SYS_INDEX_GLO][i]=-alpha*DCB_p1p2;
                    if(sat_info_->P_code[i]==GNSS_CODE_L2C) cbias_[SYS_INDEX_GLO][i]+=nav_.code_bias[sat_no-1][GLO_C2CC2P];
                }
            }
        }
        else if(sys==SYS_QZS){
            double DCB_p1p2=nav_.code_bias[sat_no-1][QZS_C1CC2L];
            double a=(SQR(FREQ_QZS_L1)-SQR(FREQ_QZS_L2));
            double alpha=SQR(FREQ_QZS_L1)/a;
            double beta=-SQR(FREQ_QZS_L2)/a;

            for(int i=0;i<MAX_GNSS_FRQ_NUM;i++){
                if(sat_info_->P_code[i]==GNSS_CODE_NONE){
                    cbias_[SYS_INDEX_QZS][i]=0.0;
                    continue;
                }
                if(i==0){
                    if(sat_info_->P_code[i]==GNSS_CODE_L1X) DCB_p1p2=nav_.code_bias[sat_no-1][QZS_C1XC2X];
                    cbias_[SYS_INDEX_QZS][i]=beta*DCB_p1p2;
                }
                else if(i==1){
                    if(sat_info_->P_code[i]==GNSS_CODE_L2X) DCB_p1p2=nav_.code_bias[sat_no-1][QZS_C1XC2X];
                    cbias_[SYS_INDEX_QZS][i]=-alpha*DCB_p1p2;
                }
                else if(i==2){
                    if(sat_info_->P_code[0]==GNSS_CODE_L1X) DCB_p1p2=nav_.code_bias[sat_no-1][QZS_C1XC2X];
                    double beta_13=-SQR(FREQ_QZS_L1)/(SQR(FREQ_QZS_L1)-SQR(FREQ_QZS_L5));
                    double DCB_p1p3=0.0;
                    if(sat_info_->P_code[i]==GNSS_CODE_L5Q) DCB_p1p3=nav_.code_bias[sat_no-1][QZS_C1CC5Q];
                    else if(sat_info_->P_code[i]==GNSS_CODE_L5X) DCB_p1p3=nav_.code_bias[sat_no-1][QZS_C1CC5X];
                    cbias_[SYS_INDEX_QZS][i]=beta_13*DCB_p1p2-DCB_p1p3;
                }
            }
        }
    }

    cGnssErrCorr::cGnssErrCorr() {}

    cGnssErrCorr::~cGnssErrCorr() {}

    void cGnssErrCorr::InitGnssErrCorr(tPPPLibConf C, tNav nav) {
        trp_model_.InitErrModel(C,nav);
        ion_model_.InitErrModel(C,nav);
        cbia_model_.InitErrModel(C,nav);
    }


    void cGnssErrCorr::BD2MultipathModel(tSatInfoUnit* sat_info) {
        const static double kBDIGSOCoef[3][10]={
                {-0.55,-0.40,-0.34,-0.23,-0.15,-0.04,0.09,0.19,0.27,0.35},	//B1
                {-0.71,-0.36,-0.33,-0.19,-0.14,-0.03,0.08,0.17,0.24,0.33},	//B2
                {-0.27,-0.23,-0.21,-0.15,-0.11,-0.04,0.05,0.14,0.19,0.32},	//B3
        };
        const static double kBDMEOCoef[3][10]={
                {-0.47,-0.38,-0.32,-0.23,-0.11,0.06,0.34,0.69,0.97,1.05},	//B1
                {-0.40,-0.31,-0.26,-0.18,-0.06,0.09,0.28,0.48,0.64,0.69},	//B2
                {-0.22,-0.15,-0.13,-0.10,-0.04,0.05,0.14,0.27,0.36,0.47},	//B3
        };

        double el=sat_info->el_az[0]*R2D*0.1;
        int int_el=(int)el;
        int prn=sat_info->sat.sat_.prn;
        int BD2_IGSO,BD2_MEO;
        double mp[3]={0};

        BD2_IGSO=std::binary_search(kBD2_IGSO,kBD2_IGSO+NUM_BD2_IGSO,prn);
        BD2_MEO=std::binary_search(kBD2_MEO,kBD2_MEO+NUM_BD2_MEO,prn);

        if(!BD2_IGSO&&!BD2_MEO) return;

        if(BD2_IGSO){
            if(el<0) for(int i=0;i<3;i++) mp[i]=kBDIGSOCoef[i][0];
            else if(el>=9) for(int i=0;i<3;i++) mp[i]=kBDIGSOCoef[i][9];
            else for(int i=0;i<3;i++) mp[i]=kBDIGSOCoef[i][int_el]*(1.0-el+int_el)+kBDIGSOCoef[i][int_el+1]*(el-int_el);
        }
        if(BD2_MEO){
            if(el<0) for(int i=0;i<3;i++) mp[i]=kBDMEOCoef[i][0];
            else if(el>=9) for(int i=0;i<3;i++) mp[i]=kBDMEOCoef[i][9];
            else for(int i=0;i<3;i++) mp[i]=kBDMEOCoef[i][int_el]*(1.0-el+int_el)+kBDMEOCoef[i][int_el+1]*(el-int_el);
        }

        for(int i=0;i<3;i++){
            if(sat_info->frq[i]==0.0) continue;
            if(sat_info->frq[i]==FREQ_BDS_B1) sat_info->bd2_mp[i]=mp[i];
            if(sat_info->frq[i]==FREQ_BDS_B2) sat_info->bd2_mp[i]=mp[i];
            if(sat_info->frq[i]==FREQ_BDS_B3) sat_info->bd2_mp[i]=mp[i];
        }

        for(int i=0;i<3;i++) bd2_mp_[i]=mp[i];
    }

}