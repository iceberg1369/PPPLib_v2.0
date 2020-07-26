//
// Created by cc on 7/9/20.
//

#include "CmnFunc.h"

namespace PPPLib{

    string Doul2Str(int str_len, int dec_len, const string str_filler, const double src_num, string &dst_str){
        dst_str=to_string(src_num); /* with 6 decimal digit */
        if(dec_len>0)dst_str=dst_str.substr(0,dst_str.length()-6+dec_len); /* decimal digits */
        else dst_str=dst_str.substr(0,dst_str.length()-7);
        while (str_len>dst_str.length())
            dst_str=str_filler+dst_str;
        return dst_str;
    }

    string Int2Str(int str_len, const string str_filler, const int src_num, string &dst_str){
        dst_str=to_string(src_num);
        while(str_len>dst_str.length())
            dst_str=str_filler+dst_str;
        return dst_str;
    }

    int Str2Double(string src_str,double &dst_num){
        int i,fnum;

        if (src_str.length()<=0) return 0;

        for (i=0,fnum=1; i<src_str.length(); i++){
            if (fnum && src_str[i]<='9' && src_str[i]>='0') fnum=0;
            if (fnum==0 && src_str[i]=='D') { src_str[i]='E'; break; }
        }
        if (fnum) return 0;

        dst_num=stod(src_str);

        return 1;
    }

    int Str2Int(const string src_str,int &dst_num){
        int i;
        for (i=0; i<src_str.length(); i++){
            if (src_str[i]<='9'&&src_str[i]>='0') break;
        }
        if (i>=src_str.length()) return 0;

        dst_num=stoi(src_str);

        return 1;
    }

    string StrTrim(string s){
        if(s.empty()) return s;
        s.erase(0,s.find_first_not_of(" "));
        s.erase(s.find_last_not_of(" ")+1);
        return s;
    }

    cTime::cTime(){
        for(double & i : epoch_) i=0.0;
        epoch_[0]=0;epoch_[1]=0;epoch_[2]=0;
        Epoch2Time(epoch_);
    }

    cTime::cTime(const double *ep){
        if(ep){
            for(int i=0;i<6;i++) epoch_[i]=ep[i];
            Epoch2Time(ep);
        }
        else{
            for(int i=0;i<6;i++) epoch_[i]=0.0;
            Epoch2Time(epoch_);
        }

    }

    cTime::cTime(string s):time_str_(s){
        Str2Time(s);
    }

    cTime cTime::operator=(const tTime t) {
        t_=t;
    }

    cTime cTime::operator+(double sec) const{
        double tt;
        cTime t;
        t=*this;
        t.t_.sec+=sec;
        tt=floor(t.t_.sec);
        t.t_.long_time+=(int)tt;
        t.t_.sec-=tt;
        return t;
    }

    void cTime::operator+=(double sec) {
        double tt;
        t_.sec+=sec;
        tt=floor(t_.sec);
        t_.long_time+=(int)tt;
        t_.sec-=tt;
    }

    cTime::~cTime() {}

    double* cTime::GetEpoch() {
        return epoch_;
    }

    string cTime::GetTimeStr(int n) {
        Time2Str(n);
        return time_str_;
    }

    int cTime::GetDoy() {
        Time2Doy();
        return doy_;
    }

    tMjd* cTime::GetMjd() {
        Time2Mjd();
        return &mjd_;
    }

    string cTime::Time2Str(int n){
        if (n<0) n=0; else if (n>12) n=12;
        string str;
        if (1.0-t_.sec<0.5/pow(10.0,n)) { t_.long_time++; t_.sec=0.0; };
        Time2Epoch();
        time_str_=Int2Str(4,"0",(int)epoch_[0],str)+"/"+Int2Str(2,"0",(int)epoch_[1],str)+"/"+
                  Int2Str(2,"0",(int)epoch_[2],str)+" "+Int2Str(2,"0",(int)epoch_[3],str)+":"+
                  Int2Str(2,"0",(int)epoch_[4],str)+":"+Doul2Str(2+n+1,n,"0",epoch_[5],str);
        return time_str_;
    }

    cTime* cTime::Epoch2Time(const double *ep){
        const int doy[]={ 1,32,60,91,121,152,182,213,244,274,305,335 };
        int days, dsec, year=int(ep[0]), mon=(int)ep[1], day=(int)ep[2];

        if (year<1970||2099<year||mon<1||12<mon) {
            t_.long_time=0;t_.sec=0; return this;
        }

        /* leap year if year%4==0 in 1901-2099 */
        days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3 ? 1 : 0);
        dsec=(int)floor(ep[5]);
        t_.long_time=(time_t)days*86400+(time_t)ep[3]*3600+(time_t)ep[4]*60+dsec;
        t_.sec=ep[5]-dsec;

        return this;
    }

    void cTime::Time2Epoch() {
        const int mday[]={ /* # of days in a month */
                31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
                31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
        };
        int days, dsec, mon, day;

        /* leap year if year%4==0 in 1901-2099 */
        days=(int)(t_.long_time/86400);
        dsec=(int)(t_.long_time-(time_t)days*86400);
        for (day=days%1461, mon=0; mon<48; mon++) {
            if (day>=mday[mon]) day-=mday[mon]; else break;
        }
        epoch_[0]=1970+days/1461*4+mon/12; epoch_[1]=mon%12+1; epoch_[2]=day+1;
        epoch_[3]=dsec/3600; epoch_[4]=dsec%3600/60; epoch_[5]=dsec%60+t_.sec;
    }

    int cTime::Str2Time(string s) {
        if (sscanf(s.c_str(),"%lf %lf %lf %lf %lf %lf",epoch_,epoch_+1,epoch_+2,epoch_+3,epoch_+4,epoch_+5)<6){
            LOG(ERROR)<<"time string error";
            return -1;
        }

        if (epoch_[0]<100) epoch_[0]+=2000;
        if (epoch_[0]<=1900||epoch_[1]==0||epoch_[2]==0){
            LOG(ERROR)<<"time format error";
            return -1;
        }

        Epoch2Time(epoch_);

        time_str_=s;

        return 0;
    }

    double cTime::TimeDiff(PPPLib::tTime t1) {
        return difftime(t_.long_time,t1.long_time)+t_.sec-t1.sec;
    }

    void cTime::Time2Mjd() {
        this->Time2Epoch();
        int year=(int)round(epoch_[0]);
        int mon=(int)round(epoch_[1]);
        if(mon<=2){
            year=year-1;
            mon=mon+12;
        }

        int a=(int)(365.25*year);
        int b=(int)(30.6001)*(mon+1);
        mjd_.day=a+b+round(epoch_[2])-679019;
        mjd_.sod.sn=round(epoch_[3])*3600+round(epoch_[4])*60+round(epoch_[5]);
        mjd_.sod.tos=epoch_[5]-round(epoch_[5]);
    }

    double cTime::Time2Doy() {
        double ep0[6]={0};

        Time2Epoch();
        ep0[0]=epoch_[0];ep0[1]=ep0[2]=1.0;
        cTime t0(ep0);
        doy_=(int) (TimeDiff(t0.t_))/86400.0+1.0;
        return doy_;
    }

    double cTime::Time2Sec(cTime& day){
        double sec;
        double ep0[6]={0};
        int i;
        Time2Epoch();

        sec=epoch_[3]*3600.0+epoch_[4]*60.0+epoch_[5];
        for(i=0;i<3;i++) ep0[i]=epoch_[i];
        day.Epoch2Time(ep0);
        return sec;
    }


    double cTime::Time2Gpst(int* week, int* day, int sys) {
        const double *ep=kGpsTimeStart;
        switch(sys){
            case SYS_BDS: ep=kBdsTimeStart;break;
            case SYS_GAL: ep=kGalTimeStart;break;
        }
        cTime t0(ep);
        time_t sec;

        sec=t_.long_time-t0.t_.long_time;

        int w=(int)(sec/(86400*7));
        if(week) *week=w;

        double s=(double)(sec-(double)w*86400*7)+t_.sec;
        int d=(int)s/86400;
        if(day) *day=d;

        return s;
    }

    cTime* cTime::Gpst2Time(int week, double wos, int sys) {
        const double *ep=kGpsTimeStart;
        switch(sys){
            case SYS_BDS: ep=kBdsTimeStart;break;
            case SYS_GAL: ep=kGalTimeStart;break;
        }
        Epoch2Time(ep);

        if(wos<-1E9||1E9<wos) wos=0.0;
        t_.long_time+=(time_t)86400*7*week+(int)wos;
        t_.sec=wos-(int)wos;
        return this;
    }

    cTime cTime::Utc2Gpst() {
        int i;
        cTime t,t0,tg;
        t=*this;

        for(i=0;kUtcLeapSec[i][0]>0;i++){
            if((t.TimeDiff(t0.Epoch2Time(kUtcLeapSec[i])->t_))>=0.0){
                tg=t+(-kUtcLeapSec[i][6]);
                return tg;
            }
        }
    }

    cTime cTime::Gpst2Utc() {
        int i;
        cTime tu,t0;

        for(i=0; kUtcLeapSec[i][0]>0;i++){
            tu=*this;
            tu+=(kUtcLeapSec[i][6]);
            t0.Epoch2Time(kUtcLeapSec[i]);
            if((TimeDiff(t0.t_))>=0.0){
                *this=tu;
                return *this;
            }
        }
    }

    cTime cTime::Gpst2Bdst() const{
        return *this+(-14.0);
    }

    cTime cTime::Bdst2Gpst() const {
        return *this+14.0;
    }

    double cTime::Utc2Gmst(double ut1_utc) {
        const double epoch_2000[]={2000,1,1,12,0,0};
        cTime tut,tut0,t2000;
        double ut,t1,t2,t3,gmst0,gmst;

        tut=*this;tut+=ut1_utc;
        ut=tut.Time2Sec(tut0);
        t1=tut0.TimeDiff(t2000.Epoch2Time(epoch_2000)->t_)/86400.0/36525.0;
        t2=t1*t1; t3=t2*t1;
        gmst0=24110.54841+8640184.812866*t1+0.093104*t2-6.2E-6*t3;
        gmst=gmst0+1.002737909350795*ut;

        return fmod(gmst,86400.0)*PI/43200.0; /* 0 <= gmst <= 2*PI */
    }

    cTime* cTime::AdjWeek(cTime t) {
        double dt=TimeDiff(t.t_);
        if(dt<-302400.0) *this+=604800.0;
        if(dt> 302400.0) *this+=(-604800.0);
        return this;
    }

    cTime* cTime::AdjDay(cTime t) {
        double dt=TimeDiff(t.t_);
//        if(dt<-43200.0) return *this+  86400.0;
//        if(dt> 43200.0) return *this+(-86400.0);
    }

    cCoord::cCoord() {
        coord_ENU_={0.0,0.0,0.0};
        coord_BLH_={0.0,0.0,0.0};
        coord_XYZ_={0.0,0.0,0.0};
    }

    cCoord::cCoord(const Vector3d& coord, const PPPLib::COORDINATE_TYPE coord_type) {
        if(coord_type==COORD_ENU){
            coord_ENU_=coord;
            Enu2Ned();
        }
        else if(coord_type==COORD_NED){
            coord_NED_=coord;
            Ned2Enu();
        }
        else if(coord_type==COORD_BLH){
            coord_BLH_=coord;
            Blh2Xyz();
        }
        else if(coord_type==COORD_XYZ){
            coord_XYZ_=coord;
            Xyz2Blh();
        }
        else{
            coord_ENU_={0.0,0.0,0.0};
            coord_BLH_={0.0,0.0,0.0};
            coord_XYZ_={0.0,0.0,0.0};
        }
    }

    cCoord::cCoord(const double* coord, const PPPLib::COORDINATE_TYPE coord_type) {
        if(coord_type==COORD_ENU){
            coord_ENU_[0]=coord[0];
            coord_ENU_[1]=coord[1];
            coord_ENU_[2]=coord[2];
            Enu2Ned();
        }
        else if(coord_type==COORD_BLH){
            coord_BLH_[0]=coord[0];
            coord_BLH_[1]=coord[1];
            coord_BLH_[2]=coord[2];
            Blh2Xyz();
        }
        else if(coord_type==COORD_XYZ){
            coord_XYZ_[0]=coord[0];
            coord_XYZ_[1]=coord[1];
            coord_XYZ_[2]=coord[2];
            Xyz2Blh();
        }
        else{
            coord_ENU_={0.0,0.0,0.0};
            coord_BLH_={0.0,0.0,0.0};
            coord_XYZ_={0.0,0.0,0.0};
        }
    }

    cCoord::~cCoord() {

    }

    Vector3d cCoord::GetCoordEnu(cCoord ref_coord){
        Xyz2Enu(ref_coord);
        return coord_ENU_;
    }

    Vector3d cCoord::GetCoordBlh(){
        Xyz2Blh();
        return coord_BLH_;
    }

    Vector3d cCoord::GetCoordXyz(){
        return coord_XYZ_;
    }

    Vector3d cCoord::GetCoordNed(cCoord ref_coord){
        Xyz2Enu(ref_coord);
        Enu2Ned();
        return coord_NED_;
    }

    Matrix3d cCoord::GetCne(const COORDINATE_TYPE type){
        CalcCne(type);
        return Cne_;
    }

    void cCoord::Xyz2Blh() {
        double lon,lat,hgt;
        if(coord_XYZ_[0]>LAT_ACCURACY){
            lon=atan(coord_XYZ_[1]/coord_XYZ_[0]);
        }
        else if(coord_XYZ_[0]<LAT_ACCURACY){
            lon=atan(coord_XYZ_[1]/coord_XYZ_[0])+PI;
        }
        else{
            if(coord_XYZ_[1]>0)
                lon=PI*0.5;
            else
                lon=PI*1.5;
        }
        lat=CalcLat();
        double N=WGS84_EARTH_LONG_RADIUS/sqrt(1.0-WGS84_FIRST_E2*sin(lat)*sin(lat));
        hgt=sqrt(coord_XYZ_[0]*coord_XYZ_[0]+coord_XYZ_[1]*coord_XYZ_[1])*cos(lat)+coord_XYZ_[2]*sin(lat)
                -N*(1.0-WGS84_FIRST_E2*pow(sin(lat),2));

        coord_BLH_[0]=lat;
        coord_BLH_[1]=lon;
        coord_BLH_[2]=hgt;
    }

    void cCoord::Blh2Xyz() {
        const double sin_lat=sin(coord_BLH_[0]);
        const double cos_lat=cos(coord_BLH_[0]);
        const double sin_lon=sin(coord_BLH_[1]);
        const double cos_lon=cos(coord_BLH_[1]);
        double N=WGS84_EARTH_LONG_RADIUS/sqrt(1.0-WGS84_FIRST_E2*sin_lat*sin_lat);

        coord_XYZ_[0]=(N+coord_BLH_[2])*cos_lat*cos_lon;
        coord_XYZ_[1]=(N+coord_BLH_[2])*cos_lat*sin_lon;
        coord_XYZ_[2]=(N*(1.0-WGS84_FIRST_E2)+coord_BLH_[2])*sin_lat;
    }

    void cCoord::Xyz2Enu(cCoord ref_coord) {
        Vector3d temp_xyz;
        temp_xyz[0]=coord_XYZ_[0]-ref_coord.coord_XYZ_[0];
        temp_xyz[1]=coord_XYZ_[1]-ref_coord.coord_XYZ_[1];
        temp_xyz[2]=coord_XYZ_[2]-ref_coord.coord_XYZ_[2];

        double lat=ref_coord.coord_BLH_[0];
        double lon=ref_coord.coord_BLH_[1];

        coord_ENU_[0]=-sin(lon)*temp_xyz[0]+cos(lon)*temp_xyz[1];
        coord_ENU_[1]=-sin(lat)*cos(lon)*temp_xyz[0]-sin(lat)*sin(lon)*temp_xyz[1]+cos(lat)*temp_xyz[2];
        coord_ENU_[2]=cos(lat)*cos(lon)*temp_xyz[0]+cos(lat)*sin(lon)*temp_xyz[1]+sin(lat)*temp_xyz[2];
    }

    void cCoord::Enu2Xyz(cCoord ref_coord) {
        double lat=ref_coord.coord_BLH_[0],lon=ref_coord.coord_BLH_[1];
        double sin_lat=sin(lat);
        double cos_lat=cos(lat);
        double sin_lon=sin(lon);
        double cos_lon=cos(lon);

        double temp1=sin_lat*cos_lon*coord_ENU_[1];
        double temp2=sin_lon*coord_ENU_[0];
        double temp3=cos_lat*cos_lon*coord_ENU_[2];
        coord_XYZ_[0]=ref_coord.coord_XYZ_[0]-temp1-temp2+temp3;

        temp1=sin_lat*sin_lon*coord_ENU_[1];
        temp2=cos_lon*coord_ENU_[0];
        temp3=cos_lat*sin_lon*coord_ENU_[2];
        coord_XYZ_[1]=ref_coord.coord_XYZ_[1]-temp1+temp2+temp3;

        temp1=cos_lat*coord_ENU_[1];
        temp2=sin_lat*coord_ENU_[2];
        coord_XYZ_[2]=ref_coord.coord_XYZ_[2]+temp1+temp2;
    }

    void cCoord::Enu2Ned() {
        coord_NED_[0]= coord_ENU_[1];
        coord_NED_[1]= coord_ENU_[0];
        coord_NED_[2]=-coord_ENU_[2];
    }

    void cCoord::Ned2Enu() {
        coord_ENU_[0]= coord_NED_[1];
        coord_ENU_[1]= coord_NED_[0];
        coord_ENU_[2]=-coord_NED_[2];
    }

    void cCoord::CalcCne(const COORDINATE_TYPE type) {
        double lat=coord_BLH_[0],lon=coord_BLH_[1];
        double sin_lat=sin(lat),cos_lat=cos(lat);
        double sin_lon=sin(lon),cos_lon=cos(lon);
        if(type==COORD_NED){
            Cne_(0,0)=-sin_lat*cos_lon;
            Cne_(0,1)=-sin_lon;
            Cne_(0,2)=-cos_lat*cos_lon;

            Cne_(1,0)=-sin_lat*sin_lon;
            Cne_(1,1)=cos_lon;
            Cne_(1,2)=-cos_lat*sin_lon;

            Cne_(2,0)=cos_lat;
            Cne_(2,1)=0.0;
            Cne_(2,2)=-sin_lat;
        }
        else {

        }
    }

    double cCoord::CalcLat() {
        double temp_lat1=0.0,temp_lat2=0.0;
        double N=0.0;
        temp_lat2=atan(coord_XYZ_[2]/sqrt(coord_XYZ_[0]*coord_XYZ_[0]+coord_XYZ_[1]*coord_XYZ_[1]));
        while(true){
            temp_lat1=temp_lat2;
            N=WGS84_EARTH_LONG_RADIUS/sqrt(1.0-WGS84_FIRST_E2*sin(temp_lat1)*sin(temp_lat1));
            temp_lat2=atan((coord_XYZ_[2]+N*WGS84_FIRST_E2*sin(temp_lat1))/sqrt(coord_XYZ_[0]*coord_XYZ_[0]+coord_XYZ_[1]*coord_XYZ_[1]));
            if(fabs(temp_lat2-temp_lat1)<LAT_ACCURACY){
                return temp_lat2;
            }
        }
    }

    cParSetting::cParSetting(){}

    cParSetting::cParSetting(tPPPLibConf conf) {PPPLibC_=conf;}

    cParSetting::~cParSetting() {}

    int cParSetting::GetGnssUsedFrqs() {
        if(PPPLibC_.gnssC.ion_opt==ION_IF) return 1;
        else return PPPLibC_.gnssC.frq_opt+1;
    }

    int cParSetting::GetSppParNum() {
        return PvaParNum()+RecClkParNum();
    }

    int cParSetting::GetPppParNum() {
        return GetSppParNum()+RecDcbParNum()+RecIfbParNum()+GloIfcbParNum()+TrpParNum()+IonParNum()+AmbParNum();
    }

    int cParSetting::GetDgnssParNum() {
        return PvaParNum();
    }

    int cParSetting::GetPpkParNum() {
        return PvaParNum()+TrpParNum()+IonParNum()+AmbParNum();
    }

    int cParSetting::PvaParNum() {
        if(PPPLibC_.dynamic) return 3+3+3;
        return 3;
    }

    int cParSetting::RecClkParNum() {
        return NSYS;
    }

    int cParSetting::RecDcbParNum() {
        if(PPPLibC_.gnssC.ion_opt==ION_CONST) return NSYS;
        return 0;
    }

    int cParSetting::RecIfbParNum() {
        if(PPPLibC_.gnssC.frq_opt>=3) return NSYS;
        return 0;
    }

    int cParSetting::GloIfcbParNum() {

    }

    int cParSetting::TrpParNum() {
        if(PPPLibC_.gnssC.trp_opt<=TRP_SAAS) return 0;
        else if(PPPLibC_.gnssC.trp_opt==TRP_EST_WET) return 1;
        else if(PPPLibC_.gnssC.trp_opt==TRP_EST_GRAD) return 1+2;
    }

    int cParSetting::IonParNum() {
        if(PPPLibC_.gnssC.ion_opt<=ION_IF) return 0;
        else if(PPPLibC_.gnssC.ion_opt<=ION_CONST) return MAX_SAT_NUM;
    }

    int cParSetting::AmbParNum() {
        return MAX_SAT_NUM*GetGnssUsedFrqs();
    }

    int cParSetting::ParIndexPva(int i) {
        if(i<3) return 0;
        else if(i<6) return 3;
        else if(i<9) return 6;
    }

    int cParSetting::ParIndexClk(int sys_index) {
        return PvaParNum()+sys_index;
    }

    int cParSetting::ParIndexDcb(int sys_index) {
        return PvaParNum()+RecClkParNum()+sys_index;
    }

    int cParSetting::ParIndexIfb(int sys_index) {
        return PvaParNum()+RecClkParNum()+RecDcbParNum()+sys_index;
    }

    int cParSetting::ParIndexGloIfcb() {

    }

    int cParSetting::ParIndexTrp() {
        return PvaParNum()+RecClkParNum()+RecDcbParNum()+RecIfbParNum()+GloIfcbParNum();
    }

    int cParSetting::ParIndexIon(int sat_no) {
        return PvaParNum()+RecClkParNum()+RecDcbParNum()+RecIfbParNum()+GloIfcbParNum()+TrpParNum()+sat_no-1;
    }

    int cParSetting::ParIndexAmb(int f, int sat_no) {
        return PvaParNum()+RecClkParNum()+RecDcbParNum()+RecIfbParNum()+GloIfcbParNum()+TrpParNum()+IonParNum()
               +f*MAX_SAT_NUM+sat_no-1;
    }
}






