//
// Created by chenc on 2020/7/13.
//

#include "ReadFiles.h"

namespace PPPLib{

    ReadFile::ReadFile() { }

    ReadFile::ReadFile(string file_path):file_(file_path) {}

    ReadFile::~ReadFile() {}

    bool ReadFile::OpenFile() {
        inf_.open(file_);
        if(!inf_.is_open()){
            LOG(WARNING)<<"File open error: "<<file_;
            return false;
        }
        return true;
    }

    void ReadFile::CloseFile() {
        if(inf_.is_open()){
            inf_.close();
        }
    }

    bool ReadFile::ReadHead() {}

    bool ReadFile::Reading() {return true;}

    ReadImu::ReadImu() {}

    ReadImu::ReadImu(string file_path){file_=file_path;}

    ReadImu::~ReadImu() {}

    bool ReadImu::DecodeImu() {
        if(imu_data_.imu_type_==IMU_NOVTEL_CPT){
            double a_scale=1.52587890625E-06;
            double g_scale=1.085069444444444E-07;
            DecodeNovatel(a_scale, g_scale);
        }else if(imu_data_.imu_type_==IMU_NOVTEL_A1){
            double a_scale=1.52587890625E-06;
            double g_scale=1.085069444444444E-07;
            DecodeNovatel(a_scale, g_scale);
        }
        else{
            LOG(ERROR)<<"Unsupport imu type";
            return false;
        }
        return true;
    }

    bool ReadImu::DecodeNovatel(double a_scale, double g_scale) {
        char *seps=",;*";
        char *token;
        int i,gps_week;
        tImuDataUnit imu_data={0};

        while(getline(inf_,line_str_)&&!inf_.eof()){
            token=strtok((char *)line_str_.c_str(),seps);
            if(!strcmp(token,"%RAWIMUSA")){
                for(i=0;i<3;i++) token=strtok(NULL,seps);
                gps_week=atoi(token);
                token=strtok(NULL,seps);
                imu_data.t_tag_.Gpst2Time(gps_week, atof(token),SYS_GPS);

                if(imu_data_.ts_.GetTime()->long_time!=0.0){
                    if(imu_data.t_tag_.TimeDiff(*imu_data_.ts_.GetTime())<0) continue;
                }
                if(imu_data_.te_.GetTime()->long_time!=0.0){
                    if(imu_data.t_tag_.TimeDiff(*imu_data_.te_.GetTime())>0) continue;
                }

                for(i=0;i<2;i++) token=strtok(NULL,seps);
                imu_data.acce_[2]=atof(token)*a_scale;
                token=strtok(NULL,seps);
                imu_data.acce_[1]=-atof(token)*a_scale;
                token=strtok(NULL,seps);
                imu_data.acce_[0]=atof(token)*a_scale;

                token=strtok(NULL,seps);
                imu_data.gyro_[2]=atof(token)*g_scale;
                token=strtok(NULL,seps);
                imu_data.gyro_[1]=atof(token)*g_scale;
                token=strtok(NULL,seps);
                imu_data.gyro_[0]=atof(token)*g_scale;

                imu_data.t_tag_.GetTimeStr(3);
//                LOG(INFO)<<imu_data.t_tag_.GetTimeStr(3);
                imu_data_.data_.push_back(imu_data);
            }
        }
        if(imu_data_.data_.empty()) return false;
        return true;
    }

    cImuData* ReadImu::GetImus() {return &imu_data_;}

    void ReadImu::SetImuTimeSpan(PPPLib::cTime *ts, PPPLib::cTime *te) {
        if(ts->TimeDiff(*te->GetTime())>=0){
            LOG(WARNING)<<"Wrong time interval setting";
            return;
        }
        else{
            imu_data_.SetTimeSpan(ts,te);
        }
    }

    bool ReadImu::SetImuType(PPPLib::IMU_TYPE type) {
        if(type==IMU_UNKNOW){
            LOG(ERROR)<<"Unknow imu type";
            return false;
        }
        imu_data_.SetImuType(type);
    }

    void ReadImu::SetImuCoordType(PPPLib::IMU_COORD_TYPE type) {
        imu_data_.SetImuCoordType(type);
    }

    bool ReadImu::Reading() {
        if(!OpenFile()){
            LOG(ERROR)<<"Open imu file error: "<<file_;
            return false;
        }

        DecodeImu();

        if(OpenFile()) CloseFile();
        return true;
    }

    void ReadImu::OutImu(string out_path) {
        ofstream fout(out_path);
        string sep="   ";
        tImuDataUnit imu_data={0};

        for(int i=0;i<imu_data_.data_.size();i++){
            imu_data=imu_data_.data_[i];
            fout<<imu_data.t_tag_.GetTimeStr(3)<<sep<<imu_data.acce_[0]<<sep<<imu_data.acce_[1]<<sep<<imu_data.acce_[2];
            fout<<sep<<imu_data.gyro_[0]<<sep<<imu_data.gyro_[1]<<sep<<imu_data.gyro_[2]<<endl;
        }
    }

    ReadPos::ReadPos() {}

    ReadPos::ReadPos(string file_path) {file_=file_path;}

    ReadPos::~ReadPos() {}

    bool ReadPos::DecodePos() {
        string line_str;

        while(getline(inf_,line_str)&&!inf_.eof()){
            if(line_str.size()==0) continue;
        }
    }

    bool ReadPos::Reading() {
        if(!OpenFile()){
            LOG(ERROR)<<"Open imu file error: "<<file_;
            return false;
        }

        DecodePos();

        if(OpenFile()) CloseFile();
        return true;
    }

    ReadRnx::ReadRnx(){}

    ReadRnx::ReadRnx(string file_path) {file_=file_path;}

    ReadRnx::~ReadRnx() {}

    void ReadRnx::SetGnssSysMask(int mask) {sys_mask_=mask;}

    bool ReadRnx::ReadRnxHead() {

        while(getline(inf_,line_str_)&&!inf_.eof()){

            if(line_str_.find("RINEX VERSION / TYPE")!=string::npos){
                Str2Double(line_str_.substr(0,9),rnx_ver_);
                rnx_type_=line_str_.substr(20,1);
                switch(line_str_[40]){
                    case ' ':
                    case 'G': sat_sys_=SYS_GPS; time_sys_=TIME_GPS;break;
                    case 'C': sat_sys_=SYS_BDS; time_sys_=TIME_BDS;break;
                    case 'E': sat_sys_=SYS_GAL; time_sys_=TIME_GAL;break;
                    case 'R': sat_sys_=SYS_GLO; time_sys_=TIME_utc;break;
                    case 'J': sat_sys_=SYS_QZS; time_sys_=TIME_QZS;break;
                    case 'M': sat_sys_=SYS_NONE;time_sys_=TIME_GPS;break;
                    default: break;
                }
                break;
            }
        }

        if(rnx_type_.empty()){
            LOG(ERROR)<<"Rinex file type error";
            return false;
        }
        return true;
    }

    cGnssSignal::cGnssSignal() {}

    cGnssSignal::~cGnssSignal() {}

    tGnssSignal * cGnssSignal::GetGnssSignal() {return &signal_;}

    unsigned char cGnssSignal::Signal2Code(string signal, int *frq, int sat_sys) {
        int i;
        if(frq) *frq=0;
        for(i=0;i<MAX_GNSS_CODE_TYPE;i++){
            if(kGnssSignalCodes[i]!=signal) continue;
            if(frq){
                switch(sat_sys){
                    case SYS_GPS: *frq=kGpsFreqBand[i];break;
                    case SYS_BDS: *frq=kBdsFreqBand[i];break;
                    case SYS_GAL: *frq=kGalFreqBand[i];break;
                    case SYS_GLO: *frq=kGloFreqBand[i];break;
                    case SYS_QZS: *frq=kQzsFreqBand[i];break;
                    case SYS_IRN: *frq=0;break;
                    default:
                        *frq=0;break;
                }
            }
            return (unsigned char)i;
        }
        return GNSS_CODE_NONE;
    }

    string cGnssSignal::Code2Signal(unsigned char code, int *frq, int sat_sys) {
        if(frq) *frq=0;
        if(code<=GNSS_CODE_NONE||MAX_GNSS_CODE_TYPE<code) return "";
        if(frq){
            switch(sat_sys){
                case SYS_GPS: *frq=kGpsFreqBand[code];break;
                case SYS_BDS: *frq=kBdsFreqBand[code];break;
                case SYS_GAL: *frq=kGalFreqBand[code];break;
                case SYS_GLO: *frq=kGloFreqBand[code];break;
                case SYS_QZS: *frq=kQzsFreqBand[code];break;
                case SYS_IRN: *frq=0;break;
                default:
                    *frq=0;break;
            }
        }
        return kGnssSignalCodes[code];
    }

    int cGnssSignal::GetCodePri(int sat_sys, unsigned char code) {
        if(code==GNSS_CODE_NONE) return 0;
        size_t str_pos;
        string signal;
        int i,f;

        signal=Code2Signal(code,&f,sat_sys);
        switch(sat_sys){
            case SYS_GPS: i=SYS_INDEX_GPS;break;
            case SYS_BDS: i=SYS_INDEX_BDS;break;
            case SYS_GAL: i=SYS_INDEX_GAL;break;
            case SYS_GLO: i=SYS_INDEX_GLO;break;
            case SYS_QZS: i=SYS_INDEX_QZS;break;
            case SYS_IRN: i=0;break;
            default:
                i=0;break;
        }
        return ((str_pos=kGnssSignalPriors[i][f-1].find(signal[1]))!=string::npos)?14-(int)str_pos:0;
    }

    void cGnssSignal::GnssSignalIndex(int sat_sys, string *obs_code_type) {
        size_t p;
        string s;
        int i,j,num_signal;
        int k_code=-1,k_phase=-1,k_doppler=-1,k_snr=-1;

        for(i=num_signal=0;obs_code_type[i][0];i++,num_signal++){
            signal_.code[i]=Signal2Code(obs_code_type[i].substr(1),signal_.frq+i,sat_sys);
            signal_.type[i]=((p=kGnssObsCode.find(obs_code_type[i][0]))!=string::npos)?(int)p:0;
            signal_.pri[i]=GetCodePri(sat_sys,signal_.code[i]);
        }

        for(i=0;i<MAX_GNSS_FRQ_NUM;i++){
            for(j=0;j<num_signal;j++){
                if(signal_.type[j]==GNSS_OBS_CODE){
                    if(signal_.frq[j]==i+1&&signal_.pri[j]&&
                            (k_code<0||signal_.pri[j]>=signal_.pri[k_code])){
                        k_code=j;
                    }
                }
                else if(signal_.type[j]==GNSS_OBS_PHASE){
                    if(signal_.frq[j]==i+1&&signal_.pri[j]&&
                            (k_phase<0||signal_.pri[j]>=signal_.pri[k_phase])){
                        k_phase=j;
                    }
                }
                else if(signal_.type[j]==GNSS_OBS_DOPPLER){
                    if(signal_.frq[j]==i+1&&signal_.pri[j]&&
                            (k_doppler<0||signal_.pri[j]>=signal_.pri[k_doppler])){
                        k_doppler=j;
                    }
                }
                else if(signal_.type[j]==GNSS_OBS_SNR){
                    if(signal_.frq[j]==i+1&&signal_.pri[j]&&
                            (k_snr<0||signal_.pri[j]>=signal_.pri[k_snr])){
                        k_snr=j;
                    }
                }
            }

            if(k_code>=0)    signal_.pos[k_code]=i;
            if(k_phase>=0)   signal_.pos[k_phase]=i;
            if(k_doppler>=0) signal_.pos[k_doppler]=i;
            if(k_snr>=0)     signal_.pos[k_snr]=i;

            k_code=k_phase=k_doppler=k_snr=-1;
        }
        signal_.n=num_signal;
    }

    ReadGnssObs::ReadGnssObs() {sys_mask_=SYS_ALL;}

    ReadGnssObs::ReadGnssObs(string file_path,PPPLib::tNav &nav,RECEIVER_INDEX rcv) {
        file_=file_path;
        gnss_data_.SetRcvIdx(rcv);
        nav_=nav;
        sys_mask_=SYS_ALL;
    }

    ReadGnssObs::~ReadGnssObs() {}

    bool ReadGnssObs::CmpEpochSatData(const PPPLib::tSatObsUnit &p1, const PPPLib::tSatObsUnit &p2) {
        return p1.sat.sat_.no<p2.sat.sat_.no;
    }

    bool ReadGnssObs::SortEpochSatData(PPPLib::tEpochSatUnit& epoch_sat_data) {
        if(epoch_sat_data.sat_num<=0) return false;
        sort(epoch_sat_data.epoch_data.begin(),epoch_sat_data.epoch_data.end(),CmpEpochSatData);
    }

    int ReadGnssObs::DecodeEpoch(cTime& t, int& obs_flag) {
        int n=0;
        Str2Int(line_str_.substr(32,3),n);
        if(n<=0) return 0;

        Str2Int(line_str_.substr(31,1),obs_flag);
        if(3<obs_flag&&obs_flag<5) return n;

        if(line_str_[0]!='>'||t.Str2Time(line_str_.substr(1,28))!=0)
            return 0;

        return n;
    }

    bool ReadGnssObs::DecodeEpochSatObs(tSatObsUnit& obs) {
        cGnssSignal *gnss_signal={0};
        double val[MAX_GNSS_OBS_TYPE]={0};
        unsigned char lli[MAX_GNSS_OBS_TYPE]={0};
        string sat_id;
        int i,j,num;
        bool stat=true;

        if(rnx_ver_>2.99){
            sat_id=line_str_.substr(0,3);
            if(!sat_id.compare(1,1," ")) sat_id[1]='0';
            obs.sat=cSat(sat_id);
            obs.sat.SatId2No();
            if(!(obs.sat.sat_.sys&sys_mask_)) return false;
        }
        if(!obs.sat.sat_.no) stat=false;

        switch(obs.sat.sat_.sys){
            case SYS_BDS: gnss_signal=signal_index+SYS_INDEX_BDS;break;
            case SYS_GAL: gnss_signal=signal_index+SYS_INDEX_GAL;break;
            case SYS_GLO: gnss_signal=signal_index+SYS_INDEX_GLO;break;
            case SYS_QZS: gnss_signal=signal_index+SYS_INDEX_QZS;break;
            case SYS_IRN: gnss_signal=signal_index+SYS_INDEX_IRN;break;
            default:
                gnss_signal=signal_index;break;
        }

        if(!stat) return false;

        for(i=0,j=rnx_ver_<=2.99?0:3;i<gnss_signal->GetGnssSignal()->n&&j+15<line_str_.length();i++,j+=16){
            Str2Double(line_str_.substr(j,14),val[i]);
            val[i]+=gnss_signal->GetGnssSignal()->shift[i];

            Str2Int(line_str_.substr(j+14,1),num);
            lli[i]=(unsigned char)num&3;
        }

        int n,m,p[MAX_GNSS_OBS_TYPE],k[16],l[16];
        for(i=n=m=0;i<gnss_signal->GetGnssSignal()->n;i++){
            p[i]=rnx_ver_<=2.11?gnss_signal->GetGnssSignal()->frq[i]:gnss_signal->GetGnssSignal()->pos[i];
            if(gnss_signal->GetGnssSignal()->type[i]==0&&p[i]==0) k[n++]=i;
            if(gnss_signal->GetGnssSignal()->type[i]==0&&p[i]==1) l[m++]=i;
        }

        for(i=0;i<gnss_signal->GetGnssSignal()->n;i++){
            if(p[i]<0||val[i]==0.0) continue;
            switch(gnss_signal->GetGnssSignal()->type[i]){
                case GNSS_OBS_CODE:    obs.P[p[i]]=val[i];obs.code[p[i]]=gnss_signal->GetGnssSignal()->code[i];break;
                case GNSS_OBS_PHASE:   obs.L[p[i]]=val[i];obs.LLI[p[i]]=lli[i];break;
                case GNSS_OBS_DOPPLER: obs.D[p[i]]=(float)val[i];break;
                case GNSS_OBS_SNR:     obs.SNR[p[i]]=(unsigned char)(val[i]*4.0+0.5); break;
            }
        }
        return true;
    }

    int ReadGnssObs::ReadObsBody() {
        int line_idx=0,num_sat=0,n,obs_flag=0;
        tEpochSatUnit epoch_sat_data={nullptr};

        while(getline(inf_,line_str_)&&!inf_.eof()){
            tSatObsUnit sat_data={0};
            if(line_idx==0){
                if((num_sat=DecodeEpoch(epoch_sat_data.obs_time,obs_flag))<=0) continue;
                if(GetGnssData()->GetStartTime()->TimeDiff(*epoch_sat_data.obs_time.GetTime())>0) continue;
                if(GetGnssData()->GetEndTime()->TimeDiff(*epoch_sat_data.obs_time.GetTime())<0) continue;
            }
            else if(obs_flag<=2||obs_flag==6){
                if(DecodeEpochSatObs(sat_data)){
                    epoch_sat_data.epoch_data.push_back(sat_data);
                }
            }
            if(++line_idx>num_sat){
                epoch_sat_data.sat_num=epoch_sat_data.epoch_data.size();
                SortEpochSatData(epoch_sat_data);
                gnss_data_.GetGnssObs().push_back(epoch_sat_data);
                return num_sat;
            }
        }
        return 0;
    }

    void ReadGnssObs::SetGnssTimeSpan(cTime* ts,cTime* te) {
        if(ts->TimeDiff(*te->GetTime())>=0){
            LOG(WARNING)<<"Wrong time interval setting";
            return;
        }
        else{
            gnss_data_.SetTimeSpan(ts,te);
        }
    }


    cGnssObs* ReadGnssObs::GetGnssData() {
        return &gnss_data_;
    }

    tNav * ReadGnssObs::GetGnssNav() {
        return &nav_;
    }

    bool ReadGnssObs::ReadHead() {
        int i,j,k,num_signal,idx_signal,num_line=0,prn,fcn;
        tSta *sta=gnss_data_.GetStation();

        if(!ReadRnxHead()) return false;

        while(getline(inf_,line_str_)&&!inf_.eof()){
            if (line_str_.find("MARKER NAME")!=string::npos && sta)
                sta->name=StrTrim(line_str_.substr(0,10));
            else if (line_str_.find("MARKER NUMBER")!=string::npos && sta)
                sta->marker=StrTrim(line_str_.substr(0,20));
            else if (line_str_.find("MARKER TYPE")!=string::npos) continue;
            else if (line_str_.find("OBSERVER / AGENCY")!=string::npos) continue;
            else if (line_str_.find("REC # / TYPE / VERS")!=string::npos && sta){
                sta->ant_seri=StrTrim(line_str_.substr(0,20));
                sta->rec_type=StrTrim(line_str_.substr(20,20));
                sta->firm_ver=StrTrim(line_str_.substr(40,20));
            }
            else if (line_str_.find("ANT # / TYPE")!=string::npos && sta){
                sta->ant_seri=StrTrim(line_str_.substr(0,20));
                sta->ant_desc=StrTrim(line_str_.substr(20,20));
            }
            else if (line_str_.find("APPROX POSITION XYZ")!=string::npos && sta)
                for (i=0;i<3;i++){
                    Str2Double(line_str_.substr(i*14,14),sta->apr_pos[i]);
                }
            else if (line_str_.find("ANTENNA: DELTA H/E/N")!=string::npos && sta){
                Str2Double(line_str_.substr(0,14),sta->del[2]);  /* h */
                Str2Double(line_str_.substr(14,14),sta->del[0]); /* e */
                Str2Double(line_str_.substr(28,14),sta->del[1]); /* n */
            }
            else if (line_str_.find("ANTENNA: DELTA X/Y/Z")!=string::npos) continue; /* opt ver.3 */
            else if (line_str_.find("ANTENNA: PHASECENTER")!=string::npos) continue; /* opt ver.3 */
            else if (line_str_.find("ANTENNA: B.SIGHT XYZ")!=string::npos) continue; /* opt ver.3 */
            else if (line_str_.find("ANTENNA: ZERODIR AZI")!=string::npos) continue; /* opt ver.3 */
            else if (line_str_.find("ANTENNA: ZERODIR XYZ")!=string::npos) continue; /* opt ver.3 */
            else if (line_str_.find("CENTER OF MASS: XYZ" )!=string::npos) continue; /* opt ver.3 */
            else if (line_str_.find("SYS / # / OBS TYPES" )!=string::npos) { /* ver.3 */
                if((i=kSatSysCode.find(line_str_[0]))==string::npos){
                    LOG(WARNING)<<"Invalid satellite system: "<<line_str_[0];
                    continue;
                }
                Str2Int(line_str_.substr(3,3),num_signal);
                for(j=idx_signal=0,k=7;j<num_signal;j++,k+=4){
                    if(k>58){
                        if(!getline(inf_,line_str_)) break;
                        k=7;
                    }
                    if(idx_signal<MAX_GNSS_OBS_TYPE-1) obs_type_code_[i][idx_signal++]=line_str_.substr(k,3);
                }
                if(i==1){
                    if(rnx_ver_<3.04){
                        for(j=0;j<num_signal;j++){
                            if(obs_type_code_[i][j][1]=='1'){
                                obs_type_code_[i][j][1]='2';
                                LOG(INFO)<<"BD2 change C1x to C2x";
                            }
                        }
                    }
                }
            }
            else if (line_str_.find("WAVELENGTH FACT L1/2")!=string::npos) continue; /* opt ver.2 */
            else if (line_str_.find("# / TYPES OF OBSERV")!=string::npos) { /* ver.2 */
            }
            else if (line_str_.find("SIGNAL STRENGTH UNIT")!=string::npos) continue; /* opt ver.3 */
            else if (line_str_.find("INTERVAL"			 )!=string::npos) continue; /* opt */
            else if (line_str_.find("TIME OF FIRST OBS"   )!=string::npos) {
                if      (!line_str_.compare(48,3,"GPS")) time_sys_=TIME_GPS;
                else if (!line_str_.compare(48,3,"BDT")) time_sys_=TIME_BDS; /* ver.3.02 */
                else if (!line_str_.compare(48,3,"GAL")) time_sys_=TIME_GAL;
                else if (!line_str_.compare(48,3,"GLO")) time_sys_=TIME_utc;
                else if (!line_str_.compare(48,3,"QZS")) time_sys_=TIME_QZS; /* ver.3.02 */
            }
            else if (line_str_.find("TIME OF LAST OBS"    )!=string::npos) continue; /* opt */
            else if (line_str_.find("RCV CLOCK OFFS APPL" )!=string::npos) continue; /* opt */
            else if (line_str_.find("SYS / DCBS APPLIED"  )!=string::npos) continue; /* opt ver.3 */
            else if (line_str_.find("SYS / PCVS APPLIED"  )!=string::npos) continue; /* opt ver.3 */
            else if (line_str_.find("SYS / SCALE FACTOR"  )!=string::npos) continue; /* opt ver.3 */
            else if (line_str_.find("SYS / PHASE SHIFTS"  )!=string::npos) continue; /* ver.3.01 */
            else if (line_str_.find("GLONASS SLOT / FRQ #")!=string::npos && &nav_) { /* ver.3.02 */
                for (i=0;i<8;i++) {
                    string aa=line_str_.substr(7*i+8,2);
                    if (line_str_.compare(7*i+4,1,"R")!=0||!line_str_.compare(7*i+8,2,"  ")) continue;
                    string a=line_str_.substr(7*i+5,2);
                    string b=line_str_.substr(7*i+8,2);
                    Str2Int(line_str_.substr(7*i+5,2),prn);
                    Str2Int(line_str_.substr(7*i+8,2),fcn);
                    if (1<=prn&&prn<=GLO_MAX_PRN) nav_.glo_frq_num[prn-1]=fcn+8;
                }
            }
            else if (line_str_.find("GLONASS COD/PHS/BIS" )!=string::npos && &nav_) {  /* ver.3.02 */
                for (i=0;i<4;i++) {
                    if      (line_str_.compare(13*i+1,3,"C1C"))
                        Str2Double(line_str_.substr(13*i+5,8), nav_.glo_cp_bias[0]);
                    else if (line_str_.compare(13*i+1,3,"C1P"))
                        Str2Double(line_str_.substr(13*i+5,8),nav_.glo_cp_bias[1]);
                    else if (line_str_.compare(13*i+1,3,"C2C"))
                        Str2Double(line_str_.substr(13*i+5,8),nav_.glo_cp_bias[2]);
                    else if (line_str_.compare(13*i+1,3,"C2P"))
                        Str2Double(line_str_.substr(13*i+5,8),nav_.glo_cp_bias[3]);
                }
            }
            else if (line_str_.find("LEAP SECONDS")!=string::npos && &nav_) {/* opt */
                Str2Int(line_str_.substr(0,6),nav_.leaps);
            }
            else if (line_str_.find("# OF SALTELLITES")!=string::npos) continue;/* opt */
            else if (line_str_.find("PRN / # OF OBS"  )!=string::npos) continue;/* opt */
            else if (line_str_.find("PGM / RUN BY / DATE")!=string::npos) continue;
            else if (line_str_.find("COMMENT" )!=string::npos) continue;
            if (line_str_.find("END OF HEADER")!=string::npos)
                break;
            if (++num_line>=1024 && rnx_type_.compare(" ")==0) return false; /* no rinex file */
        }

        signal_index[SYS_INDEX_GPS].GnssSignalIndex(SYS_GPS,obs_type_code_[SYS_INDEX_GPS]);
        signal_index[SYS_INDEX_BDS].GnssSignalIndex(SYS_BDS,obs_type_code_[SYS_INDEX_BDS]);
        signal_index[SYS_INDEX_GAL].GnssSignalIndex(SYS_GAL,obs_type_code_[SYS_INDEX_GAL]);
        signal_index[SYS_INDEX_GLO].GnssSignalIndex(SYS_GLO,obs_type_code_[SYS_INDEX_GLO]);
        signal_index[SYS_INDEX_QZS].GnssSignalIndex(SYS_QZS,obs_type_code_[SYS_INDEX_QZS]);

        return true;

    }

    bool ReadGnssObs::Reading() {
        if(!OpenFile()){
            LOG(ERROR)<<"Open gnss obs file error: "<<file_;
            return false;
        }

        if(!ReadHead()) return false;

        int epoch_sat_num;
        while((epoch_sat_num=ReadObsBody())>=0){
            if(inf_.eof()) break;
        }

        *gnss_data_.GetEpochNum()=gnss_data_.GetGnssObs().size();

        if(OpenFile()) CloseFile();
        return true;
    }

    ReadGnssNav::ReadGnssNav() {}

    ReadGnssNav::ReadGnssNav(string file_path, PPPLib::tNav &nav) {
        file_=file_path;
        nav_=nav;
    }

    ReadGnssNav::~ReadGnssNav() {}

    void ReadGnssNav::DecodeEph(PPPLib::cTime toc, PPPLib::cSat sat, PPPLib::tBrdEphUnit &brd_eph) {
        brd_eph.sat=sat;
        brd_eph.toc=toc;

        brd_eph.f0=eph_data_[0];
        brd_eph.f1=eph_data_[1];
        brd_eph.f2=eph_data_[2];

        brd_eph.A=SQR(eph_data_[10]); brd_eph.e=eph_data_[ 8]; brd_eph.i0  =eph_data_[15]; brd_eph.Omg0=eph_data_[13];
        brd_eph.omg =eph_data_[17]; brd_eph.M0 =eph_data_[ 6]; brd_eph.deln=eph_data_[ 5]; brd_eph.Omgd=eph_data_[18];
        brd_eph.idot=eph_data_[19]; brd_eph.crc=eph_data_[16]; brd_eph.crs =eph_data_[ 4]; brd_eph.cuc =eph_data_[ 7];
        brd_eph.cus =eph_data_[ 9]; brd_eph.cic=eph_data_[12]; brd_eph.cis =eph_data_[14];

        if(sat.sat_.sys==SYS_GPS||sat.sat_.sys==SYS_QZS){
            brd_eph.iode=(int)eph_data_[3];
            brd_eph.iodc=(int)eph_data_[26];
            brd_eph.toes=     eph_data_[11];
            brd_eph.week=     eph_data_[21];
            brd_eph.toe.Gpst2Time(brd_eph.week,eph_data_[11],SYS_GPS)->AdjWeek(toc);
            brd_eph.ttr.Gpst2Time(brd_eph.week,eph_data_[27],SYS_GPS)->AdjWeek(toc);

            brd_eph.code=(int)eph_data_[20];
            brd_eph.svh =(int)eph_data_[24];
            brd_eph.sva =0;
            brd_eph.tgd[0]=eph_data_[25];
        }
        else if(sat.sat_.sys==SYS_GAL){
            brd_eph.iode=(int)eph_data_[3];
            brd_eph.toes=     eph_data_[11];
            brd_eph.week=(int)eph_data_[21];
            brd_eph.toe.Gpst2Time(brd_eph.week,eph_data_[11],SYS_GPS)->AdjWeek(toc);
            brd_eph.ttr.Gpst2Time(brd_eph.week,eph_data_[27],SYS_GPS)->AdjWeek(toc);
            brd_eph.code=(int)eph_data_[20];
            brd_eph.svh =(int)eph_data_[24];
            brd_eph.sva=0;
            brd_eph.tgd[0]=eph_data_[25];
            brd_eph.tgd[1]=eph_data_[26];
        }
        else if(sat.sat_.sys==SYS_BDS){
            cTime toc_temp=toc;
            brd_eph.toc=toc_temp.Bdst2Gpst();
            brd_eph.iode=(int)eph_data_[3];
            brd_eph.iodc=(int)eph_data_[28];
            brd_eph.toes=     eph_data_[11];
            brd_eph.week=(int)eph_data_[21];
            brd_eph.toe.Gpst2Time(brd_eph.week,eph_data_[11],SYS_BDS)->Bdst2Gpst();
            brd_eph.ttr.Gpst2Time(brd_eph.week,eph_data_[27],SYS_BDS)->Bdst2Gpst();
            brd_eph.toe.AdjWeek(toc);
            brd_eph.ttr.AdjWeek(toc);
            brd_eph.svh=(int)eph_data_[24];
            brd_eph.sva=0;
            brd_eph.tgd[0]=eph_data_[25];
            brd_eph.tgd[1]=eph_data_[26];
        }
        else if(sat.sat_.sys==SYS_IRN){
            brd_eph.iode=(int)eph_data_[3];
            brd_eph.toes=     eph_data_[11];
            brd_eph.week=(int)eph_data_[21];
            brd_eph.toe.Gpst2Time(brd_eph.week,eph_data_[11],SYS_GPS)->AdjWeek(toc);
            brd_eph.ttr.Gpst2Time(brd_eph.week,eph_data_[27],SYS_GPS)->AdjWeek(toc);
            brd_eph.svh=(int)eph_data_[24];
            brd_eph.sva=0;
            brd_eph.tgd[0]=eph_data_[25];
        }

        if(brd_eph.iode<0||1023<brd_eph.iode) brd_eph.svh=-1;
        if(brd_eph.iodc<0||1023<brd_eph.iodc) brd_eph.svh=-1;
    }

    void ReadGnssNav::DecodeGloEph(PPPLib::cTime toc, PPPLib::cSat sat, PPPLib::tBrdGloEphUnit &glo_eph) {
        double tow,tod;
        int week,dow;
        cTime tof;

        glo_eph.sat=sat;
        tow=toc.Time2Gpst(&week, &dow,SYS_GPS);

        tod=rnx_ver_<=2.99?eph_data_[2]:fmod(eph_data_[2],86400.0);
        tof.Gpst2Time(week,tod+dow*86400.0,SYS_GPS);
        tof.AdjDay(toc);

        cTime t1,t2;
        t1=toc;t2=toc;
        glo_eph.toe.Utc2Gpst();
        glo_eph.tof.Utc2Gpst();

        glo_eph.iode=(int)(fmod(tow+10800.0,86400.0)/900.0+0.5);

        glo_eph.taun=-eph_data_[0];
        glo_eph.gamn= eph_data_[1];

        glo_eph.pos[0]=eph_data_[3]*1E3; glo_eph.pos[1]=eph_data_[7]*1E3; glo_eph.pos[2]=eph_data_[11]*1E3;
        glo_eph.vel[0]=eph_data_[4]*1E3; glo_eph.vel[1]=eph_data_[8]*1E3; glo_eph.vel[2]=eph_data_[12]*1E3;
        glo_eph.acc[0]=eph_data_[5]*1E3; glo_eph.acc[1]=eph_data_[9]*1E3; glo_eph.acc[2]=eph_data_[13]*1E3;

        glo_eph.svh=(int)eph_data_[ 6];
        glo_eph.frq=(int)eph_data_[10];
        glo_eph.age=(int)eph_data_[14];

        if (glo_eph.frq>128) glo_eph.frq-=256;

        if(glo_eph.frq<GLO_MIN_FREQ||GLO_MAX_FREQ<glo_eph.frq){
            glo_eph.svh=-1;
        }
    }

    bool ReadGnssNav::ReadNavBody() {
        int i=0,j,sp=3,prn=0,flag=0,sys=SYS_GPS;
        string id;
        cSat sat;
        cTime toc;

        while(getline(inf_,line_str_)&&!inf_.eof()){
            if(line_str_.compare(0,3,"   ")!=0) i=0;
            if(i==0){
                if(rnx_ver_>=3.0||sat_sys_==SYS_GAL||sat_sys_==SYS_QZS||sat_sys_==SYS_NONE){
                    id=line_str_.substr(0,3);
                    sat=cSat(id);
                    sp=4;
                    if(rnx_ver_>=3.0) sys=sat.sat_.sys;
                }
                else{
                    Str2Int(line_str_.substr(0,2),prn);
                    if(sat_sys_==SYS_GLO) sat=cSat(SYS_GLO,prn);
                    else sat=cSat(SYS_GPS,prn);
                }
                if(toc.Str2Time(line_str_.substr(sp,19))) flag=0;
                else flag=1;

                for(j=0;j<3;j++){
                    Str2Double(line_str_.substr(sp+19*(j+1),19), eph_data_[i++]);
                }
            }
            else if(flag==1){
                for(j=0;j<4;j++){
                    if(sp+19*(j+1)<=line_str_.size()){
                        Str2Double(line_str_.substr(sp+19*j,19),eph_data_[i++]);
                    }
                    else eph_data_[i++]=0.0;
                }

                if(sys==SYS_GLO&&i>15){
                    if(!(sys_mask_&sys)) continue;
                    tBrdGloEphUnit glo_eph={0};
                    DecodeGloEph(toc,sat,glo_eph);
                    nav_.brd_glo_eph.push_back(glo_eph);
                }
                else if(i>=31){
                    if(!(sys_mask_&sys)) continue;
                    tBrdEphUnit eph={0};
                    DecodeEph(toc,sat,eph);
                    nav_.brd_eph.push_back(eph);
                }
            }
        }
    }

    bool ReadGnssNav::ReadHead() {
        int nline=0,i,j;

        if(!ReadRnxHead()) return false;

        while(getline(inf_,line_str_)&&!inf_.eof()){

            if (line_str_.find("IONOSPHERIC CORR",60)!=string::npos) { /* opt ver.3 */
                if (line_str_.compare(0,4,"GPSA")==0) {
                    for (i=0,j=5;i<4;i++,j+=12) Str2Double(line_str_.substr(j,12),nav_.ion_para[SYS_INDEX_GPS][i]);
                }
                else if (line_str_.compare(0,4,"GPSB")==0) {
                    for (i=0,j=5;i<4;i++,j+=12) Str2Double(line_str_.substr(j,12),nav_.ion_para[SYS_INDEX_GPS][i+4]);
                }
                else if (line_str_.compare(0,3,"GAL")==0) {
                    for (i=0,j=5;i<4;i++,j+=12) Str2Double(line_str_.substr(j,12),nav_.ion_para[SYS_INDEX_GAL][i]);
                }
                else if (line_str_.compare(0,4,"QZSA")==0) { /* v.3.02 */
                    for (i=0,j=5;i<4;i++,j+=12) Str2Double(line_str_.substr(j,12),nav_.ion_para[SYS_INDEX_QZS][i]);
                }
                else if (line_str_.compare(0,4,"QZSB")==0) { /* v.3.02 */
                    for (i=0,j=5;i<4;i++,j+=12) Str2Double(line_str_.substr(j,12),nav_.ion_para[SYS_INDEX_QZS][i+4]);
                }
                else if (line_str_.compare(0,4,"BDSA")==0) { /* v.3.02 */
                    for (i=0,j=5;i<4;i++,j+=12) Str2Double(line_str_.substr(j,12),nav_.ion_para[SYS_INDEX_BDS][i]);
                }
                else if (line_str_.compare(0,4,"BDSB")==0) { /* v.3.02 */
                    for (i=0,j=5;i<4;i++,j+=12) Str2Double(line_str_.substr(j,12),nav_.ion_para[SYS_INDEX_BDS][i+4]);
                }
                else if (line_str_.compare(0,4,"IRNA")==0) { /* v.3.03 */
                    for (i=0,j=5;i<4;i++,j+=12) Str2Double(line_str_.substr(j,12),nav_.ion_para[SYS_INDEX_IRN][i]);
                }
                else if (line_str_.compare(0,4,"IRNB")==0) { /* v.3.03 */
                    for (i=0,j=5;i<4;i++,j+=12) Str2Double(line_str_.substr(j,12),nav_.ion_para[SYS_INDEX_IRN][i+4]);
                }
            }
            else if (line_str_.find("TIME SYSTEM CORR",60)!=string::npos) { /* opt ver.3 */
                if (line_str_.compare(0,4,"GPUT")==0) {
                    Str2Double(line_str_.substr( 5,17),nav_.utc_para[SYS_INDEX_GPS][0]);
                    Str2Double(line_str_.substr(22,16),nav_.utc_para[SYS_INDEX_GPS][1]);
                    Str2Double(line_str_.substr(38, 7),nav_.utc_para[SYS_INDEX_GPS][2]);
                    Str2Double(line_str_.substr(45, 5),nav_.utc_para[SYS_INDEX_GPS][3]);
                }
                else if (line_str_.compare(0,4,"GLUT")==0) {
                    Str2Double(line_str_.substr( 5,17),nav_.utc_para[SYS_INDEX_GLO][0]);
                    Str2Double(line_str_.substr(22,16),nav_.utc_para[SYS_INDEX_GLO][1]);
                    Str2Double(line_str_.substr(38, 7),nav_.utc_para[SYS_INDEX_GLO][2]);
                    Str2Double(line_str_.substr(45, 5),nav_.utc_para[SYS_INDEX_GLO][3]);
                }
                else if (line_str_.compare(0,4,"GAUT")==0) { /* v.3.02 */
                    Str2Double(line_str_.substr( 5,17),nav_.utc_para[SYS_INDEX_GAL][0]);
                    Str2Double(line_str_.substr(22,16),nav_.utc_para[SYS_INDEX_GAL][1]);
                    Str2Double(line_str_.substr(38, 7),nav_.utc_para[SYS_INDEX_GAL][2]);
                    Str2Double(line_str_.substr(45, 5),nav_.utc_para[SYS_INDEX_GAL][3]);
                }
                else if (line_str_.compare(0,4,"QZUT")==0) { /* v.3.02 */
                    Str2Double(line_str_.substr( 5,17),nav_.utc_para[SYS_INDEX_GAL][0]);
                    Str2Double(line_str_.substr(22,16),nav_.utc_para[SYS_INDEX_GAL][1]);
                    Str2Double(line_str_.substr(38, 7),nav_.utc_para[SYS_INDEX_GAL][2]);
                    Str2Double(line_str_.substr(45, 5),nav_.utc_para[SYS_INDEX_GAL][3]);
                }
                else if (line_str_.compare(0,4,"BDUT")==0) { /* v.3.02 */
                    Str2Double(line_str_.substr( 5,17),nav_.utc_para[SYS_INDEX_BDS][2]);
                    Str2Double(line_str_.substr(22,16),nav_.utc_para[SYS_INDEX_BDS][2]);
                    Str2Double(line_str_.substr(38, 7),nav_.utc_para[SYS_INDEX_BDS][2]);
                    Str2Double(line_str_.substr(45, 5),nav_.utc_para[SYS_INDEX_BDS][3]);
                }
                else if (line_str_.compare(0,4,"IRUT")==0) { /* v.3.03 */
                    Str2Double(line_str_.substr( 5,17),nav_.utc_para[SYS_INDEX_IRN][0]);
                    Str2Double(line_str_.substr(22,16),nav_.utc_para[SYS_INDEX_IRN][1]);
                    Str2Double(line_str_.substr(38, 7),nav_.utc_para[SYS_INDEX_IRN][2]);
                    Str2Double(line_str_.substr(45, 5),nav_.utc_para[SYS_INDEX_IRN][3]);
                }
            }
            else if (line_str_.find("LEAP SECONDS",60)!=string::npos) { /* opt */
                Str2Int(line_str_.substr(0,6),nav_.leaps);
            }
            if (line_str_.find("END OF HEADER")!=string::npos) return true;
            if (++nline>=1024 && rnx_type_.compare(" ")==0) break; /* no rinex file */
        }
        return false;
    }

    bool ReadGnssNav::Reading() {
        ReadNavBody();
    }

}







