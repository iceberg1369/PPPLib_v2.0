//
// Created by chenc on 2020/7/13.
//

#ifndef PPPLIB_READFILES_H
#define PPPLIB_READFILES_H

#include "GnssFunc.h"
#include "InsFunc.h"

namespace PPPLib {

    class cReadFile {
    public:
        cReadFile();
        cReadFile(string file_path);
        ~cReadFile();

    public:
        bool OpenFile();
        void CloseFile();
        virtual bool ReadHead();
        virtual bool Reading();

    public:
        string file_;
        string line_str_;
        ifstream inf_;
    };

    class cReadImu:public cReadFile {
    public:
        cReadImu();
        cReadImu(string file_path);
        ~cReadImu();

    private:
        bool DecodeImu();
        bool DecodeNovatel(double a_scale, double g_scale);

    public:
        cImuData* GetImus();
        void SetImuTimeSpan(cTime *ts, cTime *te);
        bool SetImu(IMU_TYPE imu_type,IMU_COORD_TYPE coord_type,IMU_DATA_FORMAT data_format);
        bool Reading() override;
        void OutImu(string out_path);

    private:
        cImuData imu_data_;
    };

    class cReadPos:public cReadFile {
    public:
        cReadPos();
        cReadPos(string file_path);
        ~cReadPos();

    private:
        bool DecodePos();

    public:
        bool Reading() override;

    private:
//        cPosData poss_;
    };

    class cReadRnx:public cReadFile {
    public:
        cReadRnx();
        cReadRnx(string file_path);
        virtual ~cReadRnx();

    public:
        virtual void SetGnssSysMask(int mask);
        bool ReadRnxHead();

    public:
        double rnx_ver_;
        string rnx_type_;
        int sat_sys_;
        int time_sys_;
        int sys_mask_;
    };

    typedef struct{
        int n;
        int frq[MAX_GNSS_OBS_TYPE];
        int pos[MAX_GNSS_OBS_TYPE];
        unsigned char pri[MAX_GNSS_OBS_TYPE];
        unsigned char type[MAX_GNSS_OBS_TYPE];
        unsigned char code[MAX_GNSS_OBS_TYPE];
        double shift[MAX_GNSS_OBS_TYPE];
    }tGnssSignal;

    class cGnssSignal{
    public:
        cGnssSignal();
        ~cGnssSignal();

    public:
        tGnssSignal* GetGnssSignal();
        unsigned char Signal2Code(string signal,int* frq,int sat_sys);
        string Code2Signal(unsigned char code,int* frq,int sat_sys);
        int GetCodePri(int sat_sys, unsigned char code);
        void GnssSignalIndex(int sat_sys,string obs_code_type[MAX_GNSS_CODE_TYPE]);

    private:
        tGnssSignal signal_={0};
    };

    class cReadGnssObs:public cReadRnx {
        public:
            cReadGnssObs();
            cReadGnssObs(string file_path,tNav& nav,cGnssObs& obss,RECEIVER_INDEX rcv);
            ~cReadGnssObs();

        private:
            static bool CmpEpochSatData(const tSatObsUnit& p1, const tSatObsUnit& p2);
            bool SortEpochSatData(tEpochSatUnit& epoch_data);
            int DecodeEpoch(cTime& t,int& obs_flag);
            bool DecodeEpochSatObs(tSatObsUnit& obs);
            int ReadObsBody();

        public:
            void SetGnssTimeSpan(cTime* ts,cTime* te);
            cGnssObs* GetGnssData();
            tNav* GetGnssNav();
            bool ReadHead() override;
            bool Reading() override;

        private:
            cGnssObs* gnss_data_;
            tNav* nav_;
            string obs_type_code_[NSYS][MAX_GNSS_OBS_TYPE];
            cGnssSignal signal_index[NSYS];
    };

    class cReadGnssBrdEph:public cReadRnx {
    public:
        cReadGnssBrdEph();
        cReadGnssBrdEph(string file_path, tNav& nav);
        ~cReadGnssBrdEph();

    private:
        static bool CmpBrdEph(const tBrdEphUnit& p1,const tBrdEphUnit& p2);
        static bool CmpBrdGloEph(const tBrdGloEphUnit& p1, const tBrdGloEphUnit& p2);
        bool SortBrdEph();
        bool SortBrdGloEph();
        void ClearEphData();
        void DecodeEph(cTime toc, cSat sat, tBrdEphUnit& brd_eph);
        void DecodeGloEph(cTime toc,const cSat& sat,tBrdGloEphUnit& glo_eph);
        bool ReadBrdBody();

    public:
        tNav* GetGnssNav();
        bool ReadHead() override;
        bool Reading() override;

    private:
        double eph_data_[64]={0};
        tNav* nav_;
    };

    class cReadGnssPreEph:public cReadFile {
    public:
        cReadGnssPreEph();
        cReadGnssPreEph(string file_path,tNav& nav);
        ~cReadGnssPreEph();

    private:
        void ReadPreOrbHead();
        void ReadPreOrbBody();
        void ReadPreClkHead();
        void ReadPreClkBody();

    public:
        void SetGnssSatMask(int mask);
        void ReadHead(int type);
        bool Reading(int type);

    private:
        cTime pre_eph_time_;
        int num_sat_;
        int sys_mask_;
        tNav* nav_;
    };

    class cReadGnssCodeBias:public cReadFile {
    public:
        cReadGnssCodeBias();
        cReadGnssCodeBias(string file_path,tNav& nav);
        ~cReadGnssCodeBias();

    private:
        void DecodeCasMgexDcb();

    public:
        bool Reading() override;

    private:
        tNav* nav_;
    };

    class cReadGnssErp:public cReadFile {
    public:
        cReadGnssErp();
        cReadGnssErp(string file_path,tNav& nav);
        ~cReadGnssErp();

    private:
        void DecodeErpPara();

    public:
        bool Reading() override;

    private:
        tNav* nav_;
    };

    class cReadGnssOcean:public cReadFile {
    public:
        cReadGnssOcean();
        cReadGnssOcean(string file_path,tNav& nav,string site,RECEIVER_INDEX idx);
        ~cReadGnssOcean();

    private:
        void DecodeOceanPara();

    public:
        bool Reading() override;

    private:
        RECEIVER_INDEX index_;
        string site_name_;
        tNav* nav_;
    };

    class cReadGnssAntex:public cReadFile {
    public:
        cReadGnssAntex();
        cReadGnssAntex(string file_path,tNav& nav);
        ~cReadGnssAntex();

    private:
        tAntUnit* SearchAntPar(cTime t,int sat,const string& type);
        int DecodeAntPcv(char* p,int n,double *v);
        void ReadAntBody();

    public:
        void AlignAntPar2Sat(tPPPLibConf C,cTime t,tStaInfoUnit* sta,tAntUnit* sat_ant, tAntUnit* rec_ant);
        bool Reading() override;

    private:
        vector<tAntUnit> ant_paras_;
    };

    class cReadGnssIonex:public cReadFile {
    public:
        cReadGnssIonex();
        cReadGnssIonex(string file_path,tNav& nav);
        ~cReadGnssIonex();

    private:
        int DataIndex(int i,int j,int k,const int* ndata);
        int GetIndex(double val,const double* range);
        int GetNumItems(const double* range);
        tTecUnit* AddTec();
        bool ReadHead() override;
        void ReadIonBody();

    public:
        bool Reading() override;

    private:
        tNav nav_;
        cTime ion_time_;
        double factor_,re_;
        double lats_[3],lons_[3],hgts_[3];
    };

    class cReadRefSol:public cReadFile {
    public:
        cReadRefSol();
        cReadRefSol(string file_path,vector<tSolInfoUnit>& ref_sol);
        ~cReadRefSol();

    private:
        void ReadRefBody();

    public:
        vector<tSolInfoUnit> GetRefSols();
        bool Reading() override;

    private:
        vector<tSolInfoUnit>* ref_sols_;
    };
}

#endif //PPPLIB_READFILES_H
