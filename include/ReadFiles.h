//
// Created by chenc on 2020/7/13.
//

#ifndef PPPLIB_READFILES_H
#define PPPLIB_READFILES_H

#include "GnssFunc.h"
#include "InsFunc.h"

namespace PPPLib {

    class ReadFile {
    public:
        ReadFile();
        ReadFile(string file_path);
        ~ReadFile();

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

    class ReadImu:public ReadFile {
    public:
        ReadImu();
        ReadImu(string file_path);
        ~ReadImu();

    private:
        bool DecodeImu();
        bool DecodeNovatel(double a_scale, double g_scale);

    public:
        cImuData* GetImus();
        void SetImuTimeSpan(cTime *ts, cTime *te);
        bool SetImuType(IMU_TYPE type);
        void SetImuCoordType(IMU_COORD_TYPE type);
        bool Reading() override;
        void OutImu(string out_path);

    private:
        cImuData imu_data_;
    };

    class ReadPos:public ReadFile {
    public:
        ReadPos();
        ReadPos(string file_path);
        ~ReadPos();

    private:
        bool DecodePos();

    public:
        bool Reading() override;

    private:
//        cPosData poss_;
    };

    class ReadRnx:public ReadFile {
    public:
        ReadRnx();
        ReadRnx(string file_path);
        virtual ~ReadRnx();

    public:
        void SetGnssSysMask(int mask);
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

    class ReadGnssObs:public ReadRnx {
        public:
            ReadGnssObs();
            ReadGnssObs(string file_path,tNav& nav,RECEIVER_INDEX rcv);
            ~ReadGnssObs();

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
            int sys_mask_;
            cGnssObs gnss_data_;
            tNav nav_;
            string obs_type_code_[NSYS][MAX_GNSS_OBS_TYPE];
            cGnssSignal signal_index[NSYS];
    };

    class ReadGnssNav:public ReadRnx {
    public:
        ReadGnssNav();
        ReadGnssNav(string file_path, tNav& nav);
        ~ReadGnssNav();

    private:
        void DecodeEph(cTime toc, cSat sat, tBrdEphUnit& brd_eph);
        void DecodeGloEph(cTime toc,cSat sat, tBrdGloEphUnit& glo_eph);
        bool ReadNavBody();

    public:
        bool ReadHead() override;
        bool Reading() override;

    private:
        double eph_data_[64];
        tNav nav_;
    };
}

#endif //PPPLIB_READFILES_H
