//
// Created by cc on 7/10/20.
//

#ifndef PPPLIB_CONSTANT_H
#define PPPLIB_CONSTANT_H

namespace PPPLib {

    #define SQR(x) ((x)*(x))

    // common constant definitions
    const static double PI=3.14159265358979;
    const static double CLIGHT=299792458.0;
    const static double D2R=(PI/180.0);
    const static double R2D=(180.0/PI);

    enum TIME_TYPE {
        TIME_TYPE_NONE = 0,
        TIME_utc,
        TIME_GPS,
        TIME_BDS,
        TIME_GAL,
        TIME_GLO,
        TIME_QZS
    };

    enum SAT_SYS_TYPE {
        SYS_NONE=0x00,
        SYS_GPS=0x01,
        SYS_BDS=0x02,
        SYS_GAL=0x04,
        SYS_GLO=0x08,
        SYS_QZS=0x10,
        SYS_IRN=0x20,
        SYS_SBS=0x40,
        SYS_ALL=0xFF
    };

    const static double kSysTimeStart[] = {1970, 1, 1, 0, 0, 0};
    const static double kGpsTimeStart[] =  {1980, 1, 6, 0, 0, 0};
    const static double kGalTimeStart[] =  {1999, 8, 22, 0, 0, 0};
    const static double kBdsTimeStart[] =  {2006, 1, 1, 0, 0, 0};
    const static double kUtcLeapSec[65][7]={
            {2017,1,1,0,0,0,-18}, {2015,7,1,0,0,0,-17}, {2012,7,1,0,0,0,-16},
            {2009,1,1,0,0,0,-15}, {2006,1,1,0,0,0,-14}, {1999,1,1,0,0,0,-13},
            {1997,7,1,0,0,0,-12}, {1996,1,1,0,0,0,-11}, {1994,7,1,0,0,0,-10},
            {1993,7,1,0,0,0, -9}, {1992,7,1,0,0,0, -8}, {1991,1,1,0,0,0, -7},
            {1990,1,1,0,0,0, -6}, {1988,1,1,0,0,0, -5}, {1985,7,1,0,0,0, -4},
            {1983,7,1,0,0,0, -3}, {1982,7,1,0,0,0, -2}, {1981,7,1,0,0,0, -1},
            {0}
    };

    const double WGS84_EARTH_LONG_RADIUS=6378137.0;
    const double WGS84_EARTH_SHORT_RADIUS=6356752.3142;
    const double WGS84_EARTH_OBLATEO=1.0/298.257223563;
    const double WGS84_FIRST_E2=0.00669437999013;
    const double WGS84_SECOND_E2=0.006739496742227;

    const int COORDINATE_ACCURACY=4;  // 空间直角坐标系以及大地坐标系高程方向精确度
    const int MSEC_ACCURACY=3;        // 秒的精确度(ms)
    const int MATRIX_ACCURACY=6;      // 矩阵中double数据与角分秒格式中的秒的精确度
    const int DEGREE_ACCURACY=11;     // 大地坐标中经纬度与小数度的精确度(小数后11位)
    const double LAT_ACCURACY=1.0e-11;// 计算大地纬度B的精度

    enum COORD_FRAME_TYPE {
        WGS84=1,
        CGCS2000,
        ITRF96,
        PZ90
    };

    enum COORDINATE_TYPE {
        COORD_BLH=1,
        COORD_XYZ,
        COORD_ENU,
        COORD_NED
    };

    // gnss related constant definitions
    const int GNSS_NUM_FREQ=5;
    const int GNSS_NUM_EXOBS=1;
    const int MAX_GNSS_OBS_TYPE=36;
    const int MAX_GNSS_FRQ_NUM=6;
    static const string kSatSysCode="GCERJI";
    static const string kGnssObsCode="CLDS";

    enum GPS{
        GPS_MIN_PRN=1,
        GPS_MAX_PRN=32,
        NUM_GPS_SAT=(GPS_MAX_PRN-GPS_MIN_PRN+1),
        NSYS_GPS=1,
        SYS_INDEX_GPS=0,
    };

    enum BDS{
        BDS_MIN_PRN=1,
        BDS_MAX_PRN=45,
        NUM_BDS_SAT=(BDS_MAX_PRN-BDS_MIN_PRN+1),
        NSYS_BDS=1,
        SYS_INDEX_BDS=1
    };

    enum GAL{
        GAL_MIN_PRN=1,
        GAL_MAX_PRN=36,
        NUM_GAL_SAT=(GAL_MAX_PRN-GAL_MIN_PRN+1),
        NSYS_GAL=1,
        SYS_INDEX_GAL=2
    };

    enum GLO{
        GLO_MIN_PRN=1,
        GLO_MAX_PRN=27,
        NUM_GLO_SAT=(GLO_MAX_PRN-GLO_MIN_PRN+1),
        NSYS_GLO=1,
        SYS_INDEX_GLO=3
    };

    enum QZS{
        QZS_MIN_PRN=193,
        QZS_MAX_PRN=201,
        NUM_QZS_SAT=(QZS_MAX_PRN-QZS_MIN_PRN+1),
        NSYS_QZS=1,
        SYS_INDEX_QZS=4
    };

    enum IRN{
        IRN_MIN_PRN=0,
        IRN_MAX_PRN=10,
        NUM_IRN_SAT=(IRN_MAX_PRN-IRN_MIN_PRN+1),
        NSYS_IRN=1,
        SYS_INDEX_IRN=5
    };
    const int NSYS=NSYS_GPS+NSYS_BDS+NSYS_GAL+NSYS_GLO+NSYS_QZS+NSYS_IRN;
    const int MAX_SAT_NUM=NUM_GPS_SAT+NUM_BDS_SAT+NUM_GAL_SAT+NUM_GLO_SAT+NUM_QZS_SAT+NUM_IRN_SAT;

    enum RECEIVER_INDEX {
        REC_ROVER,
        REC_BASE
    };

    constexpr double MHZ_TO_HZ=1000000.0;
    constexpr double FREQ_NONE=0.0;
    constexpr double FREQ_GPS_L1=1575.42*MHZ_TO_HZ;
    constexpr double FREQ_GPS_L2=1227.60*MHZ_TO_HZ;
    constexpr double FREQ_GPS_L5=1176.45*MHZ_TO_HZ;
    constexpr double FREQ_BDS_B1=1561.098*MHZ_TO_HZ;
    constexpr double FREQ_BDS_B2=1207.140*MHZ_TO_HZ;
    constexpr double FREQ_BDS_B3=1268.52*MHZ_TO_HZ;
    constexpr double FREQ_BDS_B1C=1575.42*MHZ_TO_HZ;
    constexpr double FREQ_BDS_B2A=1176.45*MHZ_TO_HZ;
    constexpr double FREQ_BDS_B2B=1207.140*MHZ_TO_HZ;
    constexpr double FREQ_GAL_E1=1575.42*MHZ_TO_HZ;
    constexpr double FREQ_GAL_E5A=1176.45*MHZ_TO_HZ;
    constexpr double FREQ_GAL_E5B=1207.140*MHZ_TO_HZ;
    constexpr double FREQ_GAL_E5AB=1191.795*MHZ_TO_HZ;
    constexpr double FREQ_GAL_E6=1278.75*MHZ_TO_HZ;
    constexpr double FREQ_GLO_G1=1602.00*MHZ_TO_HZ;
    constexpr double FREQ_GLO_G2=1246.00*MHZ_TO_HZ;
    constexpr double FREQ_GLO_D1=0.5625*MHZ_TO_HZ;
    constexpr double FREQ_GLO_D2=0.4375*MHZ_TO_HZ;
    constexpr double FREQ_QZS_L1=1575.42*MHZ_TO_HZ;
    constexpr double FREQ_QZS_L2=1227.60*MHZ_TO_HZ;
    constexpr double FREQ_QZS_L5=1176.45*MHZ_TO_HZ;
    constexpr double FREQ_QZS_LEX=1278.75*MHZ_TO_HZ;
    constexpr int GLO_MIN_FREQ=-7;
    constexpr int GLO_MAX_FREQ=13;

    const int GNSS_CODE_NONE=0;
    const int GNSS_CODE_L1C=1;
    const int GNSS_CODE_L1S=2;
    const int GNSS_CODE_L1L=3;
    const int GNSS_CODE_L1X=4;
    const int GNSS_CODE_L1P=5;
    const int GNSS_CODE_L1W=6;
    const int GNSS_CODE_L1Y=7;
    const int GNSS_CODE_L1M=8;
    const int GNSS_CODE_L1A=9;
    const int GNSS_CODE_L1B=10;
    const int GNSS_CODE_L1Z=11;
    const int GNSS_CODE_L1D=12;
    const int GNSS_CODE_L2C=13;
    const int GNSS_CODE_L2D=14;
    const int GNSS_CODE_L2S=15;
    const int GNSS_CODE_L2L=16;
    const int GNSS_CODE_L2X=17;
    const int GNSS_CODE_L2P=18;
    const int GNSS_CODE_L2W=19;
    const int GNSS_CODE_L2Y=20;
    const int GNSS_CODE_L2M=21;
    const int GNSS_CODE_L2I=22;
    const int GNSS_CODE_L2Q=23;
    const int GNSS_CODE_L3I=24;
    const int GNSS_CODE_L3Q=25;
    const int GNSS_CODE_L3X=26;
    const int GNSS_CODE_L4A=27;
    const int GNSS_CODE_L4B=28;
    const int GNSS_CODE_L4X=29;
    const int GNSS_CODE_L5I=30;
    const int GNSS_CODE_L5Q=31;
    const int GNSS_CODE_L5X=32;
    const int GNSS_CODE_L5D=33;
    const int GNSS_CODE_L5P=34;
    const int GNSS_CODE_L5Z=35;
    const int GNSS_CODE_L6A=36;
    const int GNSS_CODE_L6B=37;
    const int GNSS_CODE_L6X=38;
    const int GNSS_CODE_L6C=39;
    const int GNSS_CODE_L6Z=40;
    const int GNSS_CODE_L6S=41;
    const int GNSS_CODE_L6L=42;
    const int GNSS_CODE_L6E=43;
    const int GNSS_CODE_L6I=44;
    const int GNSS_CODE_L6Q=45;
    const int GNSS_CODE_L7I=46;
    const int GNSS_CODE_L7Q=47;
    const int GNSS_CODE_L7X=48;
    const int GNSS_CODE_L7D=49;
    const int GNSS_CODE_L7P=50;
    const int GNSS_CODE_L7Z=51;
    const int GNSS_CODE_L8I=52;
    const int GNSS_CODE_L8Q=53;
    const int GNSS_CODE_L8X=54;
    const int GNSS_CODE_L8D=55;
    const int GNSS_CODE_L8P=56;
    const int GNSS_CODE_L8Z=57;
    const int MAX_GNSS_CODE_TYPE=57;

    const double kGnssFreqs[NSYS][MAX_GNSS_FRQ_NUM]{
            {FREQ_GPS_L1,FREQ_GPS_L2, FREQ_GPS_L5,  FREQ_NONE,    FREQ_NONE,    FREQ_NONE},
            {FREQ_BDS_B1,FREQ_BDS_B3, FREQ_BDS_B2,  FREQ_BDS_B1C, FREQ_BDS_B2A, FREQ_BDS_B2B},
            {FREQ_GAL_E1,FREQ_GAL_E5A,FREQ_GAL_E5B, FREQ_NONE,    FREQ_NONE,    FREQ_NONE},
            {FREQ_GLO_G1,FREQ_GLO_G2, FREQ_NONE,    FREQ_NONE,    FREQ_NONE,    FREQ_NONE},
            {FREQ_QZS_L1,FREQ_QZS_L2, FREQ_QZS_L5,  FREQ_NONE,    FREQ_NONE,    FREQ_NONE},
            {FREQ_NONE,  FREQ_NONE,   FREQ_NONE,    FREQ_NONE,    FREQ_NONE,    FREQ_NONE}
    };

    const string kGnssSignalCodes[]={
            "",  "1C","1S","1L","1X","1P","1W","1Y","1M","1A",
            "1B","1Z","1D","2C","2D","2S","2L","2X","2P","2W",
            "2Y","2M","2I","2Q","3I","3Q","3X","4A","4B","4X",
            "5I","5Q","5X","5D","5P","5Z","6A","6B","6X","6C",
            "6Z","6S","6L","6E","6I","6Q","7I","7Q","7X","7D",
            "7P","7Z","8I","8Q","8X","8D","8P", "8Z" , "" , ""
    };

    // GPS L1(1),L2(2),L5(5)
    const unsigned char kGpsFreqBand[]={
            0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
            0, 0, 0, 2, 2, 2, 2, 2, 2, 2,
            2, 2, 0, 0, 0, 0, 0, 0, 0, 0,
            3, 3, 3, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    //BDS B1(2),B3(6),B2(7),B1C(1),B2a(5),B2b(7)
    const unsigned char kBdsFreqBand[]={
            0, 0, 0, 0, 4, 4, 0, 0, 0, 4,
            0, 0, 4, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 5, 5, 5, 0, 2, 0, 2, 0,
            0, 0, 0, 0, 2, 2, 3, 3, 3, 6,
            6, 6, 0, 0, 0, 0, 0, 0, 0, 0,
    };
    //GAL E1(1) E5a(5) E5b(7) E5ab(8) E6(6)
    const unsigned char kGalFreqBand[]={
            0, 1, 0, 0, 1, 0, 0, 0, 0, 1,
            1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            2, 2, 2, 0, 0, 0, 5, 5, 5, 5,
            5, 0, 0, 0, 0, 0, 3, 3, 3, 0,
            0, 0, 4, 4, 4, 0, 0, 0, 0, 0,
    };

    //GLO
    const unsigned char kGloFreqBand[]={
            0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 2, 0, 0, 0, 0, 2, 0,
            0, 0, 0, 0, 3, 3, 3, 4, 4, 4,
            0, 0, 0, 0, 0, 0, 5, 5, 5, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };

    //QZS
    const unsigned char kQzsFreqBand[]={
            0, 1, 1, 1, 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 2, 2, 2, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            3, 3, 3, 3, 3, 0, 0, 0, 4, 0,
            4, 4, 4, 4, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };

    const string kGnssSignalPriors[NSYS][MAX_GNSS_FRQ_NUM]{
            {"CWPYMSLX", "CWPYMDSLX",      "IQX",         "",        "",    ""},
            {     "IQX",      "IQXA",      "IQX",     "DPXA",     "DPX", "DPZ"},
            {   "CABXZ",       "IQX",      "IQX",      "IQX",   "CABXZ",    ""},
            {      "PC",        "PC",      "IQX",      "ABX",    "ABXP",    ""},
            {   "CSLXZ",       "SLX",   "IQXDPZ",    "SLXEZ",        "",    ""},
            {        "",          "",         "",         "",        "",    ""}
    };

    enum GNSS_OBS {
        GNSS_OBS_CODE,
        GNSS_OBS_PHASE,
        GNSS_OBS_DOPPLER,
        GNSS_OBS_SNR
    };

    // ins related constant definitions
    enum IMU_TYPE {
        IMU_UNKNOW=-1,
        IMU_NOVTEL_CPT,
        IMU_NOVTEL_A1
    };

    enum IMU_COORD_TYPE {
        IMU_COORD_ENU,
        IMU_COORD_NED,
    };

}

#endif //PPPLIB_CONSTANT_H
