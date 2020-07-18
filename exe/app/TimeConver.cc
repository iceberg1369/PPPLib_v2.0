//
// Created by cc on 7/10/20.
//

#include "CmnFunc.h"

using namespace PPPLib;


int main(int argc,char** argv){
    char s[256];
    cout<<"please enter: yyyy mm dd hh mm ss.s"<<endl<<endl;
    cin.getline(s,22);
    cTime t(s);
    int gweek,gsec,gday;
    gsec=t.Time2Gpst(&gweek,&gday,SYS_GPS);
    t.GetTimeStr();

    cout<<"Time format: "<<t.GetTime()->long_time<<" "<<t.GetTime()->sec<<endl;
    cout<<"Day of year: "<<(int)t.GetEpoch()[0]<<" "<<t.GetDoy()<<endl;
    cout<<"GPS time:    "<<gweek<<" "<<gsec<<" "<<gday<<endl;
    int bweek,bsec,bday;
    bsec=t.Time2Gpst(&bweek,&bday,SYS_BDS);
    cout<<"BDS time:    "<<bweek<<" "<<bsec<<" "<<bday<<endl;
    cTime tu;
//    tu=*t.Gpst2Utc();
    cout<<"UTC time:    "<<t.GetTime()->long_time<<" "<<t.GetTime()->sec<<endl;
    cout<<"MJD time:    "<<t.GetMjd()->day<<" "<<t.GetMjd()->sod.sn<<" "<<t.GetMjd()->sod.tos<<endl;
}

