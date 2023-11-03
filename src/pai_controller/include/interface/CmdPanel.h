#ifndef CMDPANEL_H
#define CMDPANEL_H

#include "../common/enumClass.h"
#include "../messages/LowlevelState.h"
#include <pthread.h>

struct WaypointCmd{
    float x;
    float y;
    float yaw;
    int mode;
    WaypointCmd(){
        setZero();
    }
    void setZero(){
        x = 0;
        y = 0; 
        yaw = 0;
        mode = 0;
    }
};

class CmdPanel{
public:
    CmdPanel(){}
    ~CmdPanel(){}
    UserCommand getUserCmd(){return userCmd;}
    UserValue getUserValue(){return userValue;}
    void setPassive(){userCmd = UserCommand::Passive;}
    void setZero(){userValue.setZero();}
    virtual void updateVelCmd(const LowlevelState *state){};

protected:
    virtual void *run(void *arg){};
    UserCommand userCmd;
    UserValue userValue;
};

#endif  // CMDPANEL_H