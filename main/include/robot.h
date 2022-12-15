//
// Created by Kevin Williams on 11/28/22.
//

#pragma once


#include "app_drive.h"
#include "app_auto_drive.h"
#include "app_look.h"
#include "app_proximity.h"

class Robot {

public:
    Robot(AppDrive *pAppDrive, AppAutoDrive *pAppAutoDrive, AppLook *pAppLook, AppProximity *pAppProximity);
    AppDrive * appDrive;
    AppAutoDrive * appAutoDrive;
    AppLook * appLook;
    AppProximity * appProximity;
    TOFArray * tofArray;
    void run();

private:


};


