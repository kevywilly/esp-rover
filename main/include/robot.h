//
// Created by Kevin Williams on 11/28/22.
//

#pragma once


#include "app_drive.h"
#include "app_auto_drive.h"

class Robot {

public:
    Robot(AppDrive *pAppDrive, AppAutoDrive *pAppAutoDrive);
    AppDrive * appDrive;
    AppAutoDrive * appAutoDrive;
    TOFArray * tofArray;
    void run();

private:


};


