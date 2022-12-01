//
// Created by Kevin Williams on 11/28/22.
//

#ifndef ESPROVER_ROBOT_H
#define ESPROVER_ROBOT_H

#include <app_drive.hpp>
#include <app_auto_drive.hpp>
#include "app_look.hpp"

class Robot {

public:
    Robot(AppDrive *pAppDrive, AppAutoDrive *pAppAutoDrive, AppLook *pAppLook);
    AppDrive * appDrive;
    AppAutoDrive * appAutoDrive;
    AppLook * appLook;
    TOFArray * tofArray;
    void run();

private:


};


#endif //ESPROVER_ROBOT_H
