//
// Created by Kevin Williams on 11/28/22.
//

#include "robot.h"

Robot::Robot(
        AppDrive *pAppDrive,
        AppAutoDrive *pAppAutoDrive,
        AppLook *pAppLook,
        AppProximity *pProximity) : appDrive(pAppDrive), appAutoDrive(pAppAutoDrive), appLook(pAppLook), appProximity(pProximity) {
    tofArray = new TOFArray();
    tofArray->init();
    pAppAutoDrive->tofArray = tofArray;
}

void Robot::run() {
    appProximity->run();
    appDrive->run();
    appLook->run();
    appAutoDrive->run();
}
