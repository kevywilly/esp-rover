//
// Created by Kevin Williams on 11/28/22.
//

#include "robot.hpp"

Robot::Robot(AppDrive *pAppDrive, AppAutoDrive *pAppAutoDrive, AppLook * pAppLook) : appDrive(pAppDrive), appAutoDrive(pAppAutoDrive), appLook(pAppLook) {
    tofArray = new TOFArray();
    tofArray->init();
    pAppAutoDrive->tofArray = tofArray;
}

void Robot::run() {
    appDrive->run();
    //appLook->run();
    appAutoDrive->run();
}
