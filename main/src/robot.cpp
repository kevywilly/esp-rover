//
// Created by Kevin Williams on 11/28/22.
//

#include "robot.hpp"

Robot::Robot(AppDrive *pAppDrive, AppAutoDrive *pAppAutoDrive) : appDrive(pAppDrive), appAutoDrive(pAppAutoDrive) {
    tofArray = new TOFArray();
    tofArray->init();
    pAppAutoDrive->tofArray = tofArray;
}

void Robot::run() {
    appDrive->run();
    appAutoDrive->run();
}
