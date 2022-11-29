//
// Created by Kevin Williams on 11/28/22.
//

#ifndef ESPROVER_ROBOT_H
#define ESPROVER_ROBOT_H

#include <app_drive.hpp>
#include <app_auto_drive.hpp>

class Robot {

public:
    Robot(AppDrive *pAppDrive, AppAutoDrive *pAppAutoDrive);
    AppDrive * appDrive;
    AppAutoDrive * appAutoDrive;
    TOFArray * tofArray;
    void run();

private:


};


#endif //ESPROVER_ROBOT_H
