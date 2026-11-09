// Function: Implement time synchronization and cycle control

#include "dc_manager.h"
#include "diagnostic_axis.h"
#include <cstdio>
#include <unistd.h>

extern ecx_contextt ecx_context;

bool DCManager::configureDC() {
    const int selected = diagnostic::axis();
    if (!diagnostic::canWrite(selected, ec_slavecount)) return false;
    ec_configdc();
    return setupDCSync0(selected, true, 1000000, 0);
}

bool DCManager::setupDCSync0(int slave, bool active, uint32_t cycleTime, int32_t shiftTime) {
    if (!diagnostic::canWrite(slave, ec_slavecount)) {
        printf("Invalid slave number: %d\n", slave);
        return false;
    }

    // Check if slave supports DC
    if (!ec_slave[slave].hasdc) {
        printf("Slave %d does not support DC\n", slave);
        return false;
    }

    // Configure DC SYNC0
    ecx_dcsync0(&ecx_context, slave, active, cycleTime, shiftTime);
    
    // Verify if configuration was successful
    if (!ec_slave[slave].DCactive) {
        printf("Failed to activate DC for slave %d\n", slave);
        return false;
    }

    return true;
}

void DCManager::printDCStatus() const {
    printf("\nDC Status for all slaves:\n");
    for (int i = 1; i <= ec_slavecount; i++) {
        printf("Slave %d:\n", i);
        printf("  DC Active: %d\n", ec_slave[i].DCactive);
        printf("  DC Supported: %d\n", ec_slave[i].hasdc);
        printf("  DC Cycle Time: %d ns\n", ec_slave[i].DCcycle);
        printf("  DC Shift: %d ns\n", ec_slave[i].DCshift);
    }
}