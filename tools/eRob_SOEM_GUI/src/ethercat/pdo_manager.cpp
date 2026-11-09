#include "pdo_manager.h"
#include "diagnostic_axis.h"

#include <cstdio>
#include <cstring>  // Header for memcpy

// Only define static IOmap
char PDOManager::IOmap[4096];

PDOManager::PDOManager() {
    // Initialize member variables
    memset(&rxpdo, 0, sizeof(RxPDO));
    memset(&txpdo, 0, sizeof(TxPDO));
}

bool PDOManager::configureRxPDO(int slave) {
    if (!diagnostic::canWrite(slave, ec_slavecount)) return false;
    printf("Configuring RxPDO for slave %d...\n", slave);
    uint16_t map_1c12;
    uint8_t zero_map = 0;
    uint32_t map_object;
    uint8_t clear_val = 0;  // 子索引 0 的计数是 8 位，与 SDK 一致。

    // Clear existing mapping
    if (ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(zero_map), &zero_map, EC_TIMEOUTSAFE) <= 0) return false;
    
    // Control word (0x6040:0, 16 bits)
    map_object = 0x60400010;
    if (ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE) <= 0) return false;
    
    // Target position (0x607A:0, 32 bits)
    map_object = 0x607A0020;
    if (ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE) <= 0) return false;
    
    // Target velocity (0x60FF:0, 32 bits)
    map_object = 0x60FF0020;
    if (ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE) <= 0) return false;
    
    // Target torque (0x6071:0, 16 bits)
    map_object = 0x60710010;
    if (ec_SDOwrite(slave, 0x1600, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE) <= 0) return false;

    // Operation mode (0x6060:0, 8 bits)
    map_object = 0x60600008;
    if (ec_SDOwrite(slave, 0x1600, 0x05, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE) <= 0) return false;
    
    // Padding bits (8 bits padding)
    map_object = 0x00000008;
    if (ec_SDOwrite(slave, 0x1600, 0x06, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE) <= 0) return false;
    
    // Set number of mapping objects
    uint8_t map_count = 6;  // Includes all objects
    if (ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE) <= 0) return false;
    
    // Configure RXPDO assignment
    clear_val = 0x0000;
    if (ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE) <= 0) return false;
    map_1c12 = 0x1600;
    if (ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE) <= 0) return false;
    uint8_t assigned_count = 1;
    if (ec_SDOwrite(slave, 0x1C12, 0x00, FALSE, sizeof(assigned_count), &assigned_count, EC_TIMEOUTSAFE) <= 0) return false;

    printf("RxPDO mapping completed for slave %d\n", slave);
    return true;
}

bool PDOManager::configureTxPDO(int slave) {
    if (!diagnostic::canWrite(slave, ec_slavecount)) return false;
    printf("Configuring TxPDO for slave %d...\n", slave);
    uint16_t map_1c13;
    uint32_t map_object;
    uint8_t clear_val = 0;  // 子索引 0 的计数是 8 位，与 SDK 一致。

    // Clear existing mapping
    if (ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE) <= 0) return false;

    // Status word (0x6041:0, 16 bits)
    map_object = 0x60410010;
    if (ec_SDOwrite(slave, 0x1A00, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE) <= 0) return false;

    // Actual position (0x6064:0, 32 bits)
    map_object = 0x60640020;
    if (ec_SDOwrite(slave, 0x1A00, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE) <= 0) return false;

    // Actual velocity (0x606C:0, 32 bits)
    map_object = 0x606C0020;
    if (ec_SDOwrite(slave, 0x1A00, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE) <= 0) return false;

    // Actual torque (0x6077:0, 16 bits)
    map_object = 0x60770010;
    if (ec_SDOwrite(slave, 0x1A00, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE) <= 0) return false;

    // Mode display (0x6061:0, 8 bits)
    map_object = 0x60610008;
    if (ec_SDOwrite(slave, 0x1A00, 0x05, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE) <= 0) return false;

    // Padding (8 bits padding)
    map_object = 0x00000008;
    if (ec_SDOwrite(slave, 0x1A00, 0x06, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE) <= 0) return false;

    // Set number of mapping objects
    uint8_t map_count = 6;  // Updated to 6 objects (including padding)
    if (ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE) <= 0) return false;

    // Configure TXPDO assignment
    clear_val = 0x0000;
    if (ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE) <= 0) return false;
    map_1c13 = 0x1A00;
    if (ec_SDOwrite(slave, 0x1C13, 0x01, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE) <= 0) return false;
    uint8_t assigned_count = 1;
    if (ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(assigned_count), &assigned_count, EC_TIMEOUTSAFE) <= 0) return false;

    printf("TxPDO mapping completed for slave %d\n", slave);
    return true;
}

bool PDOManager::configureMapping() {
    printf("Configuring PDO mapping...\n");
    
    // 只映射所选轴；其他从站属于未映射的组 0，并保持 PRE-OP。
    const int selected = diagnostic::axis();
    if (!diagnostic::canWrite(selected, ec_slavecount)) return false;
    for (int i = 1; i <= ec_slavecount; ++i) ec_slave[i].group = (i == selected ? diagnostic::kGroup : 0);
    if (!configureRxPDO(selected) || !configureTxPDO(selected)) return false;

    // Configure IOmap
    if (ec_config_map_group(&IOmap, diagnostic::kGroup) <= 0) return false;
    // Give slaves some time to process PDO configuration
    osal_usleep(100000);  // 100ms
    printf("PDO mapping completed successfully\n");
    return true;
}

bool PDOManager::initializePDO() {
    // 初始只下发禁止使能的控制字，不发送故障复位或新目标。
    RxPDO initial{};
    return writeProcessData(initial);
}

bool PDOManager::readProcessData(TxPDO& out_txpdo) {
    const int slave = diagnostic::axis();
    if (!diagnostic::canWrite(slave, ec_slavecount) || !ec_slave[slave].inputs ||
        ec_slave[slave].Ibytes != sizeof(TxPDO)) return false;
    std::memcpy(&out_txpdo, ec_slave[slave].inputs, sizeof(TxPDO));
    return true;
}

bool PDOManager::writeProcessData(const RxPDO& in_rxpdo) {
    const int slave = diagnostic::axis();
    if (!diagnostic::canWrite(slave, ec_slavecount) || !ec_slave[slave].outputs ||
        ec_slave[slave].Obytes != sizeof(RxPDO)) return false;
    std::memcpy(ec_slave[slave].outputs, &in_rxpdo, sizeof(RxPDO));
    return true;
}

bool PDOManager::configurePDOs(int slave) {
    // 保留兼容入口，复用逐项检查且只允许所选轴的映射过程。
    return configureRxPDO(slave) && configureTxPDO(slave);
}
