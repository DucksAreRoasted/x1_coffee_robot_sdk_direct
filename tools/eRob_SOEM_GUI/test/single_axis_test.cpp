// 用替身 SOEM 验证真实路由函数，不打开网口、不连接硬件。
#include "diagnostic_axis.h"
#include "pdo_manager.h"
#include "sdo_manager.h"
#include "ethercat_manager.h"
#include "dc_manager.h"
#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

#define CHECK(expr) do { if (!(expr)) throw std::runtime_error(std::string(#expr) + " line " + std::to_string(__LINE__)); } while (0)
struct MailboxWrite { int slave, index, subindex, size; };
std::vector<MailboxWrite> writes;
std::vector<int> stateWrites, syncWrites;
int failedWrite = -1, mapCalls = 0, closes = 0;
std::array<std::array<uint8, 32>, 16> outputs{}, inputs{};

extern "C" {
ec_slavet ec_slave[EC_MAXSLAVE]{};
int ec_slavecount = 15;
ec_groupt ec_group[EC_MAXGROUP]{};
ecx_contextt ecx_context{};
int ec_init(const char*) { return 1; }
int ec_config_init(uint8) { return ec_slavecount; }
void ec_close() { ++closes; }
int ec_readstate() { return 1; }
int ec_writestate(uint16 slave) { stateWrites.push_back(slave); return 1; }
int ec_send_processdata_group(uint8 group) { CHECK(group == diagnostic::kGroup); return 1; }
int ec_receive_processdata_group(uint8 group, int) { CHECK(group == diagnostic::kGroup); return 3; }
char* ec_ALstatuscode2string(uint16) { static char text[] = "fake"; return text; }
int ec_SDOread(uint16, uint16, uint8, boolean, int*, void*, int) { return 0; }
int ec_SDOwrite(uint16 slave, uint16 index, uint8 subindex, boolean, int size, const void*, int) {
    writes.push_back({slave, index, subindex, size});
    // 所有计数子索引 0 必须是一字节，拒绝上游旧的两字节写法。
    if ((index == 0x1600 || index == 0x1A00 || index == 0x1C12 || index == 0x1C13) && subindex == 0)
        CHECK(size == 1);
    return static_cast<int>(writes.size()) == failedWrite ? 0 : 1;
}
int ec_config_map_group(void*, uint8 group) {
    CHECK(group == diagnostic::kGroup && group != 0);
    ++mapCalls;
    // 模拟 SOEM 的非零组语义，只映射组内轴；组 0 是广播，禁止使用。
    for (int i = 1; i <= ec_slavecount; ++i) {
        if (ec_slave[i].group == group) {
            ec_slave[i].outputs = outputs[i].data(); ec_slave[i].inputs = inputs[i].data();
            ec_slave[i].Obytes = sizeof(PDOManager::RxPDO); ec_slave[i].Ibytes = sizeof(PDOManager::TxPDO);
            ec_slave[i].state = EC_STATE_SAFE_OP;
        }
    }
    return sizeof(PDOManager::RxPDO) + sizeof(PDOManager::TxPDO);
}
boolean ec_configdc() { return TRUE; }
void ecx_dcsync0(ecx_contextt*, uint16 slave, boolean active, uint32, int32) {
    syncWrites.push_back(slave); ec_slave[slave].DCactive = active;
}
int osal_usleep(uint32) { return 0; }
}

void reset() {
    EtherCATManager::getInstance().cleanup();
    diagnostic::stopRequested.store(false);
    diagnostic::busReady.store(false); diagnostic::selectionLocked.store(false); diagnostic::selectedAxis.store(0);
    ec_slavecount = 15;
    std::memset(ec_slave, 0, sizeof(ec_slave));
    for (int i = 1; i <= 15; ++i) {
        ec_slave[i].state = EC_STATE_PRE_OP; ec_slave[i].hasdc = TRUE;
        outputs[i].fill(0xA5); inputs[i].fill(0);
    }
    writes.clear(); stateWrites.clear(); syncWrites.clear(); failedWrite = -1; mapCalls = 0;
}

int main() {
    try {
        reset();
        CHECK(!diagnostic::lockSelection());
        CHECK(!diagnostic::chooseAxis(-1)); CHECK(!diagnostic::chooseAxis(16));
        CHECK(diagnostic::chooseAxis(5)); CHECK(diagnostic::chooseAxis(0));
        CHECK(!diagnostic::lockSelection()); CHECK(!PDOManager::configureMapping()); CHECK(writes.empty());
        CHECK(!diagnostic::chooseAxis(16));
        for (int selected = 1; selected <= 15; ++selected) {
            reset();
            CHECK(diagnostic::chooseAxis(selected)); CHECK(diagnostic::lockSelection());
            CHECK(!diagnostic::chooseAxis(selected == 15 ? 1 : selected + 1));
            auto& manager = EtherCATManager::getInstance();
            CHECK(manager.initialize("fake")); CHECK(manager.checkState());
            CHECK(PDOManager::configureMapping()); CHECK(mapCalls == 1);
            CHECK(DCManager::getInstance().configureDC());
            CHECK(syncWrites == std::vector<int>{selected});
            CHECK(manager.setState(EC_STATE_OPERATIONAL));
            CHECK(manager.sendProcessData()); CHECK(manager.receiveProcessData(1) == 3);
            PDOManager pdo;
            PDOManager::RxPDO command{}; command.controlword = 0x000F; command.target_position = 123456;
            CHECK(pdo.writeProcessData(command));
            CHECK(std::memcmp(outputs[selected].data(), &command, sizeof(command)) == 0);
            PDOManager::TxPDO feedback{}; feedback.actual_position = 10000 + selected; feedback.actual_velocity = selected;
            std::memcpy(inputs[selected].data(), &feedback, sizeof(feedback));
            PDOManager::TxPDO received{}; CHECK(pdo.readProcessData(received));
            CHECK(received.actual_position == 10000 + selected && received.actual_velocity == selected);
            auto& sdo = SDOManager::getInstance();
            CHECK(!sdo.setOperationMode(selected, 8)); // 总线许可未发布时不可写参数。
            diagnostic::busReady.store(true);
            CHECK(sdo.setOperationMode(selected, 8));
            CHECK(sdo.setPPModeParams(selected, {100, 200, 300}));
            CHECK(sdo.setPVModeParams(selected, {200, 300}));
            CHECK(sdo.setPTModeParams(selected, {100, 200}));
            CHECK(sdo.setCSPModeParams(selected, {1000, 100}));
            CHECK(sdo.setCSVModeParams(selected, {100, 200, 300}));
            CHECK(sdo.setCSTParams({100, 200}));
            const auto written = writes.size();
            CHECK(!sdo.setOperationMode(0, 8)); CHECK(!sdo.setOperationMode(16, 8));
            CHECK(!sdo.setOperationMode(selected == 15 ? 1 : selected + 1, 8)); CHECK(writes.size() == written);
            CHECK(!PDOManager::configureRxPDO(selected == 15 ? 1 : selected + 1));
            for (int i = 1; i <= 15; ++i) {
                if (i == selected) continue;
                CHECK(ec_slave[i].state == EC_STATE_PRE_OP && ec_slave[i].group != diagnostic::kGroup);
                for (auto byte : outputs[i]) CHECK(byte == 0xA5);
            }
            for (const auto& write : writes) CHECK(write.slave == selected);
            for (auto slave : stateWrites) CHECK(slave == selected);
            ec_slave[selected].Obytes = 1; CHECK(!pdo.writeProcessData(command));
            ec_slave[selected].Ibytes = 1; CHECK(!pdo.readProcessData(received));
            ec_slave[selected].outputs = nullptr; CHECK(!pdo.writeProcessData(command));
            ec_slave[selected].inputs = nullptr; CHECK(!pdo.readProcessData(received));
            manager.cleanup(); CHECK(!diagnostic::busReady.load());
            CHECK(stateWrites.back() == selected);
        }
        // 任一映射写入失败都立即停止，不继续后续写入或映射。
        for (int failure = 1; failure <= 22; ++failure) {
            reset(); CHECK(diagnostic::chooseAxis(7)); CHECK(diagnostic::lockSelection());
            failedWrite = failure;
            CHECK(!PDOManager::configureMapping()); CHECK(mapCalls == 0);
            CHECK(static_cast<int>(writes.size()) == failure);
        }
        for (int count : {14, 16}) {
            reset(); CHECK(diagnostic::chooseAxis(1)); CHECK(diagnostic::lockSelection());
            ec_slavecount = count; const int before = closes;
            CHECK(!EtherCATManager::getInstance().initialize("fake")); CHECK(closes == before + 1);
            CHECK(!PDOManager::configureMapping()); CHECK(writes.empty());
        }
        std::cout << "PASS: 15 axes isolated; PDO/SDO/DC/state routing, invalid selection and mapping failures checked\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
