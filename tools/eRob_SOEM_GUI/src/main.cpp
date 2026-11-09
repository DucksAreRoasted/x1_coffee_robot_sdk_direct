// Standard C/C++ headers
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <cmath>
#include <iostream>

// POSIX system headers
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

// Custom components
#include "qcustomplot.h"

// EtherCAT-related headers
#include "ethercat.h"
#include "ethercat_manager.h"
#include "dc_manager.h"
#include "pdo_manager.h"
#include "diagnostic_axis.h"
#include "ethercat_thread.h"
#include "monitor_window.h"
#include "sdo_manager.h"

// Newly added header
#include "csp_motion_planning.h"

// EtherCATThread class is already defined in ethercat_thread.h

/* 
 * This program is an EtherCAT master implementation that initializes and configures EtherCAT slaves,
 * manages their states, and handles real-time data exchange. It includes functions for setting up 
 * PDO mappings, synchronizing time with the distributed clock, and controlling servomotors in 
 * various operational modes. The program also features multi-threading for real-time processing 
 * and monitoring of the EtherCAT network.
 */

// Global variables for EtherCAT communication
char IOmap[4096]; // I/O mapping for EtherCAT
int expectedWKC;  // Expected Work Counter
boolean needlf;   // Flag to indicate if a line feed is needed
volatile int wkc; // Work Counter (volatile to ensure it is updated correctly in multi-threaded context)
boolean inOP;     // Flag to indicate if the system is in operational state
uint8 currentgroup = diagnostic::kGroup; // Current group for EtherCAT communication
int dorun = 0;    // Flag to indicate if the thread should run
bool start_ecatthread_thread; // Flag to start the EtherCAT thread
int ctime_thread; // Cycle time for the EtherCAT thread

int64 toff, gl_delta; // Time offset and global delta for synchronization

// Add motor state change notification variables
volatile bool motorStateChanged = false;
volatile bool lastMotorEnabled = false;

// Function prototypes for EtherCAT thread functions
OSAL_THREAD_FUNC ecatcheck(void *ptr); // Function to check the state of EtherCAT slaves
OSAL_THREAD_FUNC_RT ecatthread(void *ptr); // Real-time EtherCAT thread function

// Thread handles for the EtherCAT threads
pthread_t thread1; // Handle for the EtherCAT check thread
pthread_t thread2; // Handle for the real-time EtherCAT thread

// Function to synchronize time with the EtherCAT distributed clock
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);
// Function to add nanoseconds to a timespec structure
void add_timespec(struct timespec *ts, int64 addtime);

// Define constants for stack size and timing
#define stack64k (64 * 1024) // Stack size for threads
#define NSEC_PER_SEC 1000000000   // Number of nanoseconds in one second
#define EC_TIMEOUTMON 5000        // Timeout for monitoring in microseconds
#define MAX_VELOCITY 30000        // Maximum velocity
#define MAX_ACCELERATION 50000    // Maximum acceleration

// Conversion units for the servomotor
float Cnt_to_deg = 0.000686645; // Conversion factor from counts to degrees

// Global variables
volatile int target_position = 0;
pthread_mutex_t target_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t target_position_cond = PTHREAD_COND_INITIALIZER;
bool target_updated = false;
int32_t received_target = 0;

// Global variables for PDO and shared data
PDOManager::RxPDO rxpdo;  // Data to be sent to slaves
PDOManager::TxPDO txpdo;  // Data received from slaves
monitor::SharedData sharedData;    // Global shared data instance

// Define planner instance in global scope
static CSPMotionPlanning& planner = CSPMotionPlanning::getInstance();
static bool motion_initialized = false;
static int32_t last_target = 0;

// Add error counter and state monitoring in global scope
static int communication_error_count = 0;
static const int MAX_ERROR_COUNT = 10;  // Maximum allowed consecutive errors
static bool need_reconnect = false;

// Function: Set the CPU affinity for a thread
void set_thread_affinity(pthread_t thread, int cpu_core) {
    cpu_set_t cpuset; // CPU set to specify which CPUs the thread can run on
    CPU_ZERO(&cpuset); // Clear the CPU set
    CPU_SET(cpu_core, &cpuset); // Add the specified CPU core to the set

    // Set the thread's CPU affinity
    int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
        printf("Unable to set CPU affinity for thread %d\n", cpu_core); // Error message if setting fails
    } else {
        printf("Thread successfully bound to CPU %d\n", cpu_core); // Confirmation message if successful
    }
}

// Function prototype for the EtherCAT test function
int erob_test();

uint16_t data_R;

// Function implementation for EtherCAT test
int erob_test() {
    // 等待操作者选轴和确认网口，不再在窗口打开十秒后永久退出。
    
    // Declare ifname outside if/else branches
    std::string ifname;
    
    // 未确认网口之前不创建 EtherCAT 连接。
    while (!sharedData.interfaceConfirmed.load()) {
        osal_usleep(100000);  // Wait 100ms
        
        // Check if stop signal received
        if (!sharedData.isRunning.load()) {
            printf("Received stop signal during interface wait\n");
            return 0;
        }
    }
    
    if (!sharedData.isRunning.load()) {
        printf("Interface confirmation cancelled - EtherCAT initialization stopped\n");
        printf("Please select a network interface in the UI to start EtherCAT communication\n");
        return 0;  // Return 0 instead of using default interface
    } else {
        // Use user selected interface
        ifname = sharedData.selectedInterface;
        printf("Using network interface: %s\n", ifname.c_str());
    }
    
    // Initialize shared data
    sharedData.isRunning.store(true);
    sharedData.motorEnabled.store(false);
    sharedData.writeIndex.store(0);
    sharedData.readIndex.store(0);
    
    printf("__________STEP 1___________________\n");
    if (!EtherCATManager::getInstance().initialize(ifname.c_str())) {
        printf("Failed to initialize EtherCAT on interface %s\n", ifname.c_str());
        return -1;
    }
    
    struct SessionCleanup {
        ~SessionCleanup() {
            diagnostic::busReady.store(false);
            sharedData.enableRequested.store(false);
            sharedData.motorEnabled.store(false);
            sharedData.modeConfirmed.store(false);
            sharedData.isRunning.store(false);
            start_ecatthread_thread = FALSE;
            inOP = FALSE;
            if (thread2 != 0) { pthread_join(thread2, nullptr); thread2 = 0; }
            if (thread1 != 0) { pthread_join(thread1, nullptr); thread1 = 0; }
            EtherCATManager::getInstance().cleanup();
        }
    } sessionCleanup;

    // Step 2: Check and set slave states
    printf("__________STEP 2___________________\n");
    if (!EtherCATManager::getInstance().checkState()) {
        printf("Failed to set slave states\n");
        return -1;
    }

    // Step 3: Map RXPDO and TXPDO
    printf("__________STEP 3___________________\n");
    if (!PDOManager::configureMapping()) {
        printf("PDO mapping failed\n");
        return -1;
    }
    
    // Step 4: Configure Distributed Clock (DC)
    printf("__________STEP 4___________________\n");
    printf("Configuring DC...\n");
    
    // Configure distributed clock
    if (!DCManager::getInstance().configureDC()) {
        printf("Failed to configure DC\n");
        return -1;
    }

    if (!EtherCATManager::getInstance().setState(EC_STATE_PRE_OP)) return -1;

    // Step 5: Transition to SAFE_OP state
    printf("__________STEP 5___________________\n");
    printf("Requesting SAFE_OP state...\n");
    
    if (!EtherCATManager::getInstance().setState(EC_STATE_SAFE_OP)) {
        printf("Failed to reach SAFE_OP state\n");
        return -1;
    }
    printf("Successfully reached SAFE_OP state\n");

    // 诊断组只含所选轴，WKC 不再把其他 14 轴算入。
    expectedWKC = ec_group[diagnostic::kGroup].outputsWKC * 2 + ec_group[diagnostic::kGroup].inputsWKC;
    const int selected = diagnostic::axis();
    if (ec_slave[selected].Obytes != sizeof(PDOManager::RxPDO) ||
        ec_slave[selected].Ibytes != sizeof(PDOManager::TxPDO) ||
        !ec_slave[selected].outputs || !ec_slave[selected].inputs) {
        printf("Selected axis PDO size mismatch, stopping\n");
        return -1;
    }
    // Read DC synchronization configuration
    DCManager::getInstance().printDCStatus();

    printf("__________STEP 6___________________\n");
    // Start the EtherCAT thread for real-time processing
    start_ecatthread_thread = TRUE; // Flag to indicate that the EtherCAT thread should start
    osal_thread_create_rt((void*)&thread1, stack64k * 2, (void *)&ecatthread, (void *)&ctime_thread); // Create the real-time EtherCAT thread
    osal_thread_create((void*)&thread2, stack64k * 2, (void *)&ecatcheck, NULL); // Create the EtherCAT check thread
    printf("___________________________________________\n");

    // Step 7: Transition to OP state
    printf("__________STEP 7___________________\n");

  // Send process data to the slaves
    EtherCATManager::getInstance().sendProcessData();
    int wkc = EtherCATManager::getInstance().receiveProcessData(EC_TIMEOUTRET);
    // Set the first slave to operational state
     if (!EtherCATManager::getInstance().setState(EC_STATE_OPERATIONAL)) {
        printf("Failed to reach OPERATIONAL state\n");
        return -1;
    }
    printf("Successfully reached OP state\n");

    // Step 8: Configure servomotor and mode operation
    printf("__________STEP 8___________________\n");

    if (ec_slave[diagnostic::axis()].state == EC_STATE_OPERATIONAL) {
        diagnostic::busReady.store(true);
        inOP = TRUE;
        printf("Selected axis operational.\n");
       
        while (sharedData.isRunning.load()) {
            // Check stop signal
            if (!sharedData.isRunning.load()) {
                printf("Received stop signal, breaking main loop\n");
                break;
            }
            osal_usleep(100000);
        }
    }

    printf("Selected axis session ended\n");
    return 0;
}

/* 
 * PI calculation to synchronize Linux time with the Distributed Clock (DC) time.
 * This function calculates the offset time needed to align the Linux time with the DC time.
 */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime) {
    static int64 integral = 0; // Integral term for PI controller
    int64 delta; // Variable to hold the difference between reference time and cycle time
    delta = (reftime) % cycletime; // Calculate the delta time
    if (delta > (cycletime / 2)) {
        delta = delta - cycletime; // Adjust delta if it's greater than half the cycle time
    }
    if (delta > 0) {
        integral++; // Increment integral if delta is positive
    }
    if (delta < 0) {
        integral--; // Decrement integral if delta is negative
    }
    *offsettime = -(delta / 100) - (integral / 20); // Calculate the offset time
    gl_delta = delta; // Update global delta variable
}

/* 
 * Add nanoseconds to a timespec structure.
 * This function updates the timespec structure by adding a specified amount of time.
 */
void add_timespec(struct timespec *ts, int64 addtime) {
    int64 sec, nsec; // Variables to hold seconds and nanoseconds

    nsec = addtime % NSEC_PER_SEC; // Calculate nanoseconds to add
    sec = (addtime - nsec) / NSEC_PER_SEC; // Calculate seconds to add
    ts->tv_sec += sec; // Update seconds in timespec
    ts->tv_nsec += nsec; // Update nanoseconds in timespec
    if (ts->tv_nsec >= NSEC_PER_SEC) { // If nanoseconds exceed 1 second
        nsec = ts->tv_nsec % NSEC_PER_SEC; // Adjust nanoseconds
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC; // Increment seconds
        ts->tv_nsec = nsec; // Set adjusted nanoseconds
    }
}

// Define state machine states
enum DriveState {
    STATE_NOT_READY     = 0,
    STATE_SWITCH_ON_DISABLED = 0x40,
    STATE_READY_TO_SWITCH_ON = 0x21,
    STATE_SWITCHED_ON   = 0x23,
    STATE_OPERATION_ENABLED = 0x27,
    STATE_FAULT         = 0x08,
    STATE_FAULT_REACTION = 0x0F,
    STATE_QUICK_STOP    = 0x07
};

/* 
 * EtherCAT check thread function
 * This function monitors the state of the EtherCAT slaves and attempts to recover 
 * any slaves that are not in the operational state.
 */
OSAL_THREAD_FUNC ecatcheck(void *ptr) {
    (void)ptr;
    while (sharedData.isRunning.load()) {
        if (diagnostic::busReady.load()) {
            ec_readstate();
            const int slave = diagnostic::axis();
            if (ec_slave[slave].state != EC_STATE_OPERATIONAL) {
                // 故障可见且停止本次诊断，不自动确认报警或重新使能。
                printf("Selected axis %d left OP: state=0x%02x AL=0x%04x; stopping\n",
                    slave, ec_slave[slave].state, ec_slave[slave].ALstatuscode);
                diagnostic::busReady.store(false);
                sharedData.enableRequested.store(false);
                sharedData.isRunning.store(false);
            }
        }
        osal_usleep(10000);
    }
    return;
}

/* 
 * RT EtherCAT thread function
 * This function handles the real-time processing of EtherCAT data. 
 * It sends and receives process data in a loop, synchronizing with the 
 * distributed clock if available, and ensuring timely execution based on 
 * the specified cycle time.
 */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr) {
    // Set thread priority and CPU affinity
    struct sched_param param;
    param.sched_priority = 98;  // High real-time priority
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param)) {
        printf("Warning: Failed to set RT priority for EtherCAT thread\n");
    }
    
    // Set CPU affinity
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(3, &cpu_set);  // Bind to CPU 3
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set), &cpu_set)) {
        printf("Warning: Failed to set CPU affinity for EtherCAT thread\n");
    }
    
    // Lock memory
    if (mlockall(MCL_CURRENT | MCL_FUTURE)) {
        printf("Warning: Failed to lock memory for EtherCAT thread\n");
    }

    printf("EtherCAT real-time thread started\n");

    struct timespec ts, tleft;
    int64 cycletime = *(int *)ptr * 1000;  // Convert to nanoseconds
    
    // Initialize time
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_nsec = (ts.tv_nsec / cycletime + 1) * cycletime;
    if (ts.tv_nsec >= NSEC_PER_SEC) {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }

    toff = 0;
    dorun = 0;
    
    // Initialize PDO data
    rxpdo.controlword = 0x0000;  // 初始不自动清报警。
    rxpdo.target_velocity = 0;
    rxpdo.mode_of_operation = 9;  // CSV mode (9)
    rxpdo.target_position = txpdo.actual_position;
    rxpdo.target_torque = 0;
    rxpdo.padding = 0;

    // Send initial data
    PDOManager initialPdo;
    if (!initialPdo.writeProcessData(rxpdo)) {
        sharedData.isRunning.store(false);
        return;
    }
    ec_send_processdata_group(diagnostic::kGroup);
    wkc = ec_receive_processdata_group(diagnostic::kGroup, EC_TIMEOUTRET);  // Ensure first communication succeeds

    int step = 0;
    int retry_count = 0;
    const int MAX_RETRY = 3;

    while (sharedData.isRunning.load()) {
        // Wait for the next cycle
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        if (!sharedData.isRunning.load()) break;
        
        // Calculate the next cycle time
        ts.tv_nsec += cycletime;
        if (ts.tv_nsec >= NSEC_PER_SEC) {
            ts.tv_sec++;
            ts.tv_nsec -= NSEC_PER_SEC;
        }

        dorun++;

        if (start_ecatthread_thread) {
            // Receive process data
            wkc = ec_receive_processdata_group(diagnostic::kGroup, EC_TIMEOUTRET);

            if (wkc >= expectedWKC) {
                retry_count = 0;
                
                if (!initialPdo.readProcessData(txpdo)) {
                    diagnostic::busReady.store(false);
                    sharedData.isRunning.store(false);
                    break;
                }

                if (!sharedData.motorEnabled.load()) {
                    rxpdo.target_position = txpdo.actual_position;
                    sharedData.targetPosition.store(txpdo.actual_position);
                    rxpdo.target_velocity = 0;
                    rxpdo.target_torque = 0;
                }
                // Get current status
                sharedData.statusWord.store(txpdo.statusword);
                if (txpdo.statusword & 0x0008) {
                    // 已使能过程中发生故障也撤销许可，不自动清除故障。
                    sharedData.enableRequested.store(false);
                    sharedData.motorEnabled.store(false);
                    sharedData.modeConfirmed.store(false);
                }
                uint16_t status = txpdo.statusword & 0x6F;  // Mask non-status bits
                
                // Handle mode switch request (only in non-enabled state)
                if (sharedData.modeChangeRequested.load() && !sharedData.motorEnabled.load()) {
                    static int mode_change_timeout = 0;
                    // Immediately update PDO operation mode
                    rxpdo.mode_of_operation = sharedData.operationMode.load();
                    
                    // Wait for slave to confirm mode switch
                    if (txpdo.mode_of_operation_display == rxpdo.mode_of_operation) {
                        sharedData.modeChangeRequested.store(false);
                        sharedData.modeConfirmed.store(true);
                        mode_change_timeout = 0;
                        printf("Operation mode changed and confirmed to %d\n", rxpdo.mode_of_operation);
                    } else {
                        // Add timeout detection
                        if (++mode_change_timeout > 1000) { // About 5-second timeout (assuming 5ms cycle)
                            printf("Mode change timeout! Requested: %d, Current: %d\n", 
                                   rxpdo.mode_of_operation, txpdo.mode_of_operation_display);
                            sharedData.modeChangeRequested.store(false);
                            mode_change_timeout = 0;
                        }
                    }
                    
                    // Maintain safe state during mode switch
                    rxpdo.target_velocity = 0;
                    rxpdo.target_position = txpdo.actual_position;
                    rxpdo.target_torque = 0;
                    rxpdo.controlword = 0x0006;  // Keep Ready To Switch On state
                }
                // Enable sequence (only after mode confirmation)
                else if (sharedData.enableRequested.load() && !sharedData.motorEnabled.load()) {
                    if (!sharedData.modeConfirmed.load()) {
                        // Ignore enable request if mode is not confirmed
                        sharedData.enableRequested.store(false);
                        printf("Cannot enable: Operation mode not confirmed\n");
                    } else {
                        // Normal enable sequence
                        switch(status) {
                            case STATE_FAULT:
                                // 故障保持可见，撤销使能许可，不自动清报警。
                                rxpdo.controlword = 0;
                                sharedData.enableRequested.store(false);
                                sharedData.modeConfirmed.store(false);
                                printf("Selected axis fault, enable rejected; no reset\n");
                                break;
                                
                            case STATE_SWITCH_ON_DISABLED:
                                rxpdo.controlword = 0x0006;  // Shutdown command
                                printf("Switch on disabled, sending shutdown command\n");
                                break;
                                
                            case STATE_READY_TO_SWITCH_ON:
                                rxpdo.controlword = 0x0007;  // Switch on command
                                printf("Ready to switch on, sending switch on command\n");
                                break;
                                
                            case STATE_SWITCHED_ON:
                                rxpdo.controlword = 0x000F;  // Enable operation command
                                printf("Switched on, sending enable operation command\n");
                                break;
                                
                            case STATE_OPERATION_ENABLED:
                                // 使能瞬间以实际位置为起点，不追逐旧输入框中的目标。
                                rxpdo.target_position = txpdo.actual_position;
                                sharedData.targetPosition.store(txpdo.actual_position);
                                sharedData.motorEnabled.store(true);
                                printf("Operation enabled successfully\n");
                                // Detect motor state change, notify UI
                                if (!lastMotorEnabled) {
                                    lastMotorEnabled = true;
                                    motorStateChanged = true;
                                }
                                break;
                                
                            default:
                                rxpdo.controlword = 0x0006;  // Try shutdown command
                                printf("Unknown state (0x%04x), trying shutdown\n", status);
                                break;
                        }
                    }
                }
                // Disable sequence
                else if (!sharedData.enableRequested.load()) {
                     switch(status) {
                        case STATE_OPERATION_ENABLED:
                            rxpdo.controlword = 0x0007;  // Switch to Switched On state
                            printf("Disabling operation, switching to Switched On state\n");
                            break;
                            
                        case STATE_SWITCHED_ON:
                            rxpdo.controlword = 0x0006;  // Switch to Ready To Switch On state
                            printf("Switching to Ready To Switch On state\n");
                            break;
                            
                        case STATE_READY_TO_SWITCH_ON:
                            rxpdo.controlword = 0x0000;  // Switch to Switch On Disabled state
                            sharedData.motorEnabled.store(false);  // Confirm motor disabled
                            printf("Motor disabled successfully\n");
                            // Detect motor state change, notify UI
                            if (lastMotorEnabled) {
                                lastMotorEnabled = false;
                                motorStateChanged = true;
                            }
                            break;
                            
                        default:
                            rxpdo.controlword = 0x0000;  // Force Switch On Disabled
                            sharedData.motorEnabled.store(false);
                            printf("Unknown state while disabling (0x%04x), forcing disable\n", status);
                            printf("ecatthread running, isRunning=%d\n", sharedData.isRunning.load());
                            // Detect motor state change, notify UI
                            if (lastMotorEnabled) {
                                lastMotorEnabled = false;
                                motorStateChanged = true;
                            }
                            break;
                    }
                }
                // Normal operation state
                else if (sharedData.motorEnabled.load()) {
                    rxpdo.controlword = 0x000F;
                    
                    // Set target values based on current mode
                    switch(rxpdo.mode_of_operation) {
                        case 1:  // PP mode
                        {
                            static bool newPositionSet = false;
                            static int32_t lastPosition = 0;
                            int32_t newPosition = sharedData.targetPosition.load();

                            if (newPosition != lastPosition) {
                                rxpdo.target_position = newPosition;
                                lastPosition = newPosition;
                                newPositionSet = true;
                                printf("New PP mode target position: %d\n", newPosition);

                                // Set control word to start new position motion
                                rxpdo.controlword = 0x001F;  // bit4=1 indicates new position
                            } else if (newPositionSet) {
                                // Check if target position is reached
                                if (txpdo.statusword & (1 << 10)) {  // bit10=1 indicates target reached
                                    newPositionSet = false;
                                    rxpdo.controlword = 0x000F;  // Clear new position flag
                                    printf("Target position reached\n");
                                }
                            }
                            break;
                        }
                        case 3:  // PV mode
                            // Directly set target velocity, no additional processing needed
                            rxpdo.target_velocity = sharedData.targetVelocity.load();
                            break;
                        case 4:  // PT mode
                            rxpdo.target_position = 0;  // Clear position
                            rxpdo.target_velocity = 0;  // Clear velocity
                            rxpdo.target_torque = sharedData.targetTorque.load();  // Set target torque
                            break;
                        case 8:  // CSP mode
                            if (sharedData.motorEnabled.load()) {
                                int32_t current_target = sharedData.targetPosition.load();
                                
                                // Initialize planner only when target position changes or on first run
                                if (!motion_initialized || current_target != last_target) {
                                    CSPMotionPlanning::MotionParams params;
                                    params.target_position = current_target;
                                    params.max_velocity = sharedData.cspMaxVelocity;
                                    params.acceleration = 10000;
                                    params.deceleration = 10000;
                                    params.current_position = txpdo.actual_position;
                                    params.current_velocity = txpdo.actual_velocity;
                                    
                                    planner.init(params);
                                    motion_initialized = true;
                                    last_target = current_target;
                                   // printf("Motion planner initialized with new target: %d\n", current_target);
                                } 
                                
                                // Calculate next position point
                                auto state = planner.calculateNextState(500);  // 500us cycle
                                
                                // Update target position
                                rxpdo.target_position = state.position;
                                
                                // Check if motion is completed
                                if (state.is_completed) {
                                    motion_initialized = false;
                                   // printf("Motion completed at position: %d\n", state.position);
                                }
                            } else {
                                //printf("Motion not initialized\n");
                                rxpdo.target_position = txpdo.actual_position;
                            }
                            break;
                        case 9:  // CSV mode
                            rxpdo.target_position = 0;
                            rxpdo.target_velocity = sharedData.targetVelocity.load();
                            rxpdo.target_torque = 0;
                            break;
                        case 10: // CST mode
                            rxpdo.target_position = 0;
                            rxpdo.target_velocity = 0;
                            rxpdo.target_torque = sharedData.targetTorque.load();
                            break;
                    }
                }
                
                // Update shared data for UI display
                int writeIdx = sharedData.writeIndex.load();
                monitor::SharedData::DataPoint& point = sharedData.buffer[writeIdx % monitor::SharedData::BUFFER_SIZE];
                point.time = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0;
                point.velocity = txpdo.actual_velocity;
                point.position = txpdo.actual_position;
                point.torque = txpdo.actual_torque;
                point.mode = txpdo.mode_of_operation_display;
                sharedData.writeIndex.store((writeIdx + 1) % monitor::SharedData::BUFFER_SIZE);
                
                // Add detailed state monitoring
                if (dorun % 5000 == 0) {
                    printf("Status: 0x%04x, Control Word: 0x%04x, Enable Requested: %d, Motor Enabled: %d\n",
                           status, rxpdo.controlword, sharedData.enableRequested.load(), sharedData.motorEnabled.load());
                    printf("State bits: %s%s%s%s%s%s%s%s\n",
                           (txpdo.statusword & 0x0001) ? "Ready to switch on " : "",
                           (txpdo.statusword & 0x0002) ? "Switched on " : "",
                           (txpdo.statusword & 0x0004) ? "Operation enabled " : "",
                           (txpdo.statusword & 0x0008) ? "Fault " : "",
                           (txpdo.statusword & 0x0010) ? "Voltage enabled " : "",
                           (txpdo.statusword & 0x0020) ? "Quick stop " : "",
                           (txpdo.statusword & 0x0040) ? "Switch on disabled " : "",
                           (txpdo.statusword & 0x0080) ? "Warning " : "");
                }
                
                // Send data to slaves
                if (!initialPdo.writeProcessData(rxpdo)) {
                    diagnostic::busReady.store(false);
                    sharedData.isRunning.store(false);
                    break;
                }

                // Send process data
                ec_send_processdata_group(diagnostic::kGroup);

            } else if (diagnostic::busReady.load()) {
                // SAFE-OP 进入 OP 之前不把尚未生效的输出 WKC 误判为断线。
                retry_count++;
                if (retry_count >= MAX_RETRY) {
                    printf("ERROR: Communication failure after %d retries\n", retry_count);
                    // 通信失败立即撤销操作许可，禁止恢复后追逐旧目标。
                    diagnostic::busReady.store(false);
                    sharedData.enableRequested.store(false);
                    sharedData.motorEnabled.store(false);
                    sharedData.isRunning.store(false);
                }
            }

            // Clock synchronization
            if (ec_slave[diagnostic::axis()].hasdc) {
                ec_sync(ec_DCtime, cycletime, &toff);
            }

            // Send process data
            ec_send_processdata_group(diagnostic::kGroup);
        }

        // Monitor cycle time
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ts.tv_nsec += cycletime;
        if (ts.tv_nsec >= NSEC_PER_SEC) {
            ts.tv_sec++;
            ts.tv_nsec -= NSEC_PER_SEC;
        }
        if (!sharedData.isRunning.load()) break;
    }
    
    printf("EtherCAT real-time thread exiting\n");
    return;
}

// Main function implementation
#ifndef X1_GUI_NO_MAIN
int main(int argc, char **argv) {
    QApplication app(argc, argv);
    
    needlf = FALSE;
    inOP = FALSE;
    start_ecatthread_thread = FALSE;
    dorun = 0;
    ctime_thread = 1000;

    // Set UI thread affinity - ensure UI runs on separate CPU core
    cpu_set_t ui_cpuset;
    CPU_ZERO(&ui_cpuset);
    CPU_SET(1, &ui_cpuset);  // UI uses CPU core 1
    
    if (sched_setaffinity(0, sizeof(cpu_set_t), &ui_cpuset) == -1) {
        perror("UI thread affinity");
    }

    // Set UI thread priority
    struct sched_param ui_param;
    ui_param.sched_priority = 50;  // Lower real-time priority
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &ui_param);
    
    printf("Running UI on CPU core 1\n");
    
    // Create UI window
    MonitorWindow* window = new MonitorWindow(sharedData);
    window->show();
    
    // Connect signals before startup
    QObject::connect(window, &MonitorWindow::windowClosed, [&app]() {
        // Operations to perform after UI closes
        printf("UI window closed, shutting down EtherCAT...\n");
        sharedData.isRunning.store(false);  // Notify EtherCAT thread to stop
        dorun = 0;  // Set global stop flag
        
        // Exit application - let main function handle thread cleanup
        app.quit();
    });
    
    // Run Qt event loop
    int result = app.exec();
    
    // Cleanup after UI event loop ends
    printf("UI event loop ended, cleaning up...\n");
    
    // Ensure EtherCAT threads stop
    sharedData.isRunning.store(false);
    dorun = 0;
    
    // EtherCAT 会话守卫负责回收底层线程，窗口析构等待其结束。
    delete window;

    printf("Program exited cleanly\n");
    return result;
}
#endif
