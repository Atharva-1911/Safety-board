#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include "ModbusServerRTU.h"
#include "RTUutils.h"
#include "ModbusTypeDefs.h"

#define DEBUG

// method prototypes
void setup(void);
void loop(void);
void pcf8575_writeAll(uint16_t);
void engage_relay_state(uint16_t);
void disengage_relay_state(uint16_t);
void safety_monitor();
void safety_setup();

// atomic prototypes
void rs485_loopback();

// ISR handlers
void IRAM_ATTR estop_isr();
void IRAM_ATTR handle_limit_isr(uint8_t index);
void IRAM_ATTR limit_switch_0_isr();
void IRAM_ATTR limit_switch_1_isr();
void IRAM_ATTR limit_switch_2_isr();
void IRAM_ATTR limit_switch_3_isr();
void IRAM_ATTR limit_switch_4_isr();
void IRAM_ATTR limit_switch_5_isr();

// Safety monitoring functions
uint16_t check_estop_state();
uint16_t check_limit_switches();

// Modbus handler functions
ModbusMessage FC02_handler(ModbusMessage request);
ModbusMessage FC05_handler(ModbusMessage request);

// System constants
#define DEBOUNCE_TIME_MS 50 // Debounce time in milliseconds
#define NUM_LIMIT_SWITCHES 6

// pin configuration
#define ESTOP_PIN 13
#define STATUS_LED 2
#define I2C_SDA 21
#define I2C_SCL 22
#define LIMIT_SWITCH_0 14

// Modbus RS-485 pins
#define MODBUS_DE_RE 4   // DE and RE tied together (TX enable)
#define MODBUS_RX 16     // UART2 RX
#define MODBUS_TX 17     // UART2 TX

const uint8_t LIMIT_SWITCH_PINS[NUM_LIMIT_SWITCHES] = {14, 27, 26, 25, 33, 32};


// system configuration
#define PCF8575_ADDR 0x20

// Modbus configuration
#define MODBUS_SLAVE_ID 1
#define MODBUS_BAUDRATE 115200

// Modbus register addresses - Discrete Inputs (FC02) - ESP32 → PC
#define MODBUS_DI_ESTOP 0
#define MODBUS_DI_LIMIT_0 1
#define MODBUS_DI_BRAKE_J0 7
#define MODBUS_DI_BRAKE_J1 8
#define MODBUS_DI_BRAKE_J2 9
#define MODBUS_DI_DRIVER_J0 10
#define MODBUS_DI_DRIVER_J1 11
#define MODBUS_DI_DRIVER_J2 12
#define MODBUS_DI_DRIVER_WRIST 13
#define MODBUS_DI_GRIPPER 14

// Modbus register addresses - Coils (FC05) - PC → ESP32
#define MODBUS_COIL_GRIPPER 0
#define MODBUS_COIL_BRAKE_J0 1
#define MODBUS_COIL_BRAKE_J1 2
#define MODBUS_COIL_BRAKE_J2 3

// convinience constants: Relay masks
#define J0_BRAKE_RELAY 0xFFF7
#define J1_BRAKE_RELAY 0xEFFF
#define J2_BRAKE_RELAY 0xFFFB

#define J0_DRIVER_RELAY 0xDFFF 
#define J1_DRIVER_RELAY 0xFFFD
#define J2_DRIVER_RELAY 0xBFFF
#define WRIST_DRIVER_RELAY 0xFFFE

#define GRIPPER_RELAY 0x7FFF

// Limit switch configuration

// Safety event relay masks
#define ESTOP_RELAY_STATE (J0_BRAKE_RELAY & \
                            J1_BRAKE_RELAY & \
                            J2_BRAKE_RELAY & \
                            J0_DRIVER_RELAY & \
                            J1_DRIVER_RELAY & \
                            J2_DRIVER_RELAY & \
                            WRIST_DRIVER_RELAY)
// Gripper relay stays HIGH (released) to prevent dropping held objects

const uint16_t LIMIT_SWITCH_RELAY_MASKS[NUM_LIMIT_SWITCHES] = {
    J0_BRAKE_RELAY & J0_DRIVER_RELAY,    // Limit 0 → J0 brake + driver
    J1_BRAKE_RELAY & J1_DRIVER_RELAY,    // Limit 1 → J1 brake + driver
    J2_BRAKE_RELAY & J2_DRIVER_RELAY,    // Limit 2 → J2 brake + driver
    WRIST_DRIVER_RELAY,                  // Limit 3 → Wrist driver (no brake, light duty)
    WRIST_DRIVER_RELAY,                  // Limit 4 → Wrist driver (no brake, light duty)
    WRIST_DRIVER_RELAY                   // Limit 5 → Wrist driver (no brake, light duty)
};

//global variables

// ISR flags (volatile, set by ISRs)
volatile bool estop_flag_changed = false;
volatile unsigned long estop_trigger_time = 0;
volatile bool limit_flag_changed[NUM_LIMIT_SWITCHES] = {false};
volatile unsigned long limit_trigger_time[NUM_LIMIT_SWITCHES] = {0};

// FreeRTOS task handles
TaskHandle_t estop_debounce_task_handle = NULL;
TaskHandle_t limit_debounce_task_handles[NUM_LIMIT_SWITCHES] = {NULL};

// Spinlocks for thread-safe access to state variables (shared between debounce tasks and main loop)
portMUX_TYPE estop_state_spinlock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE limit_state_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Stable state (updated by debounce tasks, read by main loop)
bool estop_stable_state = false;      // true = E-Stop pressed, false = released (Pin: LOW = pressed, HIGH = released)
uint8_t limit_stable_state = 0x00;    // Bitmap: 1 = limit hit/fault, 0 = clear

// Edge detection state
bool last_estop_stable_state = false;
uint8_t last_limit_stable_state = 0x00;
uint16_t last_relay_state = 0xFFFF;   // Start with all relays OFF

// Modbus globals
ModbusServerRTU MBserver(Serial2, MODBUS_DE_RE);
portMUX_TYPE modbus_cmd_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Modbus command state (set by FC05 handler, read by main loop)
bool modbus_gripper_cmd = false;        // Gripper engage command
bool modbus_brake_release_j0 = false;   // J0 brake release command
bool modbus_brake_release_j1 = false;   // J1 brake release command
bool modbus_brake_release_j2 = false;   // J2 brake release command

// PCF8575 Port Manipulation Functions
void pcf8575_writeAll(uint16_t value) {
    Wire.beginTransmission(PCF8575_ADDR);
    Wire.write((uint8_t)(value & 0xFF));         // Low byte (P0-P7)
    Wire.write((uint8_t)((value >> 8) & 0xFF));  // High byte (P8-P15)
    Wire.endTransmission();
}

// ===== FREERTOS DEBOUNCE TASKS =====

/**
 * Debounce task for a single limit switch
 *
 * This task sleeps until woken by the ISR. When woken:
 * 1. Waits DEBOUNCE_TIME_MS for signal to stabilize
 * 2. Reads the GPIO state (now stable after debounce)
 * 3. Updates the stable state bitmap
 * 4. Sets flag for main loop to process
 * 5. Goes back to sleep
 *
 * Each limit switch gets its own task, so they debounce in parallel.
 *
 * @param pvParameters: Pointer to uint8_t containing this switch's index (0-5)
 */
void limit_debounce_task(void *pvParameters) {
    // Extract which limit switch this task handles (0-5)
    uint8_t index = *(uint8_t*)pvParameters;

    while (true) {
        // SLEEP HERE until ISR wakes us with xTaskNotifyFromISR()
        // pdTRUE = clear notification count after waking
        // portMAX_DELAY = wait forever (no timeout)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // DEBOUNCE DELAY: Wait for electrical bounce to settle
        // During this time, this task sleeps but other tasks/ISRs keep running
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));

        // READ STABLE STATE: After debounce period, read actual GPIO level
        // NC switches: LOW = closed (not hit), HIGH = open (hit or wire break)
        bool pin_high = (digitalRead(LIMIT_SWITCH_PINS[index]) == HIGH);
        uint8_t bit_mask = (1 << index);  // Bit position for this switch

        // UPDATE STABLE STATE: Must be thread-safe since main loop also reads
        taskENTER_CRITICAL(&limit_state_spinlock);  // Acquire spinlock, block task switches

        if (pin_high) {
            limit_stable_state |= bit_mask;   // Set bit = limit hit
        } else {
            limit_stable_state &= ~bit_mask;  // Clear bit = limit clear
        }
        limit_flag_changed[index] = true;      // Tell main loop to check
        limit_trigger_time[index] = millis();  // Timestamp for logging

        taskEXIT_CRITICAL(&limit_state_spinlock);  // Release spinlock, re-enable task switches

        // Loop back to ulTaskNotifyTake() and sleep again
    }
}

/**
 * Debounce task for E-Stop button
 *
 * This task sleeps until woken by the ISR. When woken:
 * 1. Waits DEBOUNCE_TIME_MS for signal to stabilize
 * 2. Reads the GPIO state (now stable after debounce)
 * 3. Updates the stable state atomically
 * 4. Sets flag and timestamp for main loop to process
 * 5. Goes back to sleep
 *
 * E-Stop has no hardware filtering, so software debounce is critical.
 */
void estop_debounce_task(void *pvParameters) {
    while (true) {
        // SLEEP HERE until ISR wakes us with xTaskNotifyFromISR()
        // pdTRUE = clear notification count after waking
        // portMAX_DELAY = wait forever (no timeout)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // DEBOUNCE DELAY: Wait for switch bounce to settle
        // During this time, this task sleeps but other tasks/ISRs keep running
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));

        // READ STABLE STATE: After debounce period, read actual GPIO level
        // E-Stop pin: LOW = pressed (active), HIGH = released (inactive due to INPUT_PULLUP)
        bool estop_pressed = (digitalRead(ESTOP_PIN) == LOW);

        // UPDATE STABLE STATE: Must be thread-safe since main loop also reads
        taskENTER_CRITICAL(&estop_state_spinlock);  // Acquire spinlock, block task switches

        estop_stable_state = estop_pressed;           // Update debounced state
        estop_flag_changed = true;                    // Tell main loop to check
        estop_trigger_time = millis();                // Timestamp for logging

        taskEXIT_CRITICAL(&estop_state_spinlock);  // Release spinlock, re-enable task switches

        // Loop back to ulTaskNotifyTake() and sleep again
    }
}

// ===== ISR HANDLERS (Flag-setting only, no GPIO reads) =====

// E-Stop ISR
void IRAM_ATTR estop_isr() {
    // Wake the E-Stop debounce task
    // The task will handle debouncing and state update
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the task: "Wake up and debounce the E-Stop switch"
    vTaskNotifyGiveFromISR(estop_debounce_task_handle, &xHigherPriorityTaskWoken);

    // If woken task has higher priority than current task, switch to it immediately
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Generic limit switch handler (template function)
void IRAM_ATTR handle_limit_isr(uint8_t index) {
    // Wake the corresponding debounce task
    // The task will handle debouncing and state update
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the task: "Wake up and debounce this switch"
    // The task will sleep for DEBOUNCE_TIME_MS, then read GPIO
    vTaskNotifyGiveFromISR(limit_debounce_task_handles[index], &xHigherPriorityTaskWoken);

    // If woken task has higher priority than current task, switch to it immediately
    // This ensures debounce tasks (priority 2) preempt main loop (priority 1)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Limit switch ISR wrappers (ESP32 requires separate functions)
void IRAM_ATTR limit_switch_0_isr() { handle_limit_isr(0); }
void IRAM_ATTR limit_switch_1_isr() { handle_limit_isr(1); }
void IRAM_ATTR limit_switch_2_isr() { handle_limit_isr(2); }
void IRAM_ATTR limit_switch_3_isr() { handle_limit_isr(3); }
void IRAM_ATTR limit_switch_4_isr() { handle_limit_isr(4); }
void IRAM_ATTR limit_switch_5_isr() { handle_limit_isr(5); }

// ISR function pointer array (for setup loop)
void (*LIMIT_SWITCH_ISR_HANDLERS[NUM_LIMIT_SWITCHES])() = {
    limit_switch_0_isr, limit_switch_1_isr, limit_switch_2_isr,
    limit_switch_3_isr, limit_switch_4_isr, limit_switch_5_isr
};

// ===== SAFETY MONITORING FUNCTIONS =====

/**
 * Check E-Stop state and return relay mask
 *
 * Debounce task has already filtered state with software 50ms debounce.
 * Main loop detects edges (state changes) and handles logging.
 *
 * Returns:
 *   0xFFFF - E-Stop released, all relays OFF (safe)
 *   ESTOP_RELAY_STATE - E-Stop pressed, engage all brakes + drivers (except gripper)
 */
uint16_t check_estop_state() {
    // Check if debounce task detected a change
    if (estop_flag_changed) {
        // Acquire spinlock to read debounced state safely
        taskENTER_CRITICAL(&estop_state_spinlock);
        bool current_estop_state = estop_stable_state;
        taskEXIT_CRITICAL(&estop_state_spinlock);

        // EDGE DETECTION: Only act if state actually changed
        if (current_estop_state != last_estop_stable_state) {
            last_estop_stable_state = current_estop_state;

            // Log state change
            if (current_estop_state) {
                Serial.println("E-STOP ACTIVATED");
                digitalWrite(STATUS_LED, HIGH);
            } else {
                Serial.println("E-Stop released");
                digitalWrite(STATUS_LED, LOW);
            }
        }

        // Clear flag
        estop_flag_changed = false;
    }

    // Return relay mask (active-low logic)
    // Pressed: engage brakes+drivers (except gripper)
    // Released: all relays off
    return last_estop_stable_state ? ESTOP_RELAY_STATE : 0xFFFF;
}

/**
 * Check all limit switches and return composed relay mask
 *
 * Debounce tasks have already filtered state and updated limit_stable_state.
 * This function detects edges (state changes) and composes the relay mask.
 *
 * Returns:
 *   0xFFFF - All limits clear, all relays OFF (safe)
 *   Composed mask - One or more limits hit, engage corresponding relays
 */
uint16_t check_limit_switches() {
    // Acquire spinlock to read stable state safely (debounce tasks might be updating it)
    taskENTER_CRITICAL(&limit_state_spinlock);
    uint8_t current_limit_state = limit_stable_state;
    taskEXIT_CRITICAL(&limit_state_spinlock);

    // EDGE DETECTION: Debounce tasks updated limit_stable_state
    // We only log if the state actually changed
    if (current_limit_state != last_limit_stable_state) {
        Serial.print("LIMIT SWITCHES: 0b");
        Serial.print(current_limit_state, BIN);
        Serial.print(" (");

        // Log which switches are active
        bool first = true;
        for (uint8_t i = 0; i < NUM_LIMIT_SWITCHES; i++) {
            if (current_limit_state & (1 << i)) {
                if (!first) Serial.print(", ");
                Serial.print("SW");
                Serial.print(i);
                first = false;
            }
        }
        if (first) Serial.print("NONE");
        Serial.println(")");

        last_limit_stable_state = current_limit_state;
    }

    // COMPOSE RELAY MASK: Start with 0xFFFF (all relays off)
    // AND with each active switch's mask to engage its relays
    // If multiple switches hit, AND them all together to engage all needed brakes
    uint16_t result = 0xFFFF;
    for (uint8_t i = 0; i < NUM_LIMIT_SWITCHES; i++) {
        if (current_limit_state & (1 << i)) {
            result &= LIMIT_SWITCH_RELAY_MASKS[i];
        }
    }

    return result;
}

// ===== MODBUS HANDLERS =====

/**
 * FC02 Handler - Read Discrete Inputs (ESP32 → PC status reporting)
 *
 * Returns current safety state to PC:
 * - DI 0: E-Stop state (1=pressed, 0=released)
 * - DI 1-6: Limit switches 0-5 (1=hit, 0=clear)
 * - DI 7-9: Brake states J0-J2 (1=engaged, 0=released)
 * - DI 10-13: Driver states J0-J2, Wrist (1=enabled, 0=disabled)
 * - DI 14: Gripper state (1=engaged, 0=released)
 *
 * Thread-safe: Uses spinlocks to read stable state
 */
ModbusMessage FC02_handler(ModbusMessage request) {
    ModbusMessage response;
    uint16_t start_addr = 0;
    uint16_t num_inputs = 0;

    // Parse request
    request.get(2, start_addr);
    request.get(4, num_inputs);

    // Validate request (we have 15 discrete inputs: 0-14)
    if (start_addr + num_inputs > 15) {
        response.setError(request.getServerID(), request.getFunctionCode(), Error::ILLEGAL_DATA_ADDRESS);  // ILLEGAL_DATA_ADDRESS
        return response;
    }

    // Read current state (thread-safe)
    taskENTER_CRITICAL(&estop_state_spinlock);
    bool estop = estop_stable_state;
    taskEXIT_CRITICAL(&estop_state_spinlock);

    taskENTER_CRITICAL(&limit_state_spinlock);
    uint8_t limits = limit_stable_state;
    taskEXIT_CRITICAL(&limit_state_spinlock);

    uint16_t current_relay_state = last_relay_state;

    // Build discrete input array (15 inputs)
    bool discrete_inputs[15] = {false};

    // DI 0: E-Stop (1=pressed, 0=released)
    discrete_inputs[MODBUS_DI_ESTOP] = estop;

    // DI 1-6: Limit switches (1=hit, 0=clear)
    for (uint8_t i = 0; i < NUM_LIMIT_SWITCHES; i++) {
        discrete_inputs[MODBUS_DI_LIMIT_0 + i] = (limits & (1 << i)) != 0;
    }

    // DI 7-9: Brake states (1=engaged, 0=released)
    // Relay mask logic: 0 bit = relay engaged (brake engaged)
    discrete_inputs[MODBUS_DI_BRAKE_J0] = (current_relay_state & ~J0_BRAKE_RELAY) != 0;
    discrete_inputs[MODBUS_DI_BRAKE_J1] = (current_relay_state & ~J1_BRAKE_RELAY) != 0;
    discrete_inputs[MODBUS_DI_BRAKE_J2] = (current_relay_state & ~J2_BRAKE_RELAY) != 0;

    // DI 10-13: Driver states (1=enabled, 0=disabled)
    // Relay mask logic: 0 bit = relay engaged (driver enabled)
    discrete_inputs[MODBUS_DI_DRIVER_J0] = (current_relay_state & ~J0_DRIVER_RELAY) != 0;
    discrete_inputs[MODBUS_DI_DRIVER_J1] = (current_relay_state & ~J1_DRIVER_RELAY) != 0;
    discrete_inputs[MODBUS_DI_DRIVER_J2] = (current_relay_state & ~J2_DRIVER_RELAY) != 0;
    discrete_inputs[MODBUS_DI_DRIVER_WRIST] = (current_relay_state & ~WRIST_DRIVER_RELAY) != 0;

    // DI 14: Gripper state (1=engaged, 0=released)
    // Relay mask logic: 0 bit = relay engaged (gripper engaged)
    discrete_inputs[MODBUS_DI_GRIPPER] = (current_relay_state & ~GRIPPER_RELAY) != 0;

    // Pack discrete inputs into bytes for Modbus response
    uint8_t byte_count = (num_inputs + 7) / 8;
    uint8_t data_bytes[2] = {0, 0};  // Max 15 inputs = 2 bytes

    for (uint16_t i = 0; i < num_inputs; i++) {
        if (discrete_inputs[start_addr + i]) {
            data_bytes[i / 8] |= (1 << (i % 8));
        }
    }

    // Build response
    response.add(request.getServerID(), request.getFunctionCode(), byte_count);
    for (uint8_t i = 0; i < byte_count; i++) {
        response.add(data_bytes[i]);
    }

    return response;
}

/**
 * FC05 Handler - Write Single Coil (PC → ESP32 commands)
 *
 * Receives commands from PC:
 * - Coil 0: Gripper control (1=engage, 0=release) - ALWAYS allowed
 * - Coil 1-3: Brake release (1=release, 0=engage) - BLOCKED if E-Stop active
 *
 * Commands are stored in global variables and processed by main loop.
 * E-Stop guard ensures brake release commands are rejected when E-Stop active.
 *
 * Thread-safe: Uses spinlock to write command state
 */
ModbusMessage FC05_handler(ModbusMessage request) {
    ModbusMessage response;
    uint16_t coil_addr = 0;
    uint16_t coil_value = 0;

    // Parse request
    request.get(2, coil_addr);
    request.get(4, coil_value);

    // Validate coil address (we have 4 coils: 0-3)
    if (coil_addr > MODBUS_COIL_BRAKE_J2) {
        response.setError(request.getServerID(), request.getFunctionCode(), Error::ILLEGAL_DATA_ADDRESS);  // ILLEGAL_DATA_ADDRESS
        return response;
    }

    // Convert Modbus value to boolean (0xFF00 = ON, 0x0000 = OFF)
    bool cmd_value = (coil_value == 0xFF00);

    // Read current E-Stop state (thread-safe)
    taskENTER_CRITICAL(&estop_state_spinlock);
    bool estop_active = estop_stable_state;
    taskEXIT_CRITICAL(&estop_state_spinlock);

    // E-STOP GUARD: Block brake release commands if E-Stop active
    // Gripper commands always allowed (safety release)
    if (estop_active && coil_addr >= MODBUS_COIL_BRAKE_J0) {
        // E-Stop active, reject brake release command
        response.setError(request.getServerID(), request.getFunctionCode(), Error::SERVER_DEVICE_FAILURE);  // SERVER_DEVICE_FAILURE
        #ifdef DEBUG
        Serial.print("MODBUS: Rejected brake release (coil ");
        Serial.print(coil_addr);
        Serial.println(") - E-Stop active");
        #endif
        return response;
    }

    // Update command state (thread-safe)
    taskENTER_CRITICAL(&modbus_cmd_spinlock);
    switch (coil_addr) {
        case MODBUS_COIL_GRIPPER:
            modbus_gripper_cmd = cmd_value;
            break;
        case MODBUS_COIL_BRAKE_J0:
            modbus_brake_release_j0 = cmd_value;
            break;
        case MODBUS_COIL_BRAKE_J1:
            modbus_brake_release_j1 = cmd_value;
            break;
        case MODBUS_COIL_BRAKE_J2:
            modbus_brake_release_j2 = cmd_value;
            break;
    }
    taskEXIT_CRITICAL(&modbus_cmd_spinlock);

    #ifdef DEBUG
    Serial.print("MODBUS: FC05 coil ");
    Serial.print(coil_addr);
    Serial.print(" = ");
    Serial.println(cmd_value ? "ON" : "OFF");
    #endif

    // Echo request as response (standard Modbus behavior)
    response.add(request.getServerID(), request.getFunctionCode(), coil_addr, coil_value);
    return response;
}

void setup(){
    Serial.begin(115200);
    delay(5000);

    safety_setup();
}

void safety_setup()
{
    bool wire_success = Wire.begin(I2C_SDA, I2C_SCL);
    if (wire_success)
    {
        Serial.println("Wire successfully initialized");
    }
    else
    {
        Serial.println("Failed to initialize Wire");
        while (true)
        {
            delay(100);
        }
    }

    // Configure pin modes FIRST (before creating tasks or attaching interrupts)
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED, OUTPUT);

    for (uint8_t i = 0; i < NUM_LIMIT_SWITCHES; i++)
    {
        pinMode(LIMIT_SWITCH_PINS[i], INPUT_PULLUP);
    }

    // CREATE ALL TASKS BEFORE ATTACHING INTERRUPTS
    // This ensures task handles are valid when ISRs fire

    // Create E-Stop debounce task
    // Runs on Core 0 with priority 3 (higher than main loop priority 1)
    xTaskCreatePinnedToCore(
        estop_debounce_task,
        "EstopDebounce",
        2048,                        // Stack size in bytes
        NULL,                        // No parameters needed (no index like limit switches)
        3,                           // Priority (higher than main loop at priority 1)
        &estop_debounce_task_handle, // Handle to store task reference
        0                            // Run on Core 0 (dedicated safety core)
    );

    Serial.println("Created debounce task for E-Stop");

    // Create debounce tasks for each limit switch
    // Each task runs on Core 0 with priority 2 (higher than main loop priority 1)
    for (uint8_t i = 0; i < NUM_LIMIT_SWITCHES; i++)
    {
        // Allocate memory for index parameter
        uint8_t *pIndex = (uint8_t *)malloc(sizeof(uint8_t));
        *pIndex = i;

        char task_name[16];
        snprintf(task_name, sizeof(task_name), "LimitDebounce%d", i);

        // Create task:
        // - limit_debounce_task: The task function that will debounce
        // - task_name: Name for debugging (e.g., "LimitDebounce0")
        // - 2048: Stack size in bytes (enough for FreeRTOS overhead + GPIO reads)
        // - pIndex: Pointer to index parameter (which limit switch this task handles)
        // - 2: Priority (higher than main loop at priority 1)
        // - &limit_debounce_task_handles[i]: Handle to store task reference
        // - 0: Run on Core 0 (dedicated safety core)
        xTaskCreatePinnedToCore(
            limit_debounce_task,
            task_name,
            2048,
            (void *)pIndex,
            2,
            &limit_debounce_task_handles[i],
            0);

        Serial.print("Created debounce task for limit switch ");
        Serial.println(i);
    }

    Serial.println("All debounce tasks created on Core 0");

    // NOW attach interrupts (task handles are guaranteed valid)
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estop_isr, CHANGE);
    Serial.println("E-Stop configured");

    // Attach limit switch interrupts
    for (uint8_t i = 0; i < NUM_LIMIT_SWITCHES; i++)
    {
        attachInterrupt(
            digitalPinToInterrupt(LIMIT_SWITCH_PINS[i]),
            LIMIT_SWITCH_ISR_HANDLERS[i],
            CHANGE);
    }

    Serial.print("Configured ");
    Serial.print(NUM_LIMIT_SWITCHES);
    Serial.println(" limit switches");

    // Initialize Modbus RTU server
    Serial.println("\nInitializing Modbus RTU server...");

    // Prepare UART2 buffer sizes for Modbus
    RTUutils::prepareHardwareSerial(Serial2);

    // Initialize UART2 for RS-485 communication
    Serial2.begin(MODBUS_BAUDRATE, SERIAL_8N1, MODBUS_RX, MODBUS_TX);

    // Register Modbus function code handlers
    MBserver.registerWorker(MODBUS_SLAVE_ID, READ_DISCR_INPUT, &FC02_handler);
    MBserver.registerWorker(MODBUS_SLAVE_ID, WRITE_COIL, &FC05_handler);

    // Start Modbus server (runs on Core 1 by default)
    MBserver.begin(Serial2);

    Serial.print("Modbus RTU server started on UART2 (slave ID: ");
    Serial.print(MODBUS_SLAVE_ID);
    Serial.print(", baud: ");
    Serial.print(MODBUS_BAUDRATE);
    Serial.println(")");

    Serial.println("\nTesting relay board...");
    Serial.println("Engaging all brakes (0xFFFF)");

    pcf8575_writeAll(0xFFFF);
    delay(1000);

    Serial.println("Releasing all brakes (0x0000)");
    pcf8575_writeAll(0x0000);
    delay(1000);

    Serial.println("Relay test complete \n");
    Serial.println("Ready");
}

void loop() {
    safety_monitor();
}
void safety_monitor()
{
    // Get relay states from safety monitoring functions
    // 0xFFFF = all relays OFF (safe, no faults)
    // Specific mask = engage relays for that fault (bits=0 where engaged)
    uint16_t estop_relays = check_estop_state();
    uint16_t limit_relays = check_limit_switches();

    // Compose with AND (active-low: engage if EITHER has 0 bits)
    // If E-Stop pressed: estop_relays has multiple 0 bits → engages those relays
    // If Limit hit: limit_relays has 0 bits for that joint → engages those relays
    // AND combines them: 0 bits from either source engage relays
    uint16_t relay_state = estop_relays & limit_relays;

    // Process Modbus commands (applied AFTER safety checks)
    taskENTER_CRITICAL(&modbus_cmd_spinlock);
    bool gripper_cmd = modbus_gripper_cmd;
    bool brake_j0_release = modbus_brake_release_j0;
    bool brake_j1_release = modbus_brake_release_j1;
    bool brake_j2_release = modbus_brake_release_j2;
    taskEXIT_CRITICAL(&modbus_cmd_spinlock);

    // Apply gripper command (always allowed)
    if (gripper_cmd)
    {
        relay_state &= GRIPPER_RELAY; // Engage gripper
    }
    else
    {
        relay_state |= ~GRIPPER_RELAY; // Release gripper
    }

    // Apply brake release commands (only if E-Stop inactive - FC05 handler already blocks these)
    // Safety layer: Double-check E-Stop state here as well
    if (!last_estop_stable_state)
    {
        if (brake_j0_release)
        {
            relay_state |= ~J0_BRAKE_RELAY; // Release J0 brake
        }
        if (brake_j1_release)
        {
            relay_state |= ~J1_BRAKE_RELAY; // Release J1 brake
        }
        if (brake_j2_release)
        {
            relay_state |= ~J2_BRAKE_RELAY; // Release J2 brake
        }
    }

    // Execute only if changed (edge detection prevents relay chatter)
    if (relay_state != last_relay_state)
    {
        Serial.print("Relay change: 0x");
        Serial.print(last_relay_state, HEX);
        Serial.print(" -> 0x");
        Serial.println(relay_state, HEX);

        pcf8575_writeAll(relay_state);
        last_relay_state = relay_state;
    }

    // USER PLACEHOLDER: Add more safety layers here
    // Example: relay_state &= check_motion_planning_limits();
    // Example: relay_state &= check_current_sensors();
    // Example: relay_state &= check_rs485_faults();

    // Maintain 10kHz loop rate for fast response
    delayMicroseconds(100);
}

void engage_relay_state(uint16_t relay_state_)
{
    pcf8575_writeAll(relay_state_);
}

void disengage_relay_state(uint16_t relay_state_){
    pcf8575_writeAll(relay_state_ | ~relay_state_);
}