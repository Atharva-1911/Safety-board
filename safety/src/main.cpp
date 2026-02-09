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

// System constants
#define DEBOUNCE_TIME_MS 50 // Debounce time in milliseconds
#define NUM_LIMIT_SWITCHES 6

// pin configuration
#define ESTOP_PIN 13
#define STATUS_LED 2
#define I2C_SDA 21
#define I2C_SCL 22
#define RXD2 16          // RS485 (UART2) (PyModbus)
#define TXD2 17
#define RTS  18

#define LIMIT_SWITCH_0 14

const uint8_t LIMIT_SWITCH_PINS[NUM_LIMIT_SWITCHES] = {14, 27, 26, 25, 33, 32};


// system configuration
#define PCF8575_ADDR 0x20

// convinience constants: Relay masks
#define J0_BRAKE_RELAY 0xEFFF
#define J1_BRAKE_RELAY 0xFFFB
#define J2_BRAKE_RELAY 0xDFFF

#define J0_DRIVER_RELAY 0xFFFD 
#define J1_DRIVER_RELAY 0xBFFF
#define J2_DRIVER_RELAY 0xFFFE
#define J3_DRIVER_RELAY 0x7FFF
#define J4_DRIVER_RELAY 0xF7FF
#define J5_DRIVER_RELAY 0xFFEF 


#define GRIPPER_RELAY 0xFBFF


// Limit switch configuration

// Safety event relay masks
// === GROUP MASKS ===

// All brake relays ON
#define ALL_BRAKES_MASK ( \
    J0_BRAKE_RELAY & \
    J1_BRAKE_RELAY & \
    J2_BRAKE_RELAY \
)

// All driver relays ON
#define ALL_DRIVERS_MASK ( \
    J0_DRIVER_RELAY & \
    J1_DRIVER_RELAY & \
    J2_DRIVER_RELAY & \
    J3_DRIVER_RELAY & \
    J4_DRIVER_RELAY & \
    J5_DRIVER_RELAY  \
)

// Gripper relay stays HIGH (released) to prevent dropping held objects

const uint16_t LIMIT_SWITCH_RELAY_MASKS[NUM_LIMIT_SWITCHES] = {
    J0_BRAKE_RELAY ,    // Limit 0 → J0 brake ON, driver OFF
    J1_BRAKE_RELAY ,    // Limit 1 → J1 brake ON, driver OFF
    J2_BRAKE_RELAY ,    // Limit 2 → J2 brake ON, driver OFF,
    0xFFFF,
    0xFFFF,
    0xFFFF
};

//global variables

// Modbus RTU server (ESP32 acts as slave)
ModbusServerRTU mb(2000, RTS);   // 2s timeout, RTS pin for RS485 direction

volatile uint16_t modbus_relay_value = 0xFFFF;  // default: all relays OFF


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
        // NO switches: HIGH = closed(HIT), LOW = open (CLEAR)
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
    // Pressed: all relays off
    // Released: engage brakes+drivers (except gripper)
    // Return ONLY the E-STOP state, no relay decisions here
    return last_estop_stable_state ? 0x0001 : 0x0000;

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
    //uint16_t result = 0xFFFF;
    //for (uint8_t i = 0; i < NUM_LIMIT_SWITCHES; i++) {
        //if (current_limit_state & (1 << i)) {
        //    result &= LIMIT_SWITCH_RELAY_MASKS[i];
        //}
    //}

    // Return 1 if ANY limit switch is active
return (current_limit_state != 0);

Serial.print("LIMIT STATE = ");
Serial.println(limit_stable_state, BIN);

}


// ===== MODBUS FUNCTION HANDLERS =====
// Function Code 0x06: Write Single Register
// Used to set relay states

ModbusMessage FC06(ModbusMessage request) {
    uint16_t address, value;
    ModbusMessage response;

    request.get(2, address);
    request.get(4, value);

    if (address == 0) {
        modbus_relay_value = value;   // direct relay mask
        response = request;           // echo back
    } else {
        response.setError(
            request.getServerID(),
            request.getFunctionCode(),
            ILLEGAL_DATA_ADDRESS
        );
    }
    return response;
}


//Gripper function 
uint16_t apply_gripper(uint16_t relay_state)
{
    // get only gripper bit from modbus
    uint16_t gripper_bit = modbus_relay_value & (~GRIPPER_RELAY);
    
    //clear gripper bit from relay state and then insert modbus gripper bit
    relay_state = (relay_state & GRIPPER_RELAY) | gripper_bit;

    return relay_state;
}




// SETUP
void setup(){
    Serial.begin(115200);
    delay(5000);
    

    bool wire_success = Wire.begin(I2C_SDA, I2C_SCL);
    if (wire_success){
        Serial.println("Wire successfully initialized");}    
    else {
        Serial.println("Failed to initialize Wire");
        while (true){delay(100);}
    }
    
    // Pymodbus RTU over HardwareSerial2

    RTUutils::prepareHardwareSerial(Serial2);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
    mb.registerWorker(1, WRITE_HOLD_REGISTER, &FC06);
    mb.begin(Serial2);



    // Configure E-Stop pin and attach interrupt
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estop_isr, CHANGE);
    pinMode(STATUS_LED, OUTPUT);
    Serial.println("E-Stop configured");

    // Configure all limit switches using array-based approach
    for (uint8_t i = 0; i < NUM_LIMIT_SWITCHES; i++) {
        pinMode(LIMIT_SWITCH_PINS[i], INPUT_PULLUP);
        attachInterrupt(
            digitalPinToInterrupt(LIMIT_SWITCH_PINS[i]),
            LIMIT_SWITCH_ISR_HANDLERS[i],
            CHANGE
        );
    }

    Serial.print("Configured ");
    Serial.print(NUM_LIMIT_SWITCHES);
    Serial.println(" limit switches");

    // Create E-Stop debounce task
    // Runs on Core 0 with priority 2 (higher than main loop priority 1)
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
    for (uint8_t i = 0; i < NUM_LIMIT_SWITCHES; i++) {
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
            0
        );

        Serial.print("Created debounce task for limit switch ");
        Serial.println(i);
    }

    Serial.println("All debounce tasks created on Core 0");
    Serial.println("\\nTesting relay board...");
    Serial.println("Engaging all brakes (0xFFFF)");

    pcf8575_writeAll(0xFFFF);
    delay(1000);



    Serial.println("Relay test complete \n");
    Serial.println("Ready");
    
}

void loop() {
    // Get relay states from safety monitoring functions
    // 0xFFFF = all relays OFF (safe, no faults)
    // Specific mask = engage relays for that fault (bits=0 where engaged)
    //uint16_t estop_relays = check_estop_state();
    //uint16_t limit_relays = check_limit_switches();

    // Compose with AND (active-low: engage if EITHER has 0 bits)
    // If E-Stop pressed: estop_relays has multiple 0 bits → engages those relays
    // If Limit hit: limit_relays has 0 bits for that joint → engages those relays
    // AND combines them: 0 bits from either source engage relays
    //uint16_t relay_state = estop_relays & limit_relays;
    //bool estop_pressed = check_estop_state();
    //bool limit_hit     = check_limit_switches();

    // Highest-priority safety decision
   bool estop = check_estop_state();
   uint16_t relay_state;

    if (estop) {
    relay_state = 0xFFFF;          // stop motion
    }

   else {
    // Start from Modbus (base state)
    relay_state = modbus_relay_value ;//& ALL_DRIVERS_MASK & ALL_BRAKES_MASK;

    taskENTER_CRITICAL(&limit_state_spinlock);

    if (limit_stable_state & (1 << 0)) {
        relay_state &= ALL_DRIVERS_MASK;   // force OFF
        relay_state &= J0_BRAKE_RELAY;
    }
    else if (limit_stable_state & (1 << 1)) {
        relay_state &= ALL_DRIVERS_MASK;
        relay_state &= J1_BRAKE_RELAY;
    }
    else if (limit_stable_state & (1 << 2)) {
        relay_state &= ALL_DRIVERS_MASK;
        relay_state &= J2_BRAKE_RELAY;
    }

    taskEXIT_CRITICAL(&limit_state_spinlock);
}


   



    //gripper priority
    relay_state = apply_gripper(relay_state);


    // Execute only if changed (edge detection prevents relay chatter)
    if (relay_state != last_relay_state) {
       


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