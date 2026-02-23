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

#define LIMIT_J0_PIN 14
#define LIMIT_J1_PIN 27
#define LIMIT_J2_PIN 26
#define LIMIT_J3_PIN 25
#define LIMIT_J4_PIN 33
#define LIMIT_J5_PIN 32
const uint8_t LIMIT_SWITCH_PINS[NUM_LIMIT_SWITCHES] = {14, 27, 26, 25, 33, 32};


// system configuration
#define PCF8575_ADDR 0x20
#define REG_ADDR 0x0000

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
// ALL MASKS 
const uint16_t STARTUP_MASK =(
    J0_DRIVER_RELAY &
    J1_DRIVER_RELAY &
    J2_DRIVER_RELAY &
    J3_DRIVER_RELAY &
    J4_DRIVER_RELAY &
    J5_DRIVER_RELAY &
    J0_BRAKE_RELAY  &
    J1_BRAKE_RELAY  &
    J2_BRAKE_RELAY
);

// Gripper relay stays HIGH (released) to prevent dropping held objects

//global variables

// Modbus RTU server (ESP32 acts as slave)
ModbusServerRTU mb(2000, RTS);   // 2s timeout, RTS pin for RS485 direction

volatile uint16_t modbus_value ;  
volatile uint16_t relay_state;

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
    return last_estop_stable_state ;

}


// ===== MODBUS FUNCTION HANDLERS =====
// Function Code 0x06: Write Single Register
// Used to set relay states

ModbusMessage FC06(ModbusMessage request) {
    uint16_t address, mask;
    ModbusMessage response;

    request.get(2, address);
    request.get(4, mask);

    if (address != REG_ADDR) {
        response.setError(request.getServerID(),
                          request.getFunctionCode(),
                          ILLEGAL_DATA_ADDRESS);
        return response;
    }

    uint16_t toggle_bits = ~mask & 0xFFFF;
    modbus_value ^= toggle_bits;

    response = request;
    return response;
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
    mb.begin(Serial2); pinMode(RTS, OUTPUT); 
    
    Serial.println("Modbus ready, waiting..."); 
    
    // Configure E-Stop pin and attach interrupt 
    pinMode(ESTOP_PIN, INPUT_PULLUP); 
    delay(50); 
    // Force initial E-Stop state manually 
    if (digitalRead(ESTOP_PIN) == LOW) { 
        estop_stable_state = true; // pressed 
        last_estop_stable_state = true; } 
    else { estop_stable_state = false; // released 
        last_estop_stable_state = false; } 
        

    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estop_isr, CHANGE); 
    pinMode(STATUS_LED, OUTPUT); 
    Serial.println("E-Stop configured"); 

    if (estop_stable_state) { // E-Stop pressed 
        Serial.println("E-STOP PRESSED AT STARTUP — RELAYS OFF"); 
        relay_state = 0xFFFF; } else { // E-Stop released 
            relay_state = STARTUP_MASK; } 
            
    // Configure all limit switches using array-based approach 
    for (uint8_t i = 0; i < NUM_LIMIT_SWITCHES; i++) { 
        pinMode(LIMIT_SWITCH_PINS[i], INPUT_PULLUP); 
        attachInterrupt( digitalPinToInterrupt(LIMIT_SWITCH_PINS[i]), 
        LIMIT_SWITCH_ISR_HANDLERS[i], CHANGE ); } 
        Serial.print("Configured "); 
        Serial.print(NUM_LIMIT_SWITCHES); 
        Serial.println(" limit switches"); 
        Serial.println("Initial limit bitmap: ");
        Serial.println(limit_stable_state, BIN); 
        
    // Create E-Stop debounce task // Runs on Core 0 with priority 2 (higher than main loop priority 1) 
    Serial.println("Initial limit bitmap: ");
    Serial.println(limit_stable_state, BIN);

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

    // Decide initial relay state from E-STOP
    if (estop_stable_state) {
        relay_state = 0xFFFF;
    }        // E-STOP pressed → all relays OFF 
    else {
        relay_state = STARTUP_MASK;  // E-STOP released → normal startup
    }


    // After E-Stop released → normal startup 
    modbus_value = relay_state; 
    last_relay_state = relay_state; 
    
    //relay_state = STARTUP_MASK; 
    //modbus_value = STARTUP_MASK; 
    pcf8575_writeAll(relay_state);

    /* ---------- WATCHDOG ---------- */ 
    esp_task_wdt_init(2, true); 
    esp_task_wdt_add(NULL); 

    Serial.println("Relay init complete"); 
    Serial.println("System READY"); 




}

void loop() {
    // Get relay states from safety monitoring functions
    // 0xFFFF = all relays OFF (safe, no faults)
    // Specific mask = engage relays for that fault (bits=0 where engaged)

    // Compose with AND (active-low: engage if EITHER has 0 bits)
    // If E-Stop pressed: estop_relays has multiple 0 bits → engages those relays
    // If Limit hit: limit_relays has 0 bits for that joint → engages those relays
    // AND combines them: 0 bits from either source engage relays
    
    // Highest-priority safety decision
   bool estop = check_estop_state();

    if (estop) {
            relay_state = 0xFFFF;   // E-stop highest priority
        }
    
    else{
    // Start from Modbus (base command)
            relay_state = modbus_value;
        

    taskENTER_CRITICAL(&limit_state_spinlock);

    if (digitalRead(LIMIT_J0_PIN) == LOW)  {
            relay_state |= (~J0_DRIVER_RELAY & 0xFFFF);
            relay_state |= (~J0_BRAKE_RELAY  & 0xFFFF);
        }

    if (digitalRead(LIMIT_J1_PIN) == LOW)  {
            relay_state |= (~J1_DRIVER_RELAY & 0xFFFF);
            relay_state |= (~J1_BRAKE_RELAY  & 0xFFFF);
        }

    if (digitalRead(LIMIT_J2_PIN) == LOW)  {
            relay_state |= (~J2_DRIVER_RELAY & 0xFFFF);
            relay_state |= (~J2_BRAKE_RELAY  & 0xFFFF);
        }

    taskEXIT_CRITICAL(&limit_state_spinlock);
    }

  

    //gripper priority
    uint16_t gripper_bit = modbus_value & (~GRIPPER_RELAY);
    relay_state = (relay_state & GRIPPER_RELAY) | gripper_bit;

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

    esp_task_wdt_reset();

    // Maintain 10kHz loop rate for fast response
    delay(1);
}

