#include <stdint.h>
#include <stdbool.h>
#include <stdio.h> // Required for printf
#include "tm4c123gh6pm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ================= HARDWARE MASKS ================= */
#define LED_RED      (1U << 1)
#define LED_BLUE     (1U << 2)
#define LED_GREEN    (1U << 3)
#define LED_MASK     (LED_RED | LED_BLUE | LED_GREEN)

#define BTN_PF4      (1U << 4) // Driver OPEN (Active Low)
#define BTN_PF0      (1U << 0) // Driver CLOSE (Active Low, Pull-up, SW2)
#define BTN_PE1      (1U << 1) // Security OPEN
#define BTN_PB0      (1U << 0) // Security CLOSE
#define BTN_PB1      (1U << 1) // Open LIMIT
#define BTN_PD0      (1U << 0) // Closed LIMIT
#define BTN_PD1      (1U << 1) // Obstacle

#define RCGCGPIO_B   (1U << 1)
#define RCGCGPIO_D   (1U << 3)
#define RCGCGPIO_E   (1U << 4)
#define RCGCGPIO_F   (1U << 5)
#define RCGCGPIO_ALL (RCGCGPIO_B | RCGCGPIO_D | RCGCGPIO_E | RCGCGPIO_F)

// Increased to 500ms so simulator mouse clicks register as a "Tap"
#define TAP_THRESHOLD pdMS_TO_TICKS(500)

/* ================= RTOS EVENTS ================= */
typedef enum {
    EV_DRV_OPEN_PRESS,   EV_DRV_OPEN_RELEASE,
    EV_DRV_CLOSE_PRESS,  EV_DRV_CLOSE_RELEASE,
    EV_SEC_OPEN_PRESS,   EV_SEC_OPEN_RELEASE,
    EV_SEC_CLOSE_PRESS,  EV_SEC_CLOSE_RELEASE
} EventType_t;

typedef struct {
    EventType_t type;
    uint32_t durationTicks; 
} GateEvent_t;

/* ================= FSM STATES ================= */
typedef enum {
    IDLE_OPEN,
    IDLE_CLOSED,
    OPENING,
    CLOSING,
    STOPPED_MIDWAY,
    REVERSING
} GateState_t;

/* ================= RTOS OBJECTS ================= */
QueueHandle_t xEventQueue;
QueueHandle_t xSafetyQueue;
SemaphoreHandle_t xLimitSemaphore;
SemaphoreHandle_t xStateMutex;

volatile GateState_t gateState = IDLE_CLOSED;

volatile bool hitOpenLimit = false;
volatile bool hitCloseLimit = false;

/* ========================================================= */
/* DEBUG FUNCTIONS                                           */
/* ========================================================= */
static const char* GetStateName(GateState_t state)
{
    switch(state)
    {
        case IDLE_OPEN:      return "IDLE_OPEN";
        case IDLE_CLOSED:    return "IDLE_CLOSED";
        case OPENING:        return "OPENING";
        case CLOSING:        return "CLOSING";
        case STOPPED_MIDWAY: return "STOPPED_MIDWAY";
        case REVERSING:      return "REVERSING";
        default:             return "UNKNOWN";
    }
}

static void Debug_PrintState(GateState_t state)
{
    // Using printf for the simulator/terminal
    printf("[GateTask] State changed to: %s\r\n", GetStateName(state));
}

/* ========================================================= */
/* GPIO INIT                                                 */
/* ========================================================= */
static void GPIO_Init(void)
{
    SYSCTL_RCGCGPIO_R |= RCGCGPIO_ALL;
    while ((SYSCTL_PRGPIO_R & RCGCGPIO_ALL) != RCGCGPIO_ALL) { }

		// Port F: RGB (Outputs), PF4 SW1 (Input, Pull-up), PF0 SW2 (Input, Pull-up)
		GPIO_PORTF_LOCK_R  =  0x4C4F434B;          // Unlock PF0 (SW2 is locked by default)
		GPIO_PORTF_CR_R    |=  BTN_PF0;            // Commit PF0
		GPIO_PORTF_AMSEL_R &= ~(BTN_PF4 | BTN_PF0 | LED_MASK);
		GPIO_PORTF_PCTL_R  &= ~0x000FFFF1U;        // also clear PF0 PCTL bits
		GPIO_PORTF_AFSEL_R &= ~(BTN_PF4 | BTN_PF0 | LED_MASK);
		GPIO_PORTF_DIR_R   |=  LED_MASK;
		GPIO_PORTF_DIR_R   &= ~(BTN_PF4 | BTN_PF0);
		GPIO_PORTF_PUR_R   |=  (BTN_PF4 | BTN_PF0);
		GPIO_PORTF_DEN_R   |=  BTN_PF4 | BTN_PF0 | LED_MASK;
		GPIO_PORTF_DATA_R  &= ~LED_MASK;

		// Port E: only PE1 remains (Security OPEN) — remove PE0
		GPIO_PORTE_AMSEL_R &= ~BTN_PE1;
		GPIO_PORTE_PCTL_R  &= ~0x000000F0U;        // only PE1 bits
		GPIO_PORTE_AFSEL_R &= ~BTN_PE1;
		GPIO_PORTE_DIR_R   &= ~BTN_PE1;
		GPIO_PORTE_PDR_R   |=  BTN_PE1;
		GPIO_PORTE_DEN_R   |=  BTN_PE1;
    // Port B: PB0, PB1 (Inputs, Pull-down)
    GPIO_PORTB_AMSEL_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTB_AFSEL_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_DIR_R   &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_PDR_R   |=  (BTN_PB0 | BTN_PB1);
    GPIO_PORTB_DEN_R   |=  (BTN_PB0 | BTN_PB1);
        
    // Port D: PD0, PD1 (Inputs, Pull-down)
    GPIO_PORTD_AMSEL_R &= ~(BTN_PD0 | BTN_PD1);
    GPIO_PORTD_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTD_AFSEL_R &= ~(BTN_PD0 | BTN_PD1);
    GPIO_PORTD_DIR_R   &= ~(BTN_PD0 | BTN_PD1);
    GPIO_PORTD_PDR_R   |=  (BTN_PD0 | BTN_PD1);
    GPIO_PORTD_DEN_R   |=  (BTN_PD0 | BTN_PD1);
}

/* ========================================================= */
/* BUTTON HELPERS                                            */
/* ========================================================= */
static inline bool Btn_PF4(void) { return (GPIO_PORTF_DATA_R & BTN_PF4) == 0; } // Active-low
static inline bool Btn_PF0(void) { return (GPIO_PORTF_DATA_R & BTN_PF0) == 0; } // Active-low
static inline bool Btn_PE1(void) { return (GPIO_PORTE_DATA_R & BTN_PE1) != 0; }
static inline bool Btn_PB0(void) { return (GPIO_PORTB_DATA_R & BTN_PB0) != 0; }
static inline bool Btn_PB1(void) { return (GPIO_PORTB_DATA_R & BTN_PB1) != 0; }
static inline bool Btn_PD0(void) { return (GPIO_PORTD_DATA_R & BTN_PD0) != 0; }
static inline bool Btn_PD1(void) { return (GPIO_PORTD_DATA_R & BTN_PD1) != 0; }

/* ========================================================= */
/* INPUT TASK (Priority 3)                                   */
/* ========================================================= */
void InputTask(void *pv)
{
    bool prev[7] = {0};
    bool cur[7]  = {0};
    uint32_t pressTick[4] = {0};

    while (1)
    {
        cur[0] = Btn_PF4(); // DRV_OPEN
        cur[1] = Btn_PF0(); // DRV_CLOSE
        cur[2] = Btn_PE1(); // SEC_OPEN
        cur[3] = Btn_PB0(); // SEC_CLOSE
        cur[4] = Btn_PB1(); // LIMIT_OPEN
        cur[5] = Btn_PD0(); // LIMIT_CLOSE
        cur[6] = Btn_PD1(); // OBSTACLE

        /* 1. Conflict Resolution (TC-15, TC-16): Open + Close = Safe Stop */
        if ((cur[0] && cur[1]) || (cur[2] && cur[3])) 
        {
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            GateState_t oldState = gateState;
            
            if (gateState == OPENING || gateState == CLOSING || gateState == REVERSING) {
                gateState = STOPPED_MIDWAY;
            }
            
            if (gateState != oldState) {
                Debug_PrintState(gateState);
            }
            xSemaphoreGive(xStateMutex);
            
            for (int i=0; i<7; i++) prev[i] = cur[i];
            vTaskDelay(pdMS_TO_TICKS(20));
            continue; // Skip queuing events this cycle
        }

        /* 2. Security Priority (TC-13, TC-14): Mask Driver if Security is active */
        if (cur[2] || cur[3]) {
            cur[0] = false; 
            cur[1] = false;
        }

        /* 3. Panel Button Edge Detection (Press/Release duration) */
        GateEvent_t ev;
        for(int i = 0; i < 4; i++) {
            if (cur[i] && !prev[i]) {
                pressTick[i] = xTaskGetTickCount();
                ev.type = (EventType_t)(i * 2); // Map to EV_XXX_PRESS
                ev.durationTicks = 0;
                xQueueSend(xEventQueue, &ev, 0);
            } 
            else if (!cur[i] && prev[i]) {
                ev.type = (EventType_t)((i * 2) + 1); // Map to EV_XXX_RELEASE
                ev.durationTicks = xTaskGetTickCount() - pressTick[i];
                xQueueSend(xEventQueue, &ev, pdMS_TO_TICKS(10));
            }
        }

        /* 4. Limits */
        if (cur[4] && !prev[4]) {
            hitOpenLimit = true;
            xSemaphoreGive(xLimitSemaphore);
        }
        if (cur[5] && !prev[5]) {
            hitCloseLimit = true;
            xSemaphoreGive(xLimitSemaphore);
        }

        /* 5. Obstacle Trigger */
        if (cur[6] && !prev[6]) {
            uint8_t trigger = 1;
            xQueueSend(xSafetyQueue, &trigger, 0);
        }

        for (int i=0; i<7; i++) prev[i] = cur[i];
        vTaskDelay(pdMS_TO_TICKS(20)); // RTOS Debounce
    }
}

void SafetyTask(void *pv)
{
    uint8_t trigger;
    while (1)
    {
        if (xQueueReceive(xSafetyQueue, &trigger, portMAX_DELAY))
        {
            // TC-09: Obstacle detection ignored during OPENING
            // Only triggers while CLOSING (TC-07, TC-08)
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            bool shouldReverse = (gateState == CLOSING);
            xSemaphoreGive(xStateMutex);

            if (shouldReverse)
            {
                // TC-07: Immediately reverse — Green LED ON (handled by LEDTask)
                xSemaphoreTake(xStateMutex, portMAX_DELAY);
                gateState = REVERSING;
                Debug_PrintState(gateState);
                xSemaphoreGive(xStateMutex);

                // TC-07: Reverse for 0.5s
                vTaskDelay(pdMS_TO_TICKS(500));

                // TC-07: After reversal, stop at STOPPED_MIDWAY
                // unless a limit switch already moved us to IDLE_OPEN
                xSemaphoreTake(xStateMutex, portMAX_DELAY);
                if (gateState == REVERSING) {
                    gateState = STOPPED_MIDWAY;
                    Debug_PrintState(gateState);
                }
                xSemaphoreGive(xStateMutex);  // ? always paired — no unmatched give
            }
            // TC-09: if not CLOSING, do nothing — obstacle silently ignored
        }
    }
}
/* ========================================================= */
/* GATE FSM TASK (Priority 2)                                */
/* ========================================================= */
void GateTask(void *pv)
{
    GateEvent_t ev;

    while (1)
    {
        // Wait up to 20ms for a queue event. Timeout allows limit checks.
        if (xQueueReceive(xEventQueue, &ev, pdMS_TO_TICKS(20)))
        {
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            
            GateState_t oldState = gateState; // Record state before event
            bool isHold = (ev.durationTicks >= TAP_THRESHOLD);

            switch (ev.type)
            {
                case EV_DRV_OPEN_PRESS:
                case EV_SEC_OPEN_PRESS:
                  if (gateState != IDLE_OPEN && gateState != REVERSING) 
                       gateState = OPENING;
                    break;

                case EV_DRV_CLOSE_PRESS:
                case EV_SEC_CLOSE_PRESS:
                    if (gateState != IDLE_CLOSED && gateState != REVERSING) 
                      gateState = CLOSING;
										  break;
										

                case EV_DRV_OPEN_RELEASE:
                case EV_SEC_OPEN_RELEASE:
                    // If manually held down, releasing stops movement (TC-01)
                    if (isHold && gateState == OPENING) gateState = STOPPED_MIDWAY;
                    break;

                case EV_DRV_CLOSE_RELEASE:
                case EV_SEC_CLOSE_RELEASE:
                    // If manually held down, releasing stops movement (TC-02)
                    if (isHold && gateState == CLOSING) gateState = STOPPED_MIDWAY;
                    break;
            }

            // Print only if the state actually changed
            if (gateState != oldState) {
                Debug_PrintState(gateState);
            }

            xSemaphoreGive(xStateMutex);
        }

        /* ================= LIMIT VALIDATION ================= */
        // Check semaphore without blocking to quickly process hits
        if (xSemaphoreTake(xLimitSemaphore, 0))
        {
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            
            GateState_t oldState = gateState; // Record state before limit check

            // Open limit only valid if trajectory is UP (TC-12)
            if (hitOpenLimit && (gateState == OPENING || gateState == REVERSING))
                gateState = IDLE_OPEN;

            // Closed limit only valid if trajectory is DOWN (TC-12)
            if (hitCloseLimit && gateState == CLOSING)
                gateState = IDLE_CLOSED;

            hitOpenLimit = false;
            hitCloseLimit = false;

            // Print only if the state actually changed
            if (gateState != oldState) {
                Debug_PrintState(gateState);
            }

            xSemaphoreGive(xStateMutex);
        }
    }
}

/* ========================================================= */
/* LED TASK (Priority 2)                                     */
/* ========================================================= */
void LEDTask(void *pv)
{
    while (1)
    {
        xSemaphoreTake(xStateMutex, portMAX_DELAY);

        if (gateState == OPENING || gateState == REVERSING) 
            GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~LED_MASK) | LED_GREEN;
        else if (gateState == CLOSING) 
            GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~LED_MASK) | LED_RED;
        else 
            GPIO_PORTF_DATA_R &= ~LED_MASK;

        xSemaphoreGive(xStateMutex);
        vTaskDelay(pdMS_TO_TICKS(50)); // Update interval
    }
}

/* ========================================================= */
/* MAIN                                                      */
/* ========================================================= */
int main(void)
{
    GPIO_Init();

    // Print initial boot message
    printf("\r\n==== Smart Gate System Started ====\r\n");
    Debug_PrintState(IDLE_CLOSED);

    xEventQueue     = xQueueCreate(10, sizeof(GateEvent_t));
    xSafetyQueue    = xQueueCreate(5, sizeof(uint8_t));
    xStateMutex     = xSemaphoreCreateMutex();
    xLimitSemaphore = xSemaphoreCreateBinary();

    gateState = IDLE_CLOSED; // Initial State

    xTaskCreate(InputTask,  "Input",  256, NULL, 3, NULL);
    xTaskCreate(SafetyTask, "Safety", 256, NULL, 4, NULL);
    xTaskCreate(GateTask,   "Gate",   256, NULL, 2, NULL);
    xTaskCreate(LEDTask,    "LED",    128, NULL, 2, NULL);

    vTaskStartScheduler();

    while(1);
}