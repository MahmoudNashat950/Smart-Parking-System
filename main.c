#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ================= HARDWARE MASKS ================= */
/* RGB pin masks on Port F */
#define LED_RED      (1U << 1)
#define LED_BLUE     (1U << 2)
#define LED_GREEN    (1U << 3)
#define LED_MASK     (LED_RED | LED_BLUE | LED_GREEN)

/* Button pin masks */
#define BTN_PF4      (1U << 4) // Driver OPEN
#define BTN_PE0      (1U << 0) // Driver CLOSE
#define BTN_PE1      (1U << 1) // Security OPEN
#define BTN_PB0      (1U << 0) // Security CLOSE
#define BTN_PB1      (1U << 1) // Open LIMIT
#define BTN_PD0      (1U << 0) // Closed LIMIT
#define BTN_PD1      (1U << 1) // Obstacle

/* Clock-gate masks */
#define RCGCGPIO_B   (1U << 1)
#define RCGCGPIO_D   (1U << 3)
#define RCGCGPIO_E   (1U << 4)
#define RCGCGPIO_F   (1U << 5)
#define RCGCGPIO_ALL (RCGCGPIO_B | RCGCGPIO_D | RCGCGPIO_E | RCGCGPIO_F)

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
/* OPTIMIZED GPIO INIT                     */
/* ========================================================= */
static void GPIO_Init(void)
{
    /* Enable clocks for ports B, D, E, F and wait until ready */
    SYSCTL_RCGCGPIO_R |= RCGCGPIO_ALL;
    while ((SYSCTL_PRGPIO_R & RCGCGPIO_ALL) != RCGCGPIO_ALL) { }

    /* ---------- Port F: RGB outputs (PF1-3) and button PF4 ---------- */
    GPIO_PORTF_AMSEL_R &= ~(BTN_PF4 | LED_MASK);
    GPIO_PORTF_PCTL_R  &= ~0x000FFFF0U;   
    GPIO_PORTF_AFSEL_R &= ~(BTN_PF4 | LED_MASK);

    GPIO_PORTF_DIR_R   |=  LED_MASK;      
    GPIO_PORTF_DIR_R   &= ~BTN_PF4;       

    GPIO_PORTF_PUR_R   |=  BTN_PF4;       /* pull-up for PF4 */
    GPIO_PORTF_DEN_R   |=  BTN_PF4 | LED_MASK;
    GPIO_PORTF_DATA_R  &= ~LED_MASK;      /* LEDs off initially */

    /* ---------- Port E: PE0, PE1 (pull-down, active-high) ---------- */
    GPIO_PORTE_AMSEL_R &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTE_AFSEL_R &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_DIR_R   &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_PUR_R   |=  (BTN_PE0 | BTN_PE1);
    GPIO_PORTE_DEN_R   |=  (BTN_PE0 | BTN_PE1);


    /* ---------- Port B: PB0, PB1 (pull-down, active-high) ---------- */
    GPIO_PORTB_AMSEL_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTB_AFSEL_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_DIR_R   &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_PDR_R   |=  (BTN_PB0 | BTN_PB1);
    GPIO_PORTB_DEN_R   |=  (BTN_PB0 | BTN_PB1);
		

    /* ---------- Port D: PD0, PD1 (pull-down, active-high) ---------- */
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
static inline uint32_t Btn_PF4(void) { return (GPIO_PORTF_DATA_R & BTN_PF4) == 0; } // Active-low
static inline uint32_t Btn_PE0(void) { return (GPIO_PORTE_DATA_R & BTN_PE0) != 0; }
static inline uint32_t Btn_PE1(void) { return (GPIO_PORTE_DATA_R & BTN_PE1) != 0; }
static inline uint32_t Btn_PB0(void) { return (GPIO_PORTB_DATA_R & BTN_PB0) != 0; }
static inline uint32_t Btn_PB1(void) { return (GPIO_PORTB_DATA_R & BTN_PB1) != 0; }
static inline uint32_t Btn_PD0(void) { return (GPIO_PORTD_DATA_R & BTN_PD0) != 0; }
static inline uint32_t Btn_PD1(void) { return (GPIO_PORTD_DATA_R & BTN_PD1) != 0; }

/* ========================================================= */
/* INPUT TASK (Pri: 3)                                       */
/* ========================================================= */
void InputTask(void *pv)
{
    bool prev[7] = {0};
    bool cur[7] = {0};
    uint32_t pressTick[4] = {0};

    while (1)
    {
        cur[0] = Btn_PF4(); cur[1] = Btn_PE0(); cur[2] = Btn_PE1(); cur[3] = Btn_PB0();
        cur[4] = Btn_PB1(); cur[5] = Btn_PD0(); cur[6] = Btn_PD1();

        GateEvent_t ev;

        /* Evaluate directional buttons for PRESS and RELEASE */
        for(int i = 0; i < 4; i++) {
            if (cur[i] && !prev[i]) {
                pressTick[i] = xTaskGetTickCount();
                ev.type = (EventType_t)(i * 2); 
                ev.durationTicks = 0;
                xQueueSend(xEventQueue, &ev, 0);
            } 
            else if (!cur[i] && prev[i]) {
                ev.type = (EventType_t)((i * 2) + 1); 
                ev.durationTicks = xTaskGetTickCount() - pressTick[i];
                xQueueSend(xEventQueue, &ev, 0);
            }
        }

        /* Evaluate Limits */
        if (cur[4] && !prev[4]) {
            hitOpenLimit = true;
            xSemaphoreGive(xLimitSemaphore);
        }
        if (cur[5] && !prev[5]) {
            hitCloseLimit = true;
            xSemaphoreGive(xLimitSemaphore);
        }

        /* Evaluate Obstacle */
        if (cur[6] && !prev[6]) {
            uint8_t dummy = 1;
            xQueueSend(xSafetyQueue, &dummy, 0);
        }

        for (int i=0; i<7; i++) prev[i] = cur[i];
        vTaskDelay(pdMS_TO_TICKS(20)); // RTOS Debounce
    }
}

/* ========================================================= */
/* SAFETY TASK (Pri: 4)                                      */
/* ========================================================= */
void SafetyTask(void *pv)
{
    uint8_t dummy;
    while (1)
    {
        if (xQueueReceive(xSafetyQueue, &dummy, portMAX_DELAY))
        {
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            
            if (gateState == CLOSING)
            {
                gateState = REVERSING;
                xSemaphoreGive(xStateMutex);

                vTaskDelay(pdMS_TO_TICKS(500));

                xSemaphoreTake(xStateMutex, portMAX_DELAY);
                gateState = STOPPED_MIDWAY;
            }
            xSemaphoreGive(xStateMutex);
        }
    }
}

/* ========================================================= */
/* GATE FSM TASK (Pri: 2)                                    */
/* ========================================================= */
void GateTask(void *pv)
{
    bool drvOp = false, drvCl = false, secOp = false, secCl = false;

    while (1)
    {
        GateEvent_t ev;
        bool stateChanged = false;
        bool manualStopRequested = false;

        if (xQueueReceive(xEventQueue, &ev, pdMS_TO_TICKS(10)))
        {
            xSemaphoreTake(xStateMutex, portMAX_DELAY);

            switch(ev.type) {
                case EV_DRV_OPEN_PRESS:   drvOp = true; stateChanged = true; break;
                case EV_DRV_CLOSE_PRESS:  drvCl = true; stateChanged = true; break;
                case EV_SEC_OPEN_PRESS:   secOp = true; stateChanged = true; break;
                case EV_SEC_CLOSE_PRESS:  secCl = true; stateChanged = true; break;
                
                case EV_DRV_OPEN_RELEASE:
                    drvOp = false; stateChanged = true;
                    if (ev.durationTicks >= pdMS_TO_TICKS(500)) manualStopRequested = true;
                    break;
                case EV_DRV_CLOSE_RELEASE:
                    drvCl = false; stateChanged = true;
                    if (ev.durationTicks >= pdMS_TO_TICKS(500)) manualStopRequested = true;
                    break;
                case EV_SEC_OPEN_RELEASE:
                    secOp = false; stateChanged = true;
                    if (ev.durationTicks >= pdMS_TO_TICKS(500)) manualStopRequested = true;
                    break;
                case EV_SEC_CLOSE_RELEASE:
                    secCl = false; stateChanged = true;
                    if (ev.durationTicks >= pdMS_TO_TICKS(500)) manualStopRequested = true;
                    break;
            }

            /* Priority & Conflict Resolution */
            if (stateChanged) {
                if (secOp && secCl) gateState = STOPPED_MIDWAY;
                else if (secOp) gateState = OPENING;
                else if (secCl) gateState = CLOSING;
                else if (drvOp && drvCl) gateState = STOPPED_MIDWAY;
                else if (drvOp) gateState = OPENING;
                else if (drvCl) gateState = CLOSING;
                else if (manualStopRequested) gateState = STOPPED_MIDWAY;
            }
            xSemaphoreGive(xStateMutex);
        }

        if (xSemaphoreTake(xLimitSemaphore, 0)) {
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            if (hitOpenLimit && gateState == OPENING) gateState = IDLE_OPEN;
            if (hitCloseLimit && gateState == CLOSING) gateState = IDLE_CLOSED;
            
            hitOpenLimit = false;
            hitCloseLimit = false;
            xSemaphoreGive(xStateMutex);
        }
    }
}

/* ========================================================= */
/* LED TASK (Pri: 2)                                         */
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
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ========================================================= */
/* MAIN                                                      */
/* ========================================================= */
int main(void)
{
    GPIO_Init();

    xEventQueue     = xQueueCreate(10, sizeof(GateEvent_t));
    xSafetyQueue    = xQueueCreate(5, sizeof(uint8_t));
    xStateMutex     = xSemaphoreCreateMutex();
    xLimitSemaphore = xSemaphoreCreateBinary();

    gateState = IDLE_CLOSED;

    xTaskCreate(InputTask,  "Input",  256, NULL, 3, NULL);
    xTaskCreate(GateTask,   "Gate",   256, NULL, 2, NULL);
    xTaskCreate(LEDTask,    "LED",    128, NULL, 2, NULL);
    xTaskCreate(SafetyTask, "Safety", 256, NULL, 4, NULL);

    vTaskStartScheduler();

    while(1);
}