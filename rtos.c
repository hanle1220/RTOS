// RTOS Framework - Fall 2021
// J Losh

// Student Name:
// Han Le

// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PA2
// Orange: PA3
// Yellow: PA4
// Green:  PE0
// PBs on these pins
// PB0:    PC4
// PB1:    PC5
// PB2:    PC6
// PB3:    PC7
// PB4:    PD6
// PB5:    PD7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// Memory Protection Unit (MPU):
// Regions to allow 32 1kiB SRAM access (RW or none)
// Region to allow peripheral access (RW)
// Region to allow flash access (XR)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"
#include "gpio.h"
#include "string.h"
#include "mpu.h"
#include "clock.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // off-board orange LED

#define SW0 PORTC,4
#define SW1 PORTC,5
#define SW2 PORTC,6
#define SW3 PORTC,7
#define SW4 PORTD,6
#define SW5 PORTD,7
#define BLUE_LED_MASK 4
#define RED_LED_MASK 4
#define GREEN_LED_MASK 1
#define YELLOW_LED_MASK 16
#define ORANGE_LED_MASK 8

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
extern void setPSP(uint32_t stack);
extern uint32_t* getPSP(void);
extern uint32_t* getMSP(void);
extern void setASPBit(void);
extern void pushR4toR11toPSP(void);
extern void popR4toR11fromPSP(void);
extern void pushToPSP(uint32_t);
extern void setLR(uint32_t);
extern uint32_t getR0fromPSP();
extern void setTMPL(void);

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];
#define keyPressed 1
#define keyReleased 2
#define flashReq 3
#define resource 4

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore
#define STATE_SUSPENDED  5 // has removed

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // original top of stack
    void *sp;                      // current stack pointer
    int8_t priority;               // 0=highest to 15=lowest
    uint32_t ticks;                // ticks until sleep complete
    uint32_t srd;                  // MPU subregion disable bits
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    uint32_t t;
} tcb[MAX_TASKS];

#define SCHED_PRIO  1
#define SCHED_RR    2
uint8_t schedMode = SCHED_PRIO;
bool preemption = true;
uint32_t startTime;
uint32_t time[MAX_TASKS];
static int systickCount = 0;
uint8_t taskPriority[MAX_TASKS];

typedef struct _ipcs
{
    char name[16];
    uint16_t count;
    uint16_t size;
    uint32_t queueWait[MAX_QUEUE_SIZE];
} ipcs;

struct _ps
{
    char name[16];
    uint8_t state;
    uint32_t pid;
    uint32_t time;
} ps[MAX_TASKS];

enum CASE {YIELD=1, SLEEP, WAIT, POST, SCHED, PREEMPT, PIDOF, KILL, REBOOT, RESTART, IPCS, PS};
//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }

    //init systick
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;
    NVIC_ST_RELOAD_R |= 39999;  // 40MHz/(1/10ms)-1 = 39999
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    }
    return task;
}

void priorityTask()
{
    uint8_t prio;
    uint8_t task = 0, k = 0;
    for(prio = 0; prio <= 7; prio++)
    {
        for(task = 0; task < taskCount; task++)
        {
            if(tcb[task].priority == prio)
            {
                taskPriority[k] = task;
                k++;
            }
        }
    }
}

int prioScheduler()
{
    static uint8_t i = 0xFF;
    bool found = 0;
    while(!found)
    {
        i++;
        if(i >= taskCount)
            i = 0;
        found = (tcb[taskPriority[i-1]].state == STATE_READY || tcb[taskPriority[i-1]].state == STATE_UNRUN);
        if(found)
            break;  //exit loop
    }
    return taskPriority[i-1];
}

uint32_t getSVCnum()
{
    uint32_t* R0 = getPSP();
    return *(uint32_t*)(*(R0+6)-2) & 0xFF;
}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0, k = 0, n = 0;
    uint32_t* baseAddr =(uint32_t *)0x20001000;
    bool found = false;
    // REQUIRED:
    // store the thread name
    // allocate stack space and store top of stack in sp and spInit
    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            for (k = 0; name[k] != '\0'; ++k)   {tcb[i].name[k] = name[k];}   //store name
            tcb[i].priority = priority;

            if(i==0) tcb[i].sp = (void*)((uint32_t)(baseAddr) + stackBytes );
            else   tcb[i].sp = (void*)((uint32_t)(tcb[i-1].spInit) + stackBytes);
            tcb[i].spInit = tcb[i].sp ;
            while(n != (stackBytes/1024))
            {
                tcb[i].srd |= 1<<(n+4+i);
                n++;
            }
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
    uint8_t i;
    for(i = 0; i < taskCount; i++)
        if(tcb[i].pid == fn)
        {
            tcb[i].sp = tcb[i].spInit;
            tcb[i].state = STATE_UNRUN;
            break;
        }
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(_fn fn)
{
    uint8_t i, j;
    for(i = 0; i < taskCount; i++)
        if(tcb[i].pid == fn)
        {
            semaphore* sema = (semaphore*)tcb[i].semaphore;
            if(sema != 0)
            {
                sema->count = 0;
                sema->queueSize = 0;
                for(j=0; j<MAX_QUEUE_SIZE; j++)
                {
                    sema->processQueue[j] = sema->processQueue[j+1];
                    sema->processQueue[j] = 0;
                }
            }
         tcb[i].state = STATE_SUSPENDED;
         break;
        }
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    uint8_t i;
    for(i = 0; i < taskCount; i++)
        if(tcb[i].pid == fn)
        {
            tcb[i].priority = priority;
            break;
        }
}

bool createSemaphore(uint8_t semaphore, uint8_t count)
{
    bool ok = (semaphore < MAX_SEMAPHORES);
    {
        semaphores[semaphore].count = count;
    }
    return ok;
}

// REQUIRED: modify this function to start the operating system
// by calling scheduler, setting PSP, ASP bit, and PC
void startRtos()
{
    priorityTask();

    if(schedMode == SCHED_PRIO)   taskCurrent = prioScheduler();
    else  taskCurrent = rtosScheduler();

    startTime = TIMER1_TAV_R;

    setPSP((uint32_t)tcb[taskCurrent].sp);
    setASPBit();
    MPUOn();
    enableMPU();
    setSRDBit(tcb[taskCurrent].srd);
    //setTMPL();

    _fn fn = (_fn) tcb[taskCurrent].pid;
    tcb[taskCurrent].state = STATE_READY;
    fn();
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
    __asm("             SVC #1");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    __asm("             SVC #2");
}

// REQUIRED: modify this function to wait a semaphore using pendsv
void wait(int8_t semaphore)
{
    __asm("             SVC #3");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    __asm("             SVC #4");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t i;
    for(i=0; i<taskCount; i++)
    {
        if(tcb[i].state == STATE_DELAYED)
        {
            if(tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;
                break;
            }
            tcb[i].ticks--;
        }
    }

    systickCount++;
    if(systickCount == 2000)
    {
        systickCount = 0;
        for(i=0; i<taskCount; i++)
        {
            time[i] = tcb[i].t;
            tcb[i].t = 0;
        }
    }

    if(preemption && tcb[taskCurrent].state == STATE_READY)
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    //if MPU DERR or IERR bits are set, clear them
    if(NVIC_FAULT_STAT_R & NVIC_FAULT_STAT_DERR)
    {
        NVIC_FAULT_STAT_R &= ~(NVIC_FAULT_STAT_DERR);
        putsUart0("called from MPU.\n");
    }
    if(NVIC_FAULT_STAT_R & NVIC_FAULT_STAT_IERR)
    {
        NVIC_FAULT_STAT_R &= ~(NVIC_FAULT_STAT_IERR);
        putsUart0("called from MPU.\n");
    }

    pushR4toR11toPSP();
    uint32_t* psp = getPSP();
    tcb[taskCurrent].sp = (void*)psp;

    uint32_t stopTime = TIMER1_TAV_R;
    tcb[taskCurrent].t = (stopTime - startTime);

    if(schedMode == SCHED_PRIO)
        taskCurrent = prioScheduler();
    else
        taskCurrent = rtosScheduler();

    startTime = TIMER1_TAV_R;

    setSRDBit(tcb[taskCurrent].srd);

    if(tcb[taskCurrent].state == STATE_READY)
    {
        setPSP((uint32_t)tcb[taskCurrent].sp);
        popR4toR11fromPSP();
    }
    else //unrun
    {
        tcb[taskCurrent].state = STATE_READY;
        setPSP((uint32_t)tcb[taskCurrent].spInit);
        pushToPSP(0x61000000);    //push xPSR -> R0 on stack
        pushToPSP((uint32_t)tcb[taskCurrent].pid);    //PC
        pushToPSP(0xFFFFFFFD);    //LR
        pushToPSP(0);    //R12
        pushToPSP(0);    //R3
        pushToPSP(0);    //R2
        pushToPSP(0);    //R1
        pushToPSP(0);    //R0
    }
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    uint32_t* R0 = getPSP();
    uint32_t N = getSVCnum();
    switch (N)
    {
        case YIELD:
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;   //set pendsv bit
            break;
        case SLEEP:
            tcb[taskCurrent].ticks = *R0;    //record timeout in ms -> tcb
            tcb[taskCurrent].state = STATE_DELAYED;     //state <- DELAYED
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;   //set pendsv bit
            break;
        case WAIT:
        {
            uint32_t i = *R0;
            if(semaphores[i].count > 0)
            {
                semaphores[i].count--;
            }
            else
            {
                //add process to semaphore i queue,inc queue count
                semaphores[i].processQueue[semaphores[i].queueSize++] = taskCurrent;
                tcb[taskCurrent].semaphore = (void *)(semaphores+i);  //record sema i in tcb
                tcb[taskCurrent].state = STATE_BLOCKED;     //state <- BLOCKED
                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;   //set pendsv
            }
            break;
        }
        case POST:
        {
            uint32_t i = *R0;
            semaphores[i].count++;  //inc queue count
            if(semaphores[i].queueSize > 0)
            {
                //make next task in list ready
                tcb[semaphores[i].processQueue[0]].state = STATE_READY;
                //delete task from queue, queue.count--, dec sema.count--
                uint8_t n;
                for(n=0; n<semaphores[i].queueSize; n++)
                    semaphores[i].processQueue[n] = semaphores[i].processQueue[n+1];
                semaphores[i].queueSize--;
                semaphores[i].count--;
            }
            break;
        }
        case SCHED:
            if((bool)*R0) schedMode = SCHED_PRIO;
            else schedMode = SCHED_RR;
            break;
        case PREEMPT:
            if((bool)*R0) preemption = 1;
            else preemption = 0;
            break;
        case PIDOF:
        {
            uint32_t* pid = (uint32_t*)*R0;
            char* strName = (char*) *(R0+1);
            uint8_t i;
            for(i=0; i<taskCount; i++)
                if(strcompare(tcb[i].name, strName))
                {
                    *pid = (uint32_t)tcb[i].pid;
                    printHex(*pid);
                    printInt(*pid);
                    break;
                }
            break;
        }
        case KILL:
            destroyThread((_fn)*R0);
            break;
        case REBOOT:
            NVIC_APINT_R =  NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            break;
        case RESTART:
        {
            char* processName = (char*)*R0;
            uint8_t i;
            for(i=0; i < taskCount; i++)
            {
                if(strcompare(tcb[i].name, processName) && tcb[i].state == STATE_SUSPENDED)
                {
                    restartThread((_fn)tcb[i].pid);
                    break;
                }
            }
            break;
        }
        case IPCS:
        {
            uint8_t i, j;
            struct _ipcs* data = (struct _ipcs*) *R0;
            for(i=0; i<MAX_SEMAPHORES; i++)
            {
                if(i==0) continue;
                data[i].count = semaphores[i].count;
                data[i].size = semaphores[i].queueSize;
                for(j=0; j<data[i].size; j++)
                    data[i].queueWait[j] = semaphores[i].processQueue[j];
                if(i==keyPressed)   strcopy("keyPressed", data[i].name);
                if(i==keyReleased)  strcopy("keyReleased", data[i].name);
                if(i==flashReq)     strcopy("flashReq", data[i].name);
                if(i==resource)     strcopy("resource", data[i].name);
            }
            putsUart0("Name \t\t Count \t Waiting\n");
            putsUart0("--------------------------------\n");
            for(i=0; i<MAX_SEMAPHORES; i++)
            {
                if(i==0) continue;
                putsUart0(data[i].name); putsUart0("\t    ");
                printInt(data[i].count); putsUart0("\t    ");
                for(j=0; j<data[i].size; j++)
                    printInt(data[i].queueWait[j]);
            putcUart0('\n');
            }
        }
            break;
        case PS:
        {
            uint8_t i;
            struct _ps* psData = (struct _ps*) *R0;
            uint16_t* count = (uint16_t*) *(R0+1);
            for(i=0; i<taskCount; i++)
            {
                strcopy(tcb[i].name, psData[i].name);
                psData[i].pid = (uint32_t)tcb[i].pid;
                psData[i].state =  tcb[i].state;
                psData[i].time = time[i];
            }
            *count = taskCount;
            uint16_t psCount = *count;
            uint32_t total = 0;
            uint32_t cpu;
            for(i=0; i<psCount; i++)
                total += psData[i].time;
            putsUart0("Pid \t Name \t\tState \t\t% CPU Time\n");
            putsUart0("--------------------------------------------------");
            for(i=0; i<psCount; i++)
            {
                cpu = psData[i].time * 100 * 100 / total;
                putsUart0("\n");
                printInt(psData[i].pid);    putsUart0("\t");
                printString(psData[i].name);  putsUart0("\t");
                printState(psData[i].state);putsUart0("\t\t");
                printFloat(cpu);
            }
            break;
        }
    }
}

// REQUIRED: code this function
void mpuFaultIsr()
{
    putsUart0("\nMPU fault in process ");
    printInt(taskCurrent);

    // provide value of PSP, MSP, and MPU fault flags (in hex)
    uint32_t* psp = getPSP();
    uint32_t* msp = getMSP();

    putsUart0("PSP\t");  printHex((uint32_t)getPSP());
    putsUart0("MSP\t");   printHex((uint32_t)getMSP());
    putsUart0("MFault Flags\t");    printHex(NVIC_FAULT_STAT_R & 0x000000FF);

    //display the process stack dump
    putsUart0("R0\t");  printHex(*(psp));  //R0
    putsUart0("R1\t");  printHex(*(psp+1));  //R1
    putsUart0("R2\t");  printHex(*(psp+2));  //R2
    putsUart0("R3\t");  printHex(*(psp+3));  //R3
    putsUart0("R12\t"); printHex(*(psp+4));  //R12
    putsUart0("LR\t");  printHex(*(psp+5));  //LR
    putsUart0("PC\t");  printHex(*(psp+6));  //PC
    putsUart0("xPSR\t");printHex(*(psp+7));  //xPSR

    //clear MPU fault pending bit and trigger pendsv ISR call
    NVIC_SYS_HND_CTRL_R &= ~(NVIC_SYS_HND_CTRL_MEMP);
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // trigger pendsv
}

// REQUIRED: code this function
void hardFaultIsr()
{
    putsUart0("\nHard fault in process ");
    printInt(taskCurrent);
    putsUart0("PSP\t"); printHex(*(getPSP()));
    putsUart0("MSP\t"); printHex(*(getMSP()));
    putsUart0("Hard Fault Flags\t");
    printHex(NVIC_HFAULT_STAT_R);
    while(1)
    {

    }
}

// REQUIRED: code this function
void busFaultIsr()
{
    putsUart0("\nBus fault in process ");
    printInt(taskCurrent);
    while(1)
    {

    }
}

// REQUIRED: code this function
void usageFaultIsr()
{
    putsUart0("\nUsage fault in process ");
    printInt(taskCurrent);
    while(1)
    {

    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons
void initHw()
{
    initSystemClockTo40Mhz();
    initUart0();
    enableISR();

    enablePort(PORTA);
    enablePort(PORTE);
    enablePort(PORTF);
    enablePort(PORTC);
    enablePort(PORTD);
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    GPIO_PORTA_DIR_R |= RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;
    GPIO_PORTE_DIR_R |= GREEN_LED_MASK;
    GPIO_PORTF_DIR_R |= BLUE_LED_MASK;

    GPIO_PORTA_DEN_R |= RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;
    GPIO_PORTE_DEN_R |= GREEN_LED_MASK;
    GPIO_PORTF_DEN_R |= BLUE_LED_MASK;

    selectPinDigitalInput(SW0);
    enablePinPullup(SW0);

    selectPinDigitalInput(SW1);
    enablePinPullup(SW1);

    selectPinDigitalInput(SW2);
    enablePinPullup(SW2);

    selectPinDigitalInput(SW3);
    enablePinPullup(SW3);

    selectPinDigitalInput(SW4);
    enablePinPullup(SW4);

    setPinCommitControl(SW5);
    selectPinDigitalInput(SW5);
    enablePinPullup(SW5);
}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t total = 0;
    if(!getPinValue(SW0)) total += 1;
    if(!getPinValue(SW1)) total += 2;
    if(!getPinValue(SW2)) total += 4;
    if(!getPinValue(SW3)) total += 8;
    if(!getPinValue(SW4)) total += 16;
    if(!getPinValue(SW5)) total += 32;

    return total;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

void initTimer()
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAMR_R = TIMER_TAMR_TACDIR;               // timer counts up
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

void sched(bool prio_on)
{
    __asm("             SVC #5");
}

void preempt(bool on)
{
    __asm("             SVC #6");
}

void getPid(uint32_t* pid, char name[])
{
    __asm("             SVC #7");
}

void killPid(uint32_t* pid)
{
    __asm("             SVC #8");
}

void reboot()
{
    __asm("             SVC #9");
}

void restartTask(char processName[])
{
    __asm("             SVC #10");
}

void getIpcsData(struct _ipcs* data)
{
    __asm("             SVC #11");
}

void getPsData(struct _ps* psData, uint16_t* count)
{
    __asm("             SVC #12");
}
// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    USER_DATA data;
    while (true)
    {
        putsUart0("\nmsh> ");
        getsUart0(&data);
        parseFields(&data);

        if(strcompare("reboot", getFieldString(&data, 0)) == 1)
        {
            putsUart0("Rebooting...\n");
            reboot();
        }
        else if(strcompare("sched", getFieldString(&data, 0)) == 1)
        {
            if(strcompare("prio", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Priority Scheduling!\n");
                sched(true);
            }
            else if(strcompare("rr", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Round-Robin Scheduling!\n");
                sched(false);
            }
            else
                putsUart0("Invalid!\n");
        }
        else if(strcompare("preempt", getFieldString(&data, 0)) == 1)
        {
            if(strcompare("on", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Turns Preemption On!\n");
                preempt(true);
            }
            else if(strcompare("off", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Turns Preemption Off!\n");
                preempt(false);
            }
            else
                putsUart0("Invalid!\n");
        }
        else if(strcompare("pidof", getFieldString(&data, 0)) == 1)
        {
            uint32_t pid = 0;
            putsUart0("PID of ");
            putsUart0(getFieldString(&data, 1));
            putcUart0('\n');
            getPid(&pid, getFieldString(&data, 1));
        }
        else if(strcompare("kill", getFieldString(&data, 0)) == 1)
        {
            killPid((uint32_t*)getFieldInteger(&data, 1));
        }
        else if(strcompare("run", getFieldString(&data, 0)) == 1)
        {
            restartTask(getFieldString(&data, 1));
        }
        else if(strcompare("ipcs", getFieldString(&data, 0)) == 1)
        {
            struct _ipcs data[MAX_SEMAPHORES];
            getIpcsData(data);
        }
        else if(strcompare("ps", getFieldString(&data, 0)) == 1)
        {
            struct _ps psData[MAX_TASKS];
            uint16_t count = 0;
            getPsData(psData, &count);
        }
        else
            putsUart0("Invalid!\n");
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    initTimer();
    initRtos();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    createSemaphore(keyPressed, 1);
    createSemaphore(keyReleased, 0);
    createSemaphore(flashReq, 5);
    createSemaphore(resource, 1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7, 1024);              //0

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 6, 1024);    //1
    ok &= createThread(flash4Hz, "Flash4Hz", 4, 1024);      //2
    ok &= createThread(oneshot, "OneShot", 2, 1024);        //3
    ok &= createThread(readKeys, "ReadKeys", 6, 1024);      //4
    ok &= createThread(debounce, "Debounce", 6, 1024);      //5
    ok &= createThread(important, "Important", 0, 1024);    //6
    ok &= createThread(uncooperative, "Uncoop", 6, 1024);   //7
    ok &= createThread(errant, "Errant", 6, 1024);          //8
    ok &= createThread(shell, "Shell", 6, 1024);            //9

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;
    return 0;
}
