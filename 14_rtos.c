// RTOS Framework - Spring 2019
// Subedi_Bishrut
// Advising professor: Dr. Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Green:  PE2
// Yellow: PE3
// Orange: PE4
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <string.h>

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // off-board orange LED

#define PB0     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // swicth 0
#define PB1     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // switch 1
#define PB2     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // switch 2
#define PB3     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) // switch 3
#define PB4     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) // switch 4


//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//---------------------------------
#define YIELD 100
#define SLEEP 101
#define WAIT 102
#define POST 103
#define KILL 104
//---------------------------------
uint32_t stackPointer;
uint32_t svCallNo; // declare as pointer
uint32_t Stack_pointer;
uint32_t Program_counter;
uint32_t *Program_counter_1;
uint32_t *stp;
uint32_t RR0;
uint32_t RR1;
uint32_t RR2;
uint32_t totalTime = 0;

bool preemption = false; // if preemption on with other falseISR
bool priority_1 = true; /// SET PRIORITY SCHEDULING ON FOR START
bool priorityInheritance = true; // SET PRIORITY INHERITANCE
uint32_t timeCounter = 0;
//--------------------------------
// VARIABLES FROM 5314
uint8_t max_Char = 80;
char str[81]; // warning declared but never used
uint8_t count = 0;
uint8_t counter;
int argc;
uint8_t pos []; //can be char/int
char type[];
char str_1[82];
int arg1, arg2;
int number;
int number_1;
int arg;
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5

struct semaphore {
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE];  // store task index here
    char semaphoreName[16];                 // for storing semaphore Name
    uint8_t user;
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource , *pointerCopy;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0; // index of last dispatched task
uint8_t taskCount = 0; // total number of valid tasks

uint32_t stack[MAX_TASKS][256]; // 1024 byte stack for each thread
uint32_t temp_sp; // tcb[taskCurrent].sp for checking // aafai le create gareko

struct _tcb {
    uint8_t state; // see STATE_ values above
    void *pid; // used to uniquely identify thread
    void *sp; // location of stack pointer for thread
    int8_t priority; // -8=highest to 7=lowest
    int8_t currentPriority; // used for priority inheritance
    uint32_t ticks; // ticks until sleep complete
    char name[16]; // name of task used in ps command
    void *semaphore; // pointer to the semaphore that is blocking the thread
    uint8_t skipCounter;
    uint32_t time;  // required to store the time for cpu percentage
    float filterTime;
    uint32_t percentage; // for storing percentage of cpu time
    void *pid_1; // copy of pid for storing pid
} tcb[MAX_TASKS];

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit() {
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++) {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
    // REQUIRED: initialize systick for 1ms system timer
        NVIC_ST_CTRL_R = 0;
        NVIC_ST_RELOAD_R = 40000;
        NVIC_ST_CURRENT_R = 0;
        NVIC_ST_CTRL_R = 0x7; // bit 0, bit 1 and bit 2.
}

// REQUIRED: Implement prioritization to 8 levels

int rtosScheduler() {                       // question: How rtos scheduler schedules new task if task if always 0.
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok) {
        task++;                             // task 9 pachi feri 0 mai jancha, How 0 is idle ?
        if (task >= MAX_TASKS)
            task = 0;
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN); // what is stored in ok ?

        if ((ok == true && priority_1 == true)){

            if (tcb[task].priority + 8 == tcb[task].skipCounter){   // is it taskcurrent or task
                tcb[task].skipCounter = 0;
                    }
            else
            {
                tcb[task].skipCounter = tcb[task].skipCounter + 1; // task or taskCurrent
                ok = false;
            }
        }
    }

    uint32_t localtime = 0;
    localtime = WTIMER5_TAV_R / 40; // normalizing
    tcb[taskCurrent].time = tcb[taskCurrent].time + localtime ; // reading timer value for cpu percentage

    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER5_TAV_R = 0;
    WTIMER5_CTL_R |=TIMER_CTL_TAEN;

    return task;
    }


uint32_t getSP()                            //takes no argument and returns stack pointer
{
    __asm("            MOV  R0, SP");       // gives offset but compiles
    __asm("            BX  LR");            // like return 0
}

// takes stack pointer and returns nothing

void setSP(uint32_t x)                      // check if it has to be uint32_t stackPointer or any variable
{
    __asm("            MOV  SP, R0");       // x is placed in R0.
    __asm("            BX  LR");
}

void rtosStart() {
    // REQUIRED: add code to call the first task to be run
    _fn fn;
    taskCurrent = rtosScheduler();          // task current, how does it change? rtosScheduler returns task, which is always 0 ?

    stackPointer = getSP();                 // getting system stack pointer
                                            // this is the system start pointer at the start of the program
    setSP(tcb[taskCurrent].sp);             // setting stack pointer to point to current task
    tcb[taskCurrent].state = STATE_READY;   // yo state kaha kaha change or k le change gareko cha check garne
    fn = tcb[taskCurrent].pid;              // yaha & chaincha ki chaindaina // fn pointer lai pid fn type pointer banako or ?

    WTIMER5_CTL_R |= TIMER_CTL_TAEN;        // starting the timer
   // WTIMER5_TAV_R = 0;                      // clearing the timer
    (*fn)(); // CHECK HERE DOES IT HAVE TO BE POINTER


    // Add code to initialize the SP with tcb[task_current].sp; // what does it mean ?

}

bool createThread(_fn fn, char name[], int priority) {
    bool ok = false;
    uint8_t i = 0;
    uint8_t j = 0;
    bool found = false;
    // REQUIRED: store the thread name
    // add task if room in task list
    if (taskCount < MAX_TASKS) {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS)) {
            found = (tcb[i++].pid == fn);
        }
        if (!found) {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {
                i++;
            }
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].sp = &stack[i][255]; // top of i stack
            tcb[i].priority = priority; // ADDED
            tcb[i].currentPriority = priority;
            tcb[i].skipCounter = 0;     // ADDED
            tcb[i].time = 0;
            tcb[i].filterTime = 0;
            tcb[i].percentage = 0;
            tcb[i].pid_1 = fn; // copy of pid
            for ( j = 0 ; j < (strlen(name)) ; j++ ){
                tcb[i].name[j] = name[j];
            }
                     // ADDED
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting

void destroyThread(_fn fn) {
    // has svc something
    __asm("            SVC  #104"); // destroy thread
}

// REQUIRED: modify this function to set a thread priority

void setThreadPriority(_fn fn, uint8_t priority) {
    uint8_t i;
    for (i = 0 ; i < taskCount ; i++){
        if (tcb[i].pid == fn){
            tcb[i].priority = priority;
        }
    }
}

struct semaphore* createSemaphore(uint8_t count, char name[]) {
    struct semaphore *pSemaphore = 0;                   //why 0 ? 0 points to stack-pointer(SP), 4 is program counter (PC)
    int k = 0;
    if (semaphoreCount < MAX_SEMAPHORES) {
        pSemaphore = &semaphores[semaphoreCount++];     // euta particular structure ko row lai point garne
        pSemaphore->count = count;                      //psemaphore points to struct semaphore, ani struct semaphore bhitra ko count change gareko, lhs le point

        for (k = 0; k < (strlen(name)) ; k++){
        pSemaphore->semaphoreName[k] =  name[k];
        }
    }
    return pSemaphore;                                  // returns the value of count-> count
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function

void yield() {
    // call SVC 100 here // SVC causes software interrupt
    __asm("            SVC  #100");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)

void sleep(uint32_t tick) {
    __asm("            SVC  #101");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv

void wait(struct semaphore *pSemaphore) {
    __asm("            SVC  #102");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv

void post(struct semaphore *pSemaphore) {
    __asm("            SVC  #103");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch

void systickIsr() {
    uint8_t i = 0;
   // uint32_t timeCounter;
    for (i = 0; i < 10; i++) {
        if (tcb[i].state == STATE_DELAYED) {
            tcb[i].ticks--;

            if (tcb[i].ticks == 0) {
                tcb[i].state = STATE_READY;
            }
        }
    }
    timeCounter++;
    if (timeCounter == 1000){ // calculating time 100ms

    for( i = 0 ; i < taskCount; i++ ){  //filter time for every millisecond
        tcb[i].filterTime = (99 * tcb[i].filterTime + tcb[i].time) / 100 ; // ticks is propotional to time
        }
    timeCounter = 0; //resetting the timeCounter

    cpuTime();       // Calculating the percentage of CPU utilization

    }

    //cpuTime();    // cpu time updates itself after new filter time

    if (preemption == true){
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }

}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently

void pendSvIsr() {

    __asm("            PUSH  {R4-R7}"); // may be only 7
    tcb[taskCurrent].sp = getSP();      // saving current thread SP to its SP
    setSP(stackPointer);                // setting SP to system stack pointer so its doesnot push things on current stack

    taskCurrent = rtosScheduler();      // get scheduler for next task
    // clearing timer value for reading next task
    //misses idle here
    if (tcb[taskCurrent].state == STATE_READY) {
        setSP(tcb[taskCurrent].sp); // setting stack pointer to stack pointer of that thread
        __asm("            POP  {R4-R7}"); // pop the register in that new taskCurrent thread
    } else {
        tcb[taskCurrent].state = STATE_READY; // had double equals DO I NEED TO POP XPSR, IF NOT IMPORTANT WHERE DOES PC START 255 OR 254
        stack[taskCurrent][255] = 0x41000000; //XPSR
        stack[taskCurrent][254] = tcb[taskCurrent].pid; //PC
        stack[taskCurrent][253] = 0xFFFFFFF9; // dont matter
        stack[taskCurrent][252] = 12; //R12
        stack[taskCurrent][251] = 20; //R3
        stack[taskCurrent][250] = 3; //R2
        stack[taskCurrent][249] = 2; //R1
        stack[taskCurrent][248] = 1; //PC
        stack[taskCurrent][247] = 0xfffffff9; //LR (PC not R1)
        stack[taskCurrent][246] = 5; //R3
        stack[taskCurrent][245] = 7; //R7
        stack[taskCurrent][244] = 6; //R6
        stack[taskCurrent][243] = 5; //R5
        stack[taskCurrent][242] = 4; //R4

        tcb[taskCurrent].sp = &(stack[taskCurrent][242]);
        setSP(tcb[taskCurrent].sp); // how do i know tcb[taskCurrent].sp points to bottom
        // does it always points to tcb[255].sp
        __asm("            POP  {R4-R7}"); // Popping new thread value

    }

}

uint32_t getR0() {
} //NO BX LR

uint32_t getR1() {
    __asm("            MOV  R0, R1");
    // NO BX LR
}

uint32_t getR2() {
    __asm("            MOV  R0, R2");
    // NO BX LR
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives

void svCallIsr() {
    RR0 = getR0();
    RR1 = getR1();
    RR2 = getR2();

    struct semaphore *pointerRR0, *temp ; // address of semaphore is passed to struct semaphore pointer
    uint32_t *queuecount;
    uint8_t i, j, k , l ; // for step 8
    uint8_t taskToKill;

    switch (getSvc()) {

        case YIELD: // YIELD //100
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // setting pendsv bit to 1 BIT 28 in NVIC int control register
            break;

        case SLEEP: // SLEEP //101
            tcb[taskCurrent].ticks = RR0;
            tcb[taskCurrent].state = STATE_DELAYED;
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // setting pendsv bit
            break;

        case WAIT: // 102 WAIT  step 7. (wait *psemaphore) // doesn't return until things  waiting is actually there
                pointerRR0 = RR0;                               
            if ((pointerRR0->count) > 0) {
                pointerRR0->count = (pointerRR0->count) - 1; // *(pointerRR0)-- 5 to 0
               // if ( priorityInheritance = true){           // storing the semaphore user
                    pointerRR0->user = taskCurrent;           // semaphore LASTUSER
                    pointerCopy = RR0;                          // no need
                }
            else {


                tcb[taskCurrent].semaphore = pointerRR0; //RR0; why this line ? pachi access garna parcha

                if ((priorityInheritance == true)){
setThreadPriority(tcb[taskCurrent].pid , (minimumPriority(tcb[taskCurrent].priority,tcb[pointerRR0->user].priority)));
                }
                // add yourself to the queue
                tcb[taskCurrent].state = STATE_BLOCKED;
                pointerRR0->processQueue[(pointerRR0->queueSize)] = taskCurrent; // putting yourself in end of queue
                pointerRR0->queueSize = (pointerRR0->queueSize) + 1; // incrementing queue count


                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; // pendsv not switching task
            }
           // setThreadPriority(tcb[pointerRR0->count].priority) = tcb[taskCurrent].priority
            break;

        case POST: //103 POST step 8. (post(semaphore *s))
            // post increments semaphore values by 1.
            pointerRR0 = RR0;
            pointerRR0->count = (pointerRR0->count) + 1; // incrementing count // put something back on the pot
            // reset priority from current priority of last task need to access pointer from wait
            //tcb[pointerCopy->user].priority = tcb[pointerCopy->user].currentPriority; // taskCurrent restoring original priority
            tcb[taskCurrent].priority = tcb[taskCurrent].currentPriority;

            if ((pointerRR0->count) == 1) {  // IS THIS BREAKING THE CODE ?
                if ((pointerRR0->queueSize) > 0) { //WHICH COUNT IS SUPPOSED TO BE HERE ?
                    tcb[(pointerRR0->processQueue[0])].state = STATE_READY; //like tcb[taskCurrent].state
                    pointerRR0->count = (pointerRR0->count)-1;
                    for (i=0 ; i < ((pointerRR0->queueSize)); i++ )        // do line 386 and 388 contradict
                    {
                        pointerRR0->processQueue[i] = (pointerRR0->processQueue[i+1]);
                    }
                    pointerRR0->queueSize = (pointerRR0->queueSize) - 1; // post decrement it

                }
                //optional
                    //NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            }

            break;

        case KILL: //DESTROY THREAD

                for (i = 0; i < taskCount ; i++){
                    if (tcb[i].pid == RR0){ // fn = RR0
                        taskToKill = i;         // finding task to kill
                    }
                }

                if (tcb[taskToKill].state == STATE_BLOCKED) {

                for (i = 0; i < semaphoreCount; i++ ){
                   for (j = 0 ; j < (semaphores[i].queueSize) ; j++  ){
                   if(taskToKill == semaphores[i].processQueue[j]){        // finding taskto kill in queue
                       semaphores[i].queueSize = semaphores[i].queueSize - 1; // decrement queue size
                       for (k = j ; k < semaphores[i].queueSize ; k++ ){
                       semaphores[i].processQueue[k] = semaphores[i].processQueue[k+1]; // moving thing up the queue
                                  }
                             }
                         }
                    }
                }
                else {      // if not blocked case
                        for (i = 0; i < semaphoreCount; i++ ){
                // checking if the task is using the semaphore and posting
                     temp = &semaphores[i];
                     if (tcb[taskToKill].semaphore == temp){ // comparing two address
                         semaphores[i].count = semaphores[i].count + 1;
                         if((semaphores[i].count) == 1 ){
                             if((semaphores[i].queueSize) > 0) {
                                 tcb[semaphores[i].processQueue[0]].state = STATE_READY;
                                 semaphores[i].count = semaphores[i].count - 1;
                                 for (l = 0 ; l < ((semaphores[i].queueSize)); l++ )        // do line 386 and 388 contradict
                                     {
                                     semaphores[i].processQueue[l] = (semaphores[i].processQueue[l+1]);
                                     }
                                 semaphores[i].queueSize = semaphores[i].queueSize - 1;
                                 }
                             }
                         }
                    }
                }

                // TASK KILLED HERE
               tcb[taskToKill].state = STATE_INVALID;
               tcb[taskToKill].pid = 0;
               taskCount = taskCount - 1 ; //decreasing taskcount after creating the thread

               break;
    }

}

uint32_t getSvc() {
    stp = getSP();
    stp = stp + 16; // previous added 10.  pointing at pc value 40 move le, 10*4; not 24, pushed 6(now 14) thing prev 12
    Program_counter_1 = *(stp) - 2;
    Program_counter = *(Program_counter_1); // yeslai 4b0 lai dereference garna cha, pointer or not
    Program_counter = (Program_counter & 0x000000FF); // extracting 100 value
    // *Program_counter_1 = *program_counter + 2;
    // df value = dereference program counter after offset
    return Program_counter;

}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware

void initHw() {
    // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
    //           5 pushbuttons, and uart

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    //SYSCTL_GPIOHBCTL_R = 0;

    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF; //XTRA enable D,E,C

    // ENABLE PORT F LED
    GPIO_PORTF_DIR_R = 0x04; // bits 3 is outputs, other pins are inputs
    //GPIO_PORTF_DR2R_R = 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x04; // enable LEDs and pushbuttons
    //GPIO_PORTF_PUR_R = 0x10; // enable internal pull-up for push button

    // ENABLE PORT E
    GPIO_PORTE_DIR_R = 0x1E; // GPIO PORTE 1, 2, 3 and 4 are output
    //GPIO_PORTE_DR2R_R = 0x1E; // Drive 2mA current
    GPIO_PORTE_DEN_R = 0x1E; // Enabling digital function

    // ENABLE PORT A
    //GPIO_PORTA_DIR_R = 0x7C; // GPIO PORTA 2,3,4,5 and 6 as output
    //GPIO_PORTA_DR2R_R = 0x7C;
    GPIO_PORTA_DIR_R = 0x00;
    GPIO_PORTA_DEN_R = 0x7C; // don't even need this as input by default on clock
    GPIO_PORTA_PUR_R = 0x7C; // enable internal pull up for all buttons // so press ma ground huncha

    // CONFIGURE UART0 PINS
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3; // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3; // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0; // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK; // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45; // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Initializing systick timer
    //NVIC_ST_CTRL_R = 0;
    //NVIC_ST_RELOAD_R = 40000;
    //NVIC_ST_CURRENT_R = 0;
    //NVIC_ST_CTRL_R = 0x7; // bit 0, bit 1 and bit 2.

} // END initHw

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock

void waitMicrosecond(uint32_t us) {
    // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6"); // 1
    __asm("WMS_LOOP1:   SUB  R1, #1"); // 6
    __asm("             CBZ  R1, WMS_DONE1"); // 5+1*3
    __asm("             NOP"); // 5
    __asm("             B    WMS_LOOP1"); // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1"); // 1
    __asm("             CBZ  R0, WMS_DONE0"); // 1
    __asm("             B    WMS_LOOP0"); // 1*3
    __asm("WMS_DONE0:"); // ---
    // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed

uint8_t readPbs() {
    uint8_t i, a = 0, b = 0, c = 0 , d = 0 , e = 0;
    if (PB4 == 0){ a = 16;}
    if (PB3 == 0){ b = 8;}
    if (PB2 == 0){ c = 4;}
    if (PB1 == 0){ d = 2;}
    if (PB0 == 0){ e = 1;}
    i = a + b + c + d + e; // toggling
    return i;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose

void idle() {
    while (true) {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz() {
    while (true) {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot() {
    while (true) {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
        //post(flashReq); // needed to check step 8
    }
}

void partOfLengthyFn() {
    // represent some lengthy operation
    waitMicrosecond(990); // changed from 990 to 1()
    // give another process a chance to run
    yield();
}

void lengthyFn() {
    uint16_t i;
    while (true) {
        wait(resource);
        for (i = 0; i < 5000; i++){
       // for (i = 0; i < 5000; i++) { // changed
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys() {
    uint8_t buttons;
    while (true) {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0) {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0) {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0) {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0) {
            createThread(flash4Hz, "Flash4Hz", 0);
        }
        if ((buttons & 8) != 0) {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0) {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce() {
    uint8_t count;
    while (true) {
        wait(keyPressed);
        count = 10;
        while (count != 0) {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative() {
    while (true) {
        while (readPbs() == 8) {
        }
        yield();
    }
}

void important() {
    while (true) {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

void setString(){
            count = 0;
            // REQUIRED: add processing for the shell commands through the UART here
            while (1) { //str[count] != '\0'

                        char c = getcUart0();

                        if (c != 8 && c != 13 && c >= 32) { // CHECKING BACKSPACE AND CR
                            if (c >= 65 & c <= 90) {
                                c = c + 32; // changing capital to small letter
                            }
                            // str[count++] = c; //check this logic for where bs puts character (by printing ?) put on 1 or 0; ask Dr. Losh
                            str [count] = c;
                            count++;
                            if (count == max_Char) {
                                str[count] = '\0';
                                break;
                            }
                            // break; //which one does it break (find other ways to break like flag // check to create flag on stack exchanges)
                        }

                        if (c == 8) { // back space
                            if (count > 0) {
                                count--;
                            }
                        }

                        if (c == 13) { //carriage return
                            str[count] = '\0'; // null terminator, marks end of string in C.
                            break; // where does this break go to ?
                        }

                      //  yield();

                    } //END INSIDE WHILE LOOP
                    // putsUart0(str); // checking the input string
                    //putsUart0("\r\n");
}

void shell() {
    while (true) {
                int j,k,i;
                char temp[16];
                char temp_1[16];
                char temp_2[16]; // added for ps
                uint8_t length = 0;
                char space[16] = "               "; // space for adjusting table
                char name_1[9] = "TaskName";
                //char taskName[9][8] = {"idle" , "lengthyfn" , "flash4hz" , "oneshot" , "readkeys" , "debounce" , "important" , "uncoop" , "shell"};

                //typedef void (*FunctionCallback)();
                //FunctionCallback functions[9] = {&idle , &lengthyFn , &flash4Hz , &oneshot , &readKeys, &debounce , &important , &uncooperative, &shell};

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
                setString();
                parse_string(str); // calling function parse string to do step 3
                putsUart0("-----Enter Command----- \r\n");
                if (is_Command("set", 2)) { //if(true/false) // not working always // send argc-1

                            arg1 = getvalue(1); // what if there are three arguments ? we give arguments as user input // not auto
                            arg2 = getvalue(2); // How to print arg2, can't see in global variable as well
                           // YELLOW_LED ^= 1;
                }

                else if (is_Command("reboot", 1)) { //MeasC on
                            NVIC_APINT_R = 0x05FA0004;
                                }

                else if (( strcmp("&", &str[pos[1]]) == 0 )) { //checking & for position 1
                    for ( i = 0 ; i < taskCount; i++){
                        if ( (strcmp(tcb[i].name, &str[pos[0]]) == 0) && (tcb[i].state == STATE_INVALID) ){
                            createThread(tcb[i].pid_1, tcb[i].name, 0);
                            putsUart0("Enter garbage string (length >10 )");
                        }

                        else {
                            //putsUart0("\n\r");
                            //putsUart0("Enter proper task name.");
                        }
                    }
                        //if (strcmp (tcb))
                        //putsUart0(&str[pos[1]]);
                          //for( i = 0 ; i < 9 ; i++){
                   // createThread(important, "important", 0); // settting priority 0
                              //if (strcmp (taskName[i], &str[pos[0]]) == 0){
                               //   createThread(important, "important", 0); // settting priority 0
                                  //createThread(functions[i], taskName[i], 0);
                                 // putsUart0(&str[pos[0]]);
                              //}
                              //putsUart0(taskName[i]);
                          //}

                                //for ( i = 0 ; i < taskCount; i++){
                               // if (strcmp(tcb[i].name, &str[pos[0]]) == 0){
                                    // Do nothing
                                //}
                                //else{}
                                   // is_Command()
                                //}
                            }
                          // createThread(important, "important", 0); // settting priority 0
                                //}


                else if (is_Command("pi", 1)) { // PI COMMAND
                            if ((strcmp("on", &str[pos[1]]) == 0)) {
                                priorityInheritance = true;
                                putsUart0("Priority Inheritance is on");

                            } else if ((strcmp("off", &str[pos[1]]) == 0)) {
                                priorityInheritance = false;
                                putsUart0("Priority Inheritance is off");
                            }
                        }

                else if (is_Command("sched", 1)) { // ROUNDROBIN OR PRIORITY SCHEDULER
                            if ((strcmp("rr", &str[pos[1]]) == 0)) {
                                priority_1 = false;
                                putsUart0("Round-robin is on");

                            } else if ((strcmp("priority", &str[pos[1]]) == 0)) {
                                priority_1 = true;
                                putsUart0("Priority scheduling is on");
                            }
                        }
                else if (is_Command("rtos", 1)) { // PREEMPTIVE / COOPERATIVE RTOS
                            if ((strcmp("preemp", &str[pos[1]]) == 0)) {
                                preemption = true;
                                putsUart0("preemption on");

                            } else if ((strcmp("coop", &str[pos[1]]) == 0)) {
                                preemption = false;
                                putsUart0("cooperative on");

                            }
                        }

                else if (is_Command("kill", 1)) { // KILL COMMAND
                    for ( k = 0 ; k < taskCount ; k++){
                        ltoa(tcb[k].pid , temp_1, 10);    // converting pid to char and store in temp
                            if ((strcmp(temp_1, &str[pos[1]]) == 0)) { // finding pid for task
                                destroyThread(tcb[k].pid); // passing pid to kill
                                //YELLOW_LED ^= 1;

                            }
                        }
                    }

                else if (is_Command("pidof", 1)) { // PIDOF COMMAND
                    for ( k = 0; k < taskCount ; k++ ){
                            if ((strcmp(tcb[k].name, &str[pos[1]]) == 0)) { // comparing string name task/taskCurrent
                                //return pid on screen
                                ltoa(tcb[k].pid , temp, 10);
                                putsUart0(temp);
                                putsUart0("\n\r");
                               // putsUart0(tcb[k].name);
                                //YELLOW_LED = 1;
                                }
                             }
                        }

                else if (is_Command("ps", 1)) { //PS COMMAND
                        if (strlen (name_1) > 0){
                            putsUart0(name_1);
                         for (i = strlen (name_1) ; i < strlen (space) ; i++ ){
                             putcUart0(space[strlen (space) - i]);
                                 } // end for statement
                            } // end for statement
                        putsUart0(" pid");
                        putsUart0("  ");
                        putsUart0("priority");
                        putsUart0(" ");
                        putsUart0("  Cpu %");
                        putsUart0("      ");
                        putsUart0("\r\n");
                        putsUart0("\r\n");
                       /*//this works
                        putsUart0("\r\n");
                        putsUart0("TaskName      ");
                        putsUart0(" pid  ");
                        putsUart0("priority  ");
                        putsUart0(" Cpu %");
                        putsUart0("\r\n");
                        putsUart0("\n\r");
                        */

                        for ( k = 0; k < 9/*taskCount*/ ; k++ ){
                        if ( (strlen (tcb[k].name) > 0) && (tcb[k].state != STATE_INVALID)){ //if (strlen (tcb[k].name > 9)){
                            length = strlen (tcb[k].name); // storing string length in varaiable length
                            putsUart0(tcb[k].name);
                            for (i = length ; i < strlen (space) ; i++){
                               putcUart0(space[strlen (space)-i]);
                            }
                        }
                       // else{
                       // putsUart0(tcb[k].name);
                       // }
                        if (tcb[k].state != STATE_INVALID){

                        //putsUart0("    ");
                        ltoa(tcb[k].pid , temp, 10);
                        putsUart0(temp);
                        putsUart0("      ");
                        ltoa(tcb[k].priority, temp_1, 10);
                        putsUart0(temp_1);
                        putsUart0("      ");
                        ltoa(tcb[k].percentage, temp_2, 10);
                        if (strlen(temp_2) >= 4){ //CHECK THIS PART
                       // for (i = 0; i < 2; i++){
                            putcUart0(temp_2[0]);
                            putcUart0(temp_2[1]);

                       // }
                            putsUart0(".");

                       // for (i = 2 ; i < strlen(temp_2) ; i++)
                       // {
                            putcUart0(temp_2[2]);
                          //putcUart0(temp_2[3]);
                       //     }
                        }
                        else if (strlen(temp_2) == 3){
                            //for (i = 0; i < 1; i++){
                                putcUart0(temp_2[0]);
                            //}
                                putsUart0(".");
                      // for (i = 1 ; i < strlen(temp_2) ; i++)
                          //  {
                           putcUart0(temp_2[1]);
                           putcUart0(temp_2[2]);
                           // }
                        }
                        else if (strlen(temp_2) == 2){
                            putsUart0("0.");
                           // for (i = 2 ; i < strlen(temp_2) ; i++)
                           // {
                                putcUart0(temp_2[0]);
                                putcUart0(temp_2[1]);
                           // }
                        }
                        else if (strlen(strlen(temp_2) == 1)){
                            putsUart0("0.0");
                        //    for (i = 0 ; i < strlen(temp_2) ; i++)
                       // {
                                putcUart0(temp_2[0]);
                       // }

                        }
                      //  putsUart0(temp_2);
                        putsUart0("\n\r");
                      }
                   }
                }

                else if (is_Command("ipcs", 1)) { //IPCS COMMAND
                        for( k = 0 ; k < semaphoreCount ; k++){
                            putsUart0("-----------------");
                            putsUart0("\n\r");
                            putsUart0("Semaphore name: ");
                            putsUart0(semaphores[k].semaphoreName);
                            putsUart0("\n\r");
                            putsUart0("count: ");
                            ltoa(semaphores[k].count , temp, 10);
                            putsUart0(temp); // printing semaphores count
                            putsUart0("\n\r");

                            if (semaphores[k].queueSize > 0 ){
                                for( j = 0 ; j < semaphores[k].queueSize; j++){
                                   putsUart0("Waiting Semaphore: ");
                                   putsUart0(tcb[semaphores[k].processQueue[j]].name);
                                   putsUart0("\n\r");

                                }

                            }
                            putsUart0("-----------------");
                            putsUart0("\n\r");

                        }
                         // YELLOW_LED ^= 1;
                         }

                else if (is_Command("help", 1)) {
                    putsUart0("List of Commands");
                    putsUart0("\n\r");
                    putsUart0("----------------");
                    putsUart0("\n\r");
                    putsUart0("reboot : reboots the pc");
                    putsUart0("\n\r");
                    putsUart0("taskName & : starts the task if not running");
                    putsUart0("\n\r");
                    putsUart0("pi on/off : turns priority inheritance on or off");
                    putsUart0("\n\r");
                    putsUart0("sched rr/priority : select either round-robin or priority scheduling");
                    putsUart0("\n\r");
                    putsUart0("rtos preemp/coop: select either preemptive or cooperative rtos");
                    putsUart0("\n\r");
                    putsUart0("kill pid: kills thread from its pid number");
                    putsUart0("\n\r");
                    putsUart0("pidof taskname: provides pid number from taskname");
                    putsUart0("\n\r");
                    putsUart0("ps: lists process, pid, priority, cpu %"); //This line may cause issue, run to check code, check escape character
                    putsUart0("\n\r");
                    putsUart0("ipcs: lists semaphores, resources available, waiting semaphore");
                    putsUart0("\n\r");
                    //putsUart0("\n\r");

                }

                else if (is_Command("preempt", 1)) { // KILL COMMAND
                    // converting pid to char and store in temp
                            if ((strcmp("on", &str[pos[1]]) == 0)) { // finding pid for task
                                preemption = true; // passing pid to kill
                            }
                            else if((strcmp("off", &str[pos[1]]) == 0)){
                                preemption = false;
                        }
                    }

                else {
                            putsUart0("-----------------------\r\n");
                            putsUart0("Invalid Command: \r\n");
                            waitMicrosecond(100000);    // changed from 500000
                            putsUart0("Enter Command again: \r\n");
                            putsUart0("-----------------------\r\n");

                        }

    }
}
// Blocking function that writes a serial character when the UART buffer is not full

void putcUart0(char c) { // code does not leave line 90 until fifo not full
    while (UART0_FR_R & UART_FR_TXFF); // before you write on line 91 wait till you have space on FIFO
    UART0_DR_R = c; // how fast can you write , 16 at a time
} // line 91 puts character on fifo and returns the fifo, don't write on buffer until there is room for it.

// Blocking function that writes a string when the UART buffer is not full

void putsUart0(char* str) {
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty

char getcUart0() {
    while (UART0_FR_R & UART_FR_RXFE){yield();}; // before reading line 106 make sure receiver frame buffer (FIFO) is not empty
    return UART0_DR_R & 0xFF; // reads the data register

}

void parse_string(char *str) {
    str_1[0] = 'd'; // dummy string with first delimeter representation
    uint8_t j, k, m, n = 0;
    //uint8_t counter = 0; // counter defined as uint8_t check it and redefine as int
    counter = 0;
    // create global variable  of argc; pos[]; type[];
    while (1) {
        for (j = 0; j < count; j++) { // go through string to analyze all char of string

            if (str[j] >= 48 && str[j] <= 57) { // condition of number
                str_1[j + 1] = 'n'; // suedo for delimeter

            } else if ((str[j] >= 65 && str[j] <= 122) || str [j] == 38) { // condition of letter ADDED FOR AMPERSEND
                str_1[j + 1] = 'l';

            }
            else { // condition for delimeter
                str_1[j + 1] = 'd';

            }

        } // END FOR LOOP

       // putsUart0("printing dummy str1");
        putsUart0("\r\n");
       // putsUart0(str_1);
       // putsUart0("\r\n");

        for (k = 0; k < count; k++) {

            if (str_1[k] == 'd' && str_1[k + 1] == 'l') {
                pos[counter] = k;
                type[counter] = 'a'; // delimeter to letter
                counter++;
            } else if (str_1[k] == 'd' && str_1[k + 1] == 'n') {
                pos[counter] = k;
                type[counter] = 'n'; // delimeter to number
                counter++;
            }
           /* else if (str_1[k] == 'l' && str_1[k + 1] == 'n') {
                pos[counter] = k;
                type[counter] = 'n'; // letter to number
                counter++;
            } */
            else {
                /*do nothing condition*/
            }
            /*check if you could end position array or type array with empty element or not*/
        }

        argc = counter; // last null check subtracted (counter-1) or what ?
        arg = counter - 1; // can you put arg here, and update global variable, its a function or what ?

        for (m = 1; m < count + 1; m++) { // NUlling delimeter
            if (str_1[m] == 'd') { //convert every delimeter to null // actual str or suedo string
                str[m - 1] = '\0';
            }
        }
        break;
    }
   // putsUart0("printing important stuff");
    //putsUart0("\r\n");

    // checking step only
    for (n = 0; n < argc; n++) { // argc replaced  by counter
        putsUart0(&str[pos[n]]);
        putsUart0("\r\n");
        // putsUart0(13);
    }

    for (n = 0; n < count + 1; n++) { // read it
        str_1[n] = '\0'; // empty out str_1 in case if new str has less index
    }

    return;

} //END FUNCTION PARSE_STRING

int getvalue(int arg) { // check this function
    number = 0; // clearing number
    if (type[arg] == 'n') { // if or while
        number = atoi(&str[pos[arg]]); //pos[arg] changed to 1.number still 0 num 1 changed to 12.
    } // check the number as global or local variable in code composer studio varible list
    return number;
}

bool is_Command(char *str1, int arg) { // pass string and minimum number of argument here
    // so is it supposed to find argument by itself or not
    // is command ma verb and argument pass garne
    // argument pathaunu parne ma, argument lai overwrite gareko cha
    bool valid = false; // by default valid lai false banaune

    //if ((strcmp("set", &str[pos[0]]) == 0) && (arg == 2)) {  // argc-1 for funtion to execute argc = 3
    if ((strcmp(str1, &str[pos[0]]) == 0) && (arg == 2)) { // argc-1 for funtion to execute argc = 3
        valid = true;
    } else if ((strcmp(str1, &str[pos[0]]) == 0) && (arg == 1)) { //argc-1 // for funtion to execute argc = 3
        valid = true;
    }
    else {}

    return valid;
}
void setTimerMode() // doctor losh
{
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R |= TIMER_TAMR_TAMR_1_SHOT | TIMER_TAMR_TACDIR ;         //one shot -- count up
    WTIMER5_TAV_R = 0;                               // zero counter
}


void cpuTime(){
    uint8_t i;
    totalTime = 0;
    for(i=0 ; i < taskCount ; i++){
        totalTime = totalTime + tcb[i].filterTime;
    }

    for(i=0 ; i < taskCount ; i++){
        tcb[i].percentage = (((tcb[i].filterTime) * 10000) /totalTime); // multiply by higher number 10000 and add dot
    }

    totalTime = 0; // resetting total time
    for(i=0 ; i < taskCount ; i++){
            tcb[i].filterTime = 0;  // resetting filter time for next window
            tcb[i].time = 0;        // resetting time for next window
    }

    //reset tcb[taskCurrent].time after calculating all cpu values

}

int8_t minimumPriority( int8_t a , int8_t b){
    if (a > b){
        return b;
    }
    else
        return a;
}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void) {
    bool ok;

    preemption = false; // if preemption on with other falseISR
    priority_1 = true; // SET PRIORITY SCHEDULING ON FOR START
    priorityInheritance = true; // SET PRIORITY INHERITANCE

    // Initialize hardware
    initHw();
    setTimerMode(); // Initializing timer
    rtosInit();

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    putsUart0("-----Enter Command----- \r\n");

    // Initialize semaphores
    keyPressed = createSemaphore(1, "keypressed");
    keyReleased = createSemaphore(0, "keyreleased");
    flashReq = createSemaphore(5, "flashreq");
    resource = createSemaphore(1, "resource");

    // Add required idle process at lowest priority
     ok = createThread(idle, "idle", 7); // CHANGED FROM UPPERCASE TO LOWERCASE i
   // ok =  createThread(idle2, "Idle2", 7); // creating idle 2 and making equal priority as idle

    // Add other processes
     ok &= createThread(lengthyFn, "lengthyfn", 4);
     ok &= createThread(flash4Hz, "flash4hz", 0);
     ok &= createThread(oneshot, "oneshot", -4);
     ok &= createThread(readKeys, "readkeys", 4);
     ok &= createThread(debounce, "debounce", 4);
     ok &= createThread(important, "important", -8);
     ok &= createThread(uncooperative, "uncoop", 2);
     ok &= createThread(shell, "shell", 0);

    // Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;

    return 0;
}

//references: https://stackoverflow.com/questions/1118705/call-a-function-named-in-a-string-variable-in-c
