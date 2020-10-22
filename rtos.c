/* hw9.c
 * Kathryn Price
 * Homework 9
 * CS552
 * Dr. Rinker
 * May 8, 2015
 */
 
/* Purpose:
 *
 * UIK - homework 7
 * UIK is a multitasking kernal. The UIK will run 3 user tasks
 * plus an idle task with preemptive multitasking. This RTOS
 * implements a counting semaphore, a timer event 
 * handler and an aging scheduler.
 *
 * SEMAPHORE - homework 8
 * All the blinking LED tasks utilize the same portout global 
 * variable which is used to display the correct blinking 
 * sequences of LEDs via PORTD. PORTD is only used to set
 * portout at the beginning of the RTOS. After that, the
 * portout variable is protected by a semaphore and is changed
 * by all three tasks.
 *
 * EVENTS - homework 9
 * Timer0 is an event timer. Tasks 2 and 3 use the tick timer
 * to delay the LED blinks while Task 1 uses the event timer.
 *
 * Thanks to www.freertos.org for the coding examples and help.
 */

 /* Function Sizes: 
  *     UIKSemCreate             90 bytes
  *     UIKRun                  108 bytes
  *     UIKTickHandler          358 bytes
  *     initTasks               952 bytes
  *     UIKEventCreate           56 bytes
  *     UIKInitialize            96 bytes
  *     ISR(TIMER1_COMPA_vect)  338 bytes
  *     UIKSemPost               78 bytes
  *     setTaskQ                 32 bytes
  *     UIKSchedule             230 bytes
  *     UIKTaskDelay             52 bytes
  *     taskTwo                 176 bytes
  *     UIKIdle                  12 bytes
  *     SemWaitPop               98 bytes
  *     UIKSemPend               80 bytes
  *     main                    170 bytes
  *     UIKSemValue              32 bytes
  *     taskThree               124 bytes
  *     beginRTOS               142 bytes
  *     SemWaitPush              94 bytes
  *     portInit                 34 bytes
  *     init_crcTimer            92 bytes
  *     WaitEvent                78 bytes
  *     taskOne                 182 bytes
  *     InitTimer               110 bytes
  *     UIKAddTask              238 bytes  
  *     ISR(TIMER0_COMP_vect)   120 bytes
  *     StartTimer               44 bytes
  */

/* Scheduler overhead: 
 * The amount of overhead for the ISR scheduler is
 * 20% of the total program time.
 *
 * The amount of overhead for the "main" program is
 * taskOne overhead is 30%
 * taskTwo overhead is 20%
 * taskThree overhead is 20%
 */

/* includes */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <avr/interrupt.h>

/* defines */
/* context switching macros */
#define SAVE_CONTEXT()\
asm volatile (\
"push r0                \n\t"\
"in   r0,__SREG__       \n\t"\
"cli                    \n\t"\
"push r0                \n\t"\
"push r1                \n\t"\
"clr  r1                \n\t"\
"push r2            \n\t"\
"push r3            \n\t"\
"push r4            \n\t"\
"push r5            \n\t"\
"push r6            \n\t"\
"push r7            \n\t"\
"push r8            \n\t"\
"push r9            \n\t"\
"push r10           \n\t"\
"push r11           \n\t"\
"push r12           \n\t"\
"push r13           \n\t"\
"push r14           \n\t"\
"push r15           \n\t"\
"push r16           \n\t"\
"push r17           \n\t"\
"push r18           \n\t"\
"push r19           \n\t"\
"push r20           \n\t"\
"push r21           \n\t"\
"push r22           \n\t"\
"push r23           \n\t"\
"push r24           \n\t"\
"push r25           \n\t"\
"push r26           \n\t"\
"push r27           \n\t"\
"push r28           \n\t"\
"push r29           \n\t"\
"push r30           \n\t"\
"push r31           \n\t"\
"lds  r26, curr \n\t"\
"lds  r27, curr+1   \n\t"\
"in   r0, __SP_L__      \n\t"\
"st   x+, r0            \n\t"\
"in   r0, __SP_H__      \n\t"\
"st   x+, r0            \n\t"\
);

#define RESTORE_CONTEXT()\
asm volatile (\
"lds  r26, curr      \n\t"\
"lds  r27, curr+1  \n\t"\
"ld   r28, x+                \n\t"\
"out  __SP_L__, r28          \n\t"\
"ld   r29, x+                \n\t"\
"out  __SP_H__, r29          \n\t"\
"pop  r31                    \n\t"\
"pop  r30                    \n\t"\
"pop  r29                    \n\t"\
"pop  r28                    \n\t"\
"pop  r27                    \n\t"\
"pop  r26                    \n\t"\
"pop  r25                    \n\t"\
"pop  r24                    \n\t"\
"pop  r23                    \n\t"\
"pop  r22                    \n\t"\
"pop  r21                    \n\t"\
"pop  r20                    \n\t"\
"pop  r19                    \n\t"\
"pop  r18                    \n\t"\
"pop  r17                    \n\t"\
"pop  r16                    \n\t"\
"pop  r15                    \n\t"\
"pop  r14                    \n\t"\
"pop  r13                    \n\t"\
"pop  r12                    \n\t"\
"pop  r11                    \n\t"\
"pop  r10                    \n\t"\
"pop  r9                    \n\t"\
"pop  r8                    \n\t"\
"pop  r7                    \n\t"\
"pop  r6                    \n\t"\
"pop  r5                    \n\t"\
"pop  r4                    \n\t"\
"pop  r3                    \n\t"\
"pop  r2                    \n\t"\
"pop  r1                     \n\t"\
"pop  r0                     \n\t"\
"out  __SREG__, r0           \n\t"\
"pop  r0                     \n\t"\
);

#define NTASKS 4 /* maximum number of tasks allowed */
#define STACKSIZE 0x32 /* Bytes deep, all tasks have same stack size */ 

/* task stacks */
uint8_t task0Stack[STACKSIZE];
uint8_t task1Stack[STACKSIZE];
uint8_t task2Stack[STACKSIZE];
uint8_t task3Stack[STACKSIZE];

/* typedefs, data structures, globals */
typedef enum {blocked_q, ready_q, running} taskQueue;
/* each priority can only be assigned to one task */
/* lolo always assigned to idle task */
typedef enum {lolo, lo, hi, hihi} priorityLevel;

typedef void (*tdFuncPtr)();
uint32_t UIKTickNum = 0;
uint16_t UIKTickLen; 
uint8_t numberOfTasks = 0;

typedef struct tcb
{ /* task context block */
    tdFuncPtr task;   
    uint8_t taskId;
    priorityLevel priority;    
    taskQueue queue;
    uint16_t delay;
    uint8_t *stackBeg;
    uint8_t *sp;
} tcb_t;
tcb_t volatile taskTCB[NTASKS]; /* task TCB array */
uint8_t * volatile curr = NULL; /* current stack pointer */
uint8_t volatile taskId; /* current task */
uint8_t volatile oldId = 0; /* task that was running */

/* ***User can change*** */
uint8_t maxNumberOfTasks; /* include UIK idle task */
uint16_t theTickLength; /* in micro seconds */
/* ***End User Globals *** */

/* function prototypes */
ISR(TIMER1_COMPA_vect, ISR_NAKED); /* also known as UIKDispatcher/scheduler */
void UIKInitialize(uint16_t tickLen, uint8_t maxTasks);
uint8_t UIKAddTask(tdFuncPtr pTask, priorityLevel level, uint8_t *pstack);
void UIKRun(uint8_t id);
void UIKSchedule(void) __attribute__ ((naked));
void UIKIdle(void);
void UIKTickHandler(void);

void beginRTOS(void) __attribute__ ((naked));
void init_crcTimer(uint16_t tickLen);
void initTasks(uint8_t *pStackTop, tdFuncPtr pt, uint8_t taskNum);
void portInit(void);
void setTaskQ(taskQueue q, uint8_t i) __attribute__ ((naked));
void UIKTaskDelay(uint16_t count) __attribute__((naked)); // sets a task to delay for count number of ticks

/* Semaphores:
 * The portout variable which stores the new value for the LED port 
 * is now a global and is protected by a semaphore.  This will allow for
 * no changes to the output LEDs until an the portout semaphore is released.
 * Each of the user tasks use portout and must request and release it
 * before and after using portout.
 */
uint8_t portout; /* used to demo semaphore */

typedef struct sema
{ /* counting semaphore information */
    int8_t theSemaphore;
    uint8_t waitingTasks[NTASKS];
    uint8_t readTask;
    uint8_t writeTask;
} sema_t;
sema_t sema_portout;
void UIKSemCreate(sema_t *semaphore);  // Sets up the struct for counting semaphore
uint8_t UIKSemPend(sema_t *semaphore); // semaphore--, if not avail push task to list
uint8_t UIKSemPost(sema_t *s); // semaphore++, if tasks waiting, pop a task
int8_t UIKSemValue(sema_t *s); // returns sema value
uint8_t SemWaitPop(sema_t *s);
void SemWaitPush(sema_t *s);

/* Events:
 * The function taskOne now uses the event timer0 instead of the tick timer to 
 * start and suspend itself.
 */
typedef struct event
{ /* event information */
    uint8_t eventTask;
    uint8_t volatile eventFlag;
    uint8_t eventNum;
} event_t;
event_t event_timer0; /* event 0 for timer 0 */
void UIKEventCreate(event_t *e, uint8_t num); /* init event struct */
void StartTimer(uint8_t timernum);
uint8_t WaitEvent(event_t *e);
uint8_t InitTimer(uint8_t timernum, uint32_t period); 
ISR(TIMER0_COMP_vect); /* Event timer */

/* User Tasks: */
/* Task to blink the lower 2 of the lower 4 LED's. */
/* Uses semaphore to change global portout variable. */
/* Uses event timer to block and unblock itself. */
/* Lowest priority besides UIKIdle */
void taskOne(void);
/* Task to blink the upper 2 of the lower 4 LED's. */
/* Uses semaphore to change global portout variable. */
/* Uses UIKTaskDelay to block and unblock itself (based on ticks). */
void taskTwo(void);
/* Task to count on the upper 4 LEDs. */
/* Uses semaphore to change the global portout variable. */
void taskThree(void);

uint8_t main(void)
{
    uint8_t i;
    /* initalize port for tasks */
    portInit();
    portout = ~PORTD; // set the portout global

    /* initalize UIK RTOS */
    UIKInitialize(10000, 4); // tick timer(tick length, number of tasks)
    
    /* Add user tasks */
    taskId = UIKAddTask(&taskOne, lo, &task1Stack[STACKSIZE-1]);
    UIKRun(taskId); /* sets up stack */
    taskId = UIKAddTask(&taskTwo, hi, &task2Stack[STACKSIZE-1]);
    UIKRun(taskId); /* sets up stack */
    taskId = UIKAddTask(&taskThree, hihi, &task3Stack[STACKSIZE-1]);
    UIKRun(taskId); /* sets up stack */
    
    /* set up portout semaphore */
    UIKSemCreate(&sema_portout);    
    
    /* set up event info & timer */
    i = InitTimer(0, 250000); // timer 0 in micro seconds
    UIKEventCreate(&event_timer0, 0);
    
    beginRTOS();
    return(0); /* will not exit */
} 

uint8_t InitTimer(uint8_t timernum, uint32_t period)
{ 
    uint8_t i = 100; // if return 100 then invalid
    if(timernum == 0)
    {
        TCNT0 = 0;
        TCCR0 = ((1<<WGM01) | (1<<CS02) | (1<<CS00));
        //        CTC Mode    Prescaler clk/1024
        OCR0 =  (uint8_t) (period/1024); 
        i = timernum;
    }
    return i;
}

void StartTimer(uint8_t timernum)
{
    if(timernum == 0)
    {
        TIMSK |= 1<<OCIE0; /* enable timer0 compare interrupt */
    }
}

void UIKEventCreate(event_t *e, uint8_t num)
{
    e->eventTask = 0; // idle task will never use it
    e->eventFlag = 0; // not set   
    e->eventNum = num; // This links event struct to timernum
}

uint8_t WaitEvent(event_t *e)
{
    e->eventTask = taskId;
    e->eventFlag = 0; 
    setTaskQ(blocked_q, taskId);
    StartTimer(e->eventNum);  
    return e->eventNum;
}

ISR(TIMER0_COMP_vect)
{ // event handler
    event_timer0.eventFlag = 1;
    setTaskQ(ready_q, event_timer0.eventTask);
    TIMSK |= 0<<OCIE0;
}
void setTaskQ(taskQueue q, uint8_t i)
{
    taskTCB[i].queue = q;
    asm volatile ("ret");
}

void UIKSemCreate(sema_t *semaphore)
{
    uint8_t i;
    semaphore->theSemaphore = 1; /* no tasks are using it */
    for(i = 0; i < NTASKS; i++)
    {
        semaphore->waitingTasks[i] = 0;
    }
    semaphore->readTask = 0;
    semaphore->writeTask = 0;
}

int8_t UIKSemValue(sema_t *s)
{
    return s->theSemaphore;
}

uint8_t SemWaitPop(sema_t *s)
{   uint8_t i;
    i = s->waitingTasks[s->readTask];
    s->readTask = (s->readTask + 1) % NTASKS;
    return i;
}

uint8_t UIKSemPost(sema_t *s)
{
    uint8_t i = 0;
    cli();
    s->theSemaphore++;
    if(UIKSemValue(s) < 1)
    { /* waiting tasks, pop a task */
        i = SemWaitPop(s);
    }
    sei();
    return i;
}

void SemWaitPush(sema_t *s)
{
    s->waitingTasks[s->writeTask] = taskId;
    s->writeTask = (s->writeTask + 1) % NTASKS;
}

uint8_t UIKSemPend(sema_t *semaphore)
{
    uint8_t isReady = 1;
    cli();
    semaphore->theSemaphore--;
    if(UIKSemValue(semaphore) < 0)
    { // wait, push task to list
        SemWaitPush(semaphore);
        isReady = 0;
    }
    sei();
    return isReady;
}

void UIKIdle(void)
{ /* Needs to be lowest priority! E.g. lolo */
    while(1)
    { /* Do nothing */
        asm volatile ("nop");
    }    
}

/* ****USER TASKS ******* */
void taskOne(void)
{ /* Task to blink the lower 2 of the lower 4 LED's. */
  /* Uses semaphore to change global portout variable. */
  /* Uses event timer to block and unblock itself. */
  /* Lowest priority besides UIKIdle */
    uint8_t blink[2] = {0xff, 0xfc};
    static uint8_t i = 0;
    uint8_t j;
    while(1)
    {
        if(UIKSemPend(&sema_portout) == 0)
        {
            setTaskQ(blocked_q, taskId);
            UIKSchedule();
        }
        portout =  portout & 0xfc;
        portout = portout | ~(blink[i]); /* set the LEDs */
        PORTD = ~portout;
        j = UIKSemPost(&sema_portout); /* release semaphore */
        if(j > 0)
        {
            setTaskQ(ready_q, j);
        }
        i = (i + 1) % 2;
        /* set delay, block and relinquish control */
        //UIKTaskDelay(177);
        j = WaitEvent(&event_timer0);
        UIKSchedule();
    }
}

void taskTwo(void)
{ /* Task to blink the upper 2 of the lower 4 LED's. */
  /* Uses semaphore to change global portout variable. */
  /* Uses tick timer to block and unblock itself. */
    uint8_t blink[2] = {0xff, 0xf3};
    static uint8_t i = 0;
    uint8_t j;
    while(1)
    {
        if(UIKSemPend(&sema_portout) == 0)
        {
            setTaskQ(blocked_q, taskId);
            UIKSchedule();
        }
        portout = portout & 0xf3;
        portout = portout | ~(blink[i]); /* set the LEDs */
        PORTD = ~portout;
        j = UIKSemPost(&sema_portout); // release!
        if(j > 0)
        {
            setTaskQ(ready_q, j);
        }
        i = (i + 1) % 2;
        /* set delay, block and relinquish control */
        UIKTaskDelay(150);
    }
}

void taskThree(void)
{ /* task to count on the upper 4 LEDs */
    static uint8_t bcnt = 0;
    uint8_t j;
    while(1)
    {
        if(UIKSemPend(&sema_portout) == 0)
        {
            setTaskQ(blocked_q, taskId);
            UIKSchedule();
        }
        //portout = (~PORTD) & 0x0f; /* save off the lower 4 LEDs */
        portout = portout & 0x0f;
        portout = portout | bcnt; /* set the upper 4 LEDs */
        PORTD = ~portout;
        j = UIKSemPost(&sema_portout); // release!
        if(j > 0)
        {
            setTaskQ(ready_q, j);
        }
        bcnt += 0x10;
        /* set delay, block and relinquish control */
        UIKTaskDelay(50);
    }   
}
/* ***END OF USER TASKS** */

void UIKTaskDelay(uint16_t count)
{
    taskTCB[taskId].delay = count;
    setTaskQ(blocked_q, taskId);
    UIKSchedule();
    asm volatile("reti");
}

void UIKInitialize(uint16_t tickLen, uint8_t maxTasks)
{
    /* tickLen in microseconds */
    maxNumberOfTasks = maxTasks;
    theTickLength = tickLen;
    UIKTickLen = tickLen;
    
    /* set up tick timer */
    init_crcTimer(tickLen);
    
    /* Add the idle task as the first task */
    taskId = UIKAddTask(&UIKIdle, lolo, &task0Stack[STACKSIZE-1]);
    UIKRun(taskId); /* sets up stack */
}

void UIKRun(uint8_t id)
{
    uint8_t *pStack;
    pStack = taskTCB[id].stackBeg;
    initTasks(pStack, taskTCB[id].task, id);
    setTaskQ(ready_q, id);
}

uint8_t UIKAddTask(tdFuncPtr pTask, priorityLevel level, uint8_t *pstack)
{ /* sets up the TCB */
    taskTCB[numberOfTasks].task = pTask;
    taskTCB[numberOfTasks].taskId = numberOfTasks;
    taskTCB[numberOfTasks].priority = level;
    setTaskQ(blocked_q, numberOfTasks); /* task blocked until stack is set up */
    taskTCB[numberOfTasks].stackBeg = pstack;
    taskTCB[numberOfTasks].delay = 0;
    numberOfTasks++;
    return (numberOfTasks - 1);
}

void initTasks(uint8_t *pStackTop, tdFuncPtr pt, uint8_t taskNum)
{ /* sets up the task stack */
    /* This code is from FreeRTOS */
    uint16_t taskAddr;
    
    /* set up task stack pointer */
    taskTCB[taskNum].sp = pStackTop - 1;

    /* add placeholder bytes */
    *pStackTop = 0x11;
    pStackTop--;   
    *pStackTop = 0x22;
    pStackTop--;   
    *pStackTop = 0x33;
    pStackTop--;    

    /* push task code start addr */
    taskAddr = (uint16_t) pt;
    *pStackTop = (uint8_t) (taskAddr & (uint16_t) 0x00ff);
    pStackTop--;   
    
    taskAddr >>= 8;
    *pStackTop = (uint8_t) (taskAddr & ( uint16_t ) 0x00ff );
    pStackTop--;   
    
    *pStackTop = 0x00;  // R0
    pStackTop--;
    
    *pStackTop = ( uint8_t ) 0x80;   /* SREG? */
    pStackTop--;    
    *pStackTop = ( uint8_t ) 0x00;   /* R1 */
    pStackTop--;   
    *pStackTop = ( uint8_t ) 0x02;   /* R2 */
    pStackTop--;   
    *pStackTop = ( uint8_t ) 0x03;   /* R3 */
    pStackTop--;  
    *pStackTop = ( uint8_t ) 0x04;   /* R4 */
    pStackTop--;    
    *pStackTop = ( uint8_t ) 0x05;   /* R5 */
    pStackTop--;   
    *pStackTop = ( uint8_t ) 0x06;   /* R6 */
    pStackTop--;  
    *pStackTop = ( uint8_t ) 0x07;   /* R7 */
    pStackTop--;  
    *pStackTop = ( uint8_t ) 0x08;   /* R8 */
    pStackTop--;    
    *pStackTop = ( uint8_t ) 0x09;   /* R9 */
    pStackTop--;   
    *pStackTop = ( uint8_t ) 0x10;   /* R10 */
    pStackTop--;
    *pStackTop = ( uint8_t ) 0x11;   /* R11 */
    pStackTop--;   
    *pStackTop = ( uint8_t ) 0x12;   /* R12 */
    pStackTop--; 
    *pStackTop = ( uint8_t ) 0x13;   /* R13 */
    pStackTop--;    
    *pStackTop = ( uint8_t ) 0x14;   /* R14 */
    pStackTop--;   
    *pStackTop = ( uint8_t ) 0x15;   /* R15 */
    pStackTop--;   
    *pStackTop = ( uint8_t ) 0x16;   /* R16 */
    pStackTop--;  
    *pStackTop = ( uint8_t ) 0x17;   /* R17 */
    pStackTop--; 
    *pStackTop = ( uint8_t ) 0x18;   /* R18 */
    pStackTop--;   
    *pStackTop = ( uint8_t ) 0x19;   /* R19 */
    pStackTop--;    
    *pStackTop = ( uint8_t ) 0x20;   /* R20 */
    pStackTop--; 
    *pStackTop = ( uint8_t ) 0x21;   /* R21 */
    pStackTop--;  
    *pStackTop = ( uint8_t ) 0x22;   /* R22 */
    pStackTop--;  
    *pStackTop = ( uint8_t ) 0x23;   /* R23 */
    pStackTop--;  
    *pStackTop = ( uint8_t ) 0x24;   /* R24 */ 
    pStackTop--;  
    *pStackTop = ( uint8_t ) 0x25;   /* R25 */
    pStackTop--;
    *pStackTop = ( uint8_t ) 0x26;   /* R26 X */
    pStackTop--; 
    *pStackTop = ( uint8_t ) 0x27;   /* R27 */
    pStackTop--;  
    *pStackTop = ( uint8_t ) 0x28;   /* R28 Y */
    pStackTop--;   
    *pStackTop = ( uint8_t ) 0x29;   /* R29 */
    pStackTop--;  
    *pStackTop = ( uint8_t ) 0x30;   /* R30 Z */
    pStackTop--; 
    *pStackTop = ( uint8_t ) 0x031;  /* R31 */
    pStackTop--;    
    
    /* Now store the address of the stack where 0x11 & 0x22 are stored */ 
    taskAddr = (uint16_t) pStackTop;
    *(taskTCB[taskNum].sp) = (taskAddr & 0xff);
    taskAddr >>= 8;
    *(taskTCB[taskNum].sp + 1) = (taskAddr & 0xff);
}

void init_crcTimer(uint16_t tickLen)
{
    // Need to use timer1 for the 16 bits with no prescaler
    // WGM13 = 0 & WGM12 = 1 for clear timer on compare (CTC) mode
    // CS10 = 1 for no prescaling because we need microseconds
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS10);
    // OCIE1A - timer1 output compare interrupt enabled
    TIMSK |= (1 << OCIE1A);
    // Set the compare ticks
    OCR1A = tickLen;
}

void portInit(void)
{
    DDRD = 0xff; /* enable port d for output */
    PORTD = 0xff; /* lights off */
}

ISR(TIMER1_COMPA_vect, ISR_NAKED)
{ // UIKDispatcher 
    SAVE_CONTEXT();
    oldId = taskId;
    if( taskTCB[taskId].queue == blocked_q )
    { 
        oldId = 100;
    }
    UIKTickHandler();
    UIKTickNum++;
    if(oldId < 100)
    {
        setTaskQ(ready_q, oldId);
    }
    setTaskQ(running, taskId);
    curr = taskTCB[taskId].sp;
    RESTORE_CONTEXT();
    asm volatile ( "reti" );
}

void UIKSchedule(void)
{ /* same as UIKDispatcher without the tick incriment */
    SAVE_CONTEXT();
    UIKTickHandler();
    setTaskQ(taskId, running);
    curr = taskTCB[taskId].sp;
    RESTORE_CONTEXT();
    asm volatile ( "reti" );
}

void beginRTOS(void)
{ /* start idle task */
    taskId = 0;
    curr = taskTCB[taskId].sp;
    setTaskQ(taskId, running);
    RESTORE_CONTEXT();
    asm volatile("reti"); /* enables interrupts */
}

void UIKTickHandler(void)
{
    uint8_t i;
    uint8_t newTask = 0; /* start with idle task */
    
    /* process delays, idle task is never blocked */
    for(i = 1; i < numberOfTasks; i++)
    { 
        if(taskTCB[i].queue == blocked_q)
        { // if delay is 0 then something else is blocking
            if(taskTCB[i].delay > 0)
            {
                taskTCB[i].delay--;   
                if(taskTCB[i].delay == 0)
                {
                    taskTCB[i].queue = ready_q;
                }
            }
        }
    }
    /* find highest priority task */
    for(i = 1; i < numberOfTasks; i++)
    { 
        if(taskTCB[i].queue != blocked_q)
        {
            if(taskTCB[i].priority > taskTCB[newTask].priority)
            {
                newTask = i;
            }   
        }
    }
    taskId = newTask;
}
