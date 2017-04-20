#include <Arduino.h>
//#include <stdint.h>
//#include <stdbool.h>
#include <setjmp.h>
#include <libmaple/scb.h>

#define DEFAULT_STACK_SIZE  1024 /** Default tasks stack size and stack max. */
#define MAIN_STACK_SIZE  1024
#define STACK_MAX  8000

extern "C" {
    extern unsigned __msp_init; // defined by link script
}
#define RAMEND ((size_t)&__msp_init)
    
  /**
   * Start a task with given functions and stack size. Should be
   * called from main task (in setup). The functions are executed by
   * the task. The taskSetup function (if provided) is run once.
   * The taskLoop function is repeatedly called. The taskSetup may be
   * omitted (NULL). Returns true if successful otherwise false.
   * @param[in] taskSetup function (may be NULL).
   * @param[in] taskLoop function (may not be NULL).
   * @param[in] stackSize in bytes.
   * @return bool.
   */

typedef void (*func_t)();

  /**
   * Task run-time structure.
   */
typedef struct Task_t {
    struct Task_t* next;               //!< Next task
    struct Task_t* prev;               //!< Previous task
    jmp_buf context;            //!< Task context
    const uint8_t* stack;       //!< Task stack
    uint16_t id;                // id of task
    bool active;                // flag
    uint32_t ttw;          // time to work
    uint32_t t_yield;      // time of yield
    uint32_t start; // microseconds of timeslice start
    uint32_t delay; // maximal execution time of task
#ifdef MTASK_PROF
    uint32_t ticks; // ticks of CPU to calculate context switch time
    uint64_t time;  // full time
#endif
} task_t;


void * start_task(func_t taskSetup, func_t taskLoop, size_t stackSize = DEFAULT_STACK_SIZE);
void stop_task(void * h);
void resume_task(void * h);

/**               
   * Context switch to next task in run queue.
 */
void yield(uint16_t ttw=0); // optional time to wait
  
  /**
   * Return current task stack size.
   * @return bytes
   */
size_t task_stack();
  
bool is_main_task();

  

