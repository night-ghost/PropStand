#include "task.h"


// Main task and run queue
static task_t s_main = {
  &s_main,
  &s_main,
  { 0 },
  NULL, // stack
  0,    // id
  true, // active
  0,    // ttw
  0,    // t_yeld
    0, // task_start
    0, // task_delay (calculated)
#ifdef MTASK_PROF
    0, // task_ticks
    0, // task_time
#endif
};

// Reference running task
static task_t* s_running = 0;

// Initial top stack for task allocation
static size_t s_top = MAIN_STACK_SIZE;

static uint16_t task_n=0;

//[ -------- realization of cooperative multitasking --------

bool adjust_stack(size_t stackSize)
{  // Set main task stack size
  s_top = stackSize;
  return true;
}


// Add task last to run queue
static uint32_t fill_task(task_t &tp){
    tp.next = &s_main;  // linked list
    tp.prev = s_main.prev;
    s_main.prev->next = &tp;
    s_main.prev = &tp;
    
    tp.id = ++task_n; // counter
    tp.active = true;
    tp.ttw = 1;       // delay after 1st enter

#ifdef MTASK_PROF
    tp.time=0;   // total time
    tp.delay=0;  // max execution time
    tp.start=micros(); 
#endif

    return (uint32_t)&tp; // только финт с возвратом из функции снимает проклятье "локальный адрес"
}


static inline  __attribute__(( naked )) uint32_t __get_BASEPRI( void )
{
    uint32_t result;
        __asm volatile                                              \
        (                                                           \
                "       mrs %0, basepri                             \n" \
                :"=r" (result)  \
            );

        return result;
}

static inline bool in_interrupt(){ return (SCB_BASE->ICSR & SCB_ICSR_VECTACTIVE) || (__get_BASEPRI()); }


// executes  C function as task
static void * init_task(func_t t_setup, func_t t_loop, const uint8_t* stack)
{
    task_t task;
    
    uint32_t ret=fill_task(task);  // Add task last in run queue (main task)
    task.stack = stack;

  // Create context for new task, caller will return
  if (setjmp(task.context)) {
    // we comes via longjmp - the task itself
    if (t_setup != NULL) {
         t_setup();
         yield();
    }
    while (1) {
        t_loop();
        yield();        // in case that function not uses delay();
    }
  }
  // caller returns
//  return &task; GCC optimizes out so returns 0

    return (void *)ret;
}

// start C function as task
void * start_task(func_t taskSetup, func_t taskLoop, size_t stackSize)
{
  // Check called from main task and valid task loop function
  if (!is_main_task() ) return NULL;
  if ( taskLoop == NULL) return NULL;

    if(stackSize == 0) stackSize=DEFAULT_STACK_SIZE;
    

  // Adjust stack size with size of task context
  stackSize += sizeof(task_t);

  // Allocate stack(s) and check if main stack top should be set
  size_t frame = RAMEND - (size_t) &frame;
  volatile uint8_t stack[s_top - frame]; // should be volatile else it will be optimized out
  if (s_main.stack == NULL) s_main.stack = (const uint8_t*)stack; // remember on first call stack of main task

  // Check that the task can be allocated
  if (s_top + stackSize > STACK_MAX) return NULL;

  // Adjust stack top for next task allocation
  s_top += stackSize;

  // Initiate task with stack top
  return init_task(taskSetup, taskLoop, (const uint8_t*)(stack - stackSize));
}


void stop_task(void *h){
    if(h) {
        task_t *tp = (task_t *)h ;
    
        tp->active=false;
    }
}


void resume_task(void *h){
    if(h) {
        task_t *tp = (task_t *)h ;
    
        tp->active=true;
    }
}


void yield(uint16_t ttw) // time to wait 
{
    if(task_n==0) return;
    if(ttw && ttw < 5) return; // don't mess into delays less than 5uS - 840 steps
    if(in_interrupt()) return; // don't switch privileged context

    task_t *me = s_running;

// if yield() called with a time it can remember that task don't want to run all this time and exclude it from time sliceing
    uint32_t t =  micros();
    me->t_yield = t;
    me->ttw     = ttw; // remember that task want to wait

    uint32_t dt =  t - me->start; // time in task
    if(dt>me->delay) me->delay = dt; // and remember maximum

#ifdef MTASK_PROF
    me->time+=dt;                           // calculate sum
    
    uint64_t ticks = stopwatch_getticks();
#endif
    if (setjmp(me->context)) {
        // we come here via longjmp - context switch is over
#ifdef MTASK_PROF
        yield_time += stopwatch_getticks() - s_running->ticks; // time of longjmp
        yield_count++;                  // count each context switch
#endif
        return;
    }
    // begin of context switch
#ifdef MTASK_PROF
    yield_time += stopwatch_getticks()-ticks; // time of setjmp
#endif

// TODO: check for full loop and spell WFI in case
next:
    // Next task in run queue will continue
    s_running = s_running->next;
//    if(s_running == me) WFI();  // full loop - there is no job <- spoils microsecond delays
    
    if(!s_running->active) goto next; // skip finished tasks

    // task has a ttw  and time since that moment still less than ttw - skip task
    if(s_running->ttw && (t-s_running->t_yield) < s_running->ttw) goto next;

//     main task always    task max execution time more than we have
    if(s_running->id!=0 && s_running->delay > ttw) { 
        s_running->delay --; //  понемногу уменьшаем дабы совсем не выключить
        goto next;
    }

    me = s_running; // switch to

    me->ttw=0; // time to wait is over
    me->start = micros(); // task startup time
#ifdef MTASK_PROF
    me->ticks = stopwatch_getticks();
#endif
    longjmp(me->context, true);
    // never comes here
}


/**
   * Return current task stack size.
   * @return bytes
 */
size_t task_stack(){
  unsigned char marker;
  return (&marker - s_running->stack);
}

