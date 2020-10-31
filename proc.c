#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "x86.h"
#include "proc.h"
#include "spinlock.h"

struct {
  struct spinlock lock;
  struct proc proc[NPROC];
} ptable;

static struct proc *initproc;

int nextpid = 1;
extern void forkret(void);
extern void trapret(void);

static void wakeup1(void *chan);

void
pinit(void)
{
  initlock(&ptable.lock, "ptable");
}

// Must be called with interrupts disabled
int
cpuid() {
  return mycpu()-cpus;
}

// Must be called with interrupts disabled to avoid the caller being
// rescheduled between reading lapicid and running through the loop.
struct cpu*
mycpu(void)
{
  int apicid, i;
  
  if(readeflags()&FL_IF)
    panic("mycpu called with interrupts enabled\n");
  
  apicid = lapicid();
  // APIC IDs are not guaranteed to be contiguous. Maybe we should have
  // a reverse map, or reserve a register to store &cpus[i].
  for (i = 0; i < ncpu; ++i) {
    if (cpus[i].apicid == apicid)
      return &cpus[i];
  }
  panic("unknown apicid\n");
}

// Disable interrupts so that we are not rescheduled
// while reading proc from the cpu structure
struct proc*
myproc(void) {
  struct cpu *c;
  struct proc *p;
  pushcli();
  c = mycpu();
  p = c->proc;
  popcli();
  return p;
}

//PAGEBREAK: 32
// Look in the process table for an UNUSED proc.
// If found, change state to EMBRYO and initialize
// state required to run in the kernel.
// Otherwise return 0.
static struct proc*
allocproc(void)
{
  struct proc *p;
  char *sp;

  acquire(&ptable.lock);

  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    if(p->state == UNUSED)
      goto found;

  release(&ptable.lock);
  return 0;

found:
  p->state = EMBRYO;
  p->pid = nextpid++;
  p->ctime = ticks;
  p->etime = -1;
  p->rtime = 0;
  p->priority = 60; // Default priority

#ifdef MLFQ
  pushback(ProcQueues[0], p); // Pushback to the highest priority queue
  p->lastqtime = ticks;
#endif

  release(&ptable.lock);

  // Allocate kernel stack.
  if((p->kstack = kalloc()) == 0){
    p->state = UNUSED;
    return 0;
  }
  sp = p->kstack + KSTACKSIZE;

  // Leave room for trap frame.
  sp -= sizeof *p->tf;
  p->tf = (struct trapframe*)sp;

  // Set up new context to start executing at forkret,
  // which returns to trapret.
  sp -= 4;
  *(uint*)sp = (uint)trapret;

  sp -= sizeof *p->context;
  p->context = (struct context*)sp;
  memset(p->context, 0, sizeof *p->context);
  p->context->eip = (uint)forkret;

  return p;
}

//PAGEBREAK: 32
// Set up first user process.
void
userinit(void)
{
  struct proc *p;
  extern char _binary_initcode_start[], _binary_initcode_size[];

  p = allocproc();
  
  initproc = p;
  if((p->pgdir = setupkvm()) == 0)
    panic("userinit: out of memory?");
  inituvm(p->pgdir, _binary_initcode_start, (int)_binary_initcode_size);
  p->sz = PGSIZE;
  memset(p->tf, 0, sizeof(*p->tf));
  p->tf->cs = (SEG_UCODE << 3) | DPL_USER;
  p->tf->ds = (SEG_UDATA << 3) | DPL_USER;
  p->tf->es = p->tf->ds;
  p->tf->ss = p->tf->ds;
  p->tf->eflags = FL_IF;
  p->tf->esp = PGSIZE;
  p->tf->eip = 0;  // beginning of initcode.S

  safestrcpy(p->name, "initcode", sizeof(p->name));
  p->cwd = namei("/");

  // this assignment to p->state lets other cores
  // run this process. the acquire forces the above
  // writes to be visible, and the lock is also needed
  // because the assignment might not be atomic.
  acquire(&ptable.lock);

  p->state = RUNNABLE;

  release(&ptable.lock);
}

// Grow current process's memory by n bytes.
// Return 0 on success, -1 on failure.
int
growproc(int n)
{
  uint sz;
  struct proc *curproc = myproc();

  sz = curproc->sz;
  if(n > 0){
    if((sz = allocuvm(curproc->pgdir, sz, sz + n)) == 0)
      return -1;
  } else if(n < 0){
    if((sz = deallocuvm(curproc->pgdir, sz, sz + n)) == 0)
      return -1;
  }
  curproc->sz = sz;
  switchuvm(curproc);
  return 0;
}

// Create a new process copying p as the parent.
// Sets up stack to return as if from system call.
// Caller must set state of returned proc to RUNNABLE.
int
fork(void)
{
  int i, pid;
  struct proc *np;
  struct proc *curproc = myproc();

  // Allocate process.
  if((np = allocproc()) == 0){
    return -1;
  }

  // Copy process state from proc.
  if((np->pgdir = copyuvm(curproc->pgdir, curproc->sz)) == 0){
    kfree(np->kstack);
    np->kstack = 0;
    np->state = UNUSED;
    return -1;
  }
  np->sz = curproc->sz;
  np->parent = curproc;
  *np->tf = *curproc->tf;

  // Clear %eax so that fork returns 0 in the child.
  np->tf->eax = 0;

  for(i = 0; i < NOFILE; i++)
    if(curproc->ofile[i])
      np->ofile[i] = filedup(curproc->ofile[i]);
  np->cwd = idup(curproc->cwd);

  safestrcpy(np->name, curproc->name, sizeof(curproc->name));

  pid = np->pid;

  acquire(&ptable.lock);

  np->state = RUNNABLE;

  release(&ptable.lock);

  return pid;
}

// Exit the current process.  Does not return.
// An exited process remains in the zombie state
// until its parent calls wait() to find out it exited.
void
exit(void)
{
  struct proc *curproc = myproc();
  struct proc *p;
  int fd;

  if(curproc == initproc)
    panic("init exiting");

  // Close all open files.
  for(fd = 0; fd < NOFILE; fd++){
    if(curproc->ofile[fd]){
      fileclose(curproc->ofile[fd]);
      curproc->ofile[fd] = 0;
    }
  }

  begin_op();
  iput(curproc->cwd);
  end_op();
  curproc->cwd = 0;

  curproc->etime = ticks;

  acquire(&ptable.lock);

  // Parent might be sleeping in wait().
  wakeup1(curproc->parent);

  // Pass abandoned children to init.
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->parent == curproc){
      p->parent = initproc;
      if(p->state == ZOMBIE)
        wakeup1(initproc);
    }
  }

  // Jump into the scheduler, never to return.
  curproc->state = ZOMBIE;
  sched();
  panic("zombie exit");
}

// Wait for a child process to exit and return its pid.
// Return -1 if this process has no children.
int
wait(void)
{
  struct proc *p;
  int havekids, pid;
  struct proc *curproc = myproc();
  
  acquire(&ptable.lock);
  for(;;){
    // Scan through table looking for exited children.
    havekids = 0;
    for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
      if(p->parent != curproc)
        continue;
      havekids = 1;
      if(p->state == ZOMBIE){
        // Found one.
        pid = p->pid;
        kfree(p->kstack);
        p->kstack = 0;
        freevm(p->pgdir);
        p->pid = 0;
        p->parent = 0;
        p->name[0] = 0;
        p->killed = 0;
        p->state = UNUSED;
        release(&ptable.lock);
        return pid;
      }
    }

    // No point waiting if we don't have any children.
    if(!havekids || curproc->killed){
      release(&ptable.lock);
      return -1;
    }

    // Wait for children to exit.  (See wakeup1 call in proc_exit.)
    sleep(curproc, &ptable.lock);  //DOC: wait-sleep
  }
}

// WaitX implementation
int
waitx(int *wtime, int *rtime)
{
  struct proc *p;
  int havekids, pid;
  struct proc *curproc = myproc();

  acquire(&ptable.lock);
  for(;;){
    // Scan through the table looking for children
    havekids = 0;
    for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
      if(p->parent != curproc)
        continue;
      havekids = 1;
      if(p->state == ZOMBIE){
        // Found one 
        pid = p->pid;
        *rtime = p->rtime;
        *wtime = p->etime - p->ctime - p->rtime;
        kfree(p->kstack);
        p->kstack = 0;
        freevm(p->pgdir);
        p->pid = 0;
        p->parent = 0;
        p->name[0] = 0;
        p->killed = 0;
        p->state = UNUSED;
        release(&ptable.lock);
        return pid;
      }
    }

    // No point waiting if no children
    if(!havekids || curproc->killed){
      release(&ptable.lock);
      return -1;
    }

    // Wait for children to exit
    sleep(curproc, &ptable.lock);
  }
}

//PAGEBREAK: 42
// Per-CPU process scheduler.
// Each CPU calls scheduler() after setting itself up.
// Scheduler never returns.  It loops, doing:
//  - choose a process to run
//  - swtch to start running that process
//  - eventually that process transfers control
//      via swtch back to the scheduler.
// void
// scheduler(void)
// {
//   struct proc *p;
//   struct cpu *c = mycpu();
//   c->proc = 0;
  
//   for(;;){
//     // Enable interrupts on this processor.
//     sti();

//     // Loop over process table looking for process to run.
//     acquire(&ptable.lock);
//     for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
//       if(p->state != RUNNABLE)
//         continue;

//       // Switch to chosen process.  It is the process's job
//       // to release ptable.lock and then reacquire it
//       // before jumping back to us.
//       c->proc = p;
//       switchuvm(p);
//       p->state = RUNNING;

//       swtch(&(c->scheduler), p->context);
//       switchkvm();

//       // Process is done running for now.
//       // It should have changed its p->state before coming back.
//       c->proc = 0;
//     }
//     release(&ptable.lock);

//   }
// }

// FCFS scheduler
void
fcfsscheduler(void)
{
  struct proc *p;
  struct cpu *c = mycpu();
  c->proc = 0;

  for(;;){
    // Enable interrupts
    sti(); // Disabling may prevent pre-emptive
    
    struct proc *firstproc = 0;
    // Loop over process table looking for process to run
    acquire(&ptable.lock);
    for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
      if(p->state != RUNNABLE)
        continue;

      if(firstproc == 0)
        firstproc = p;
      else if(p->ctime < firstproc->ctime)
        firstproc = p;
    }
     
    if(firstproc == 0){
      release(&ptable.lock); // Check whether I can release it here.
      continue;
    }

    c->proc = firstproc;
    switchuvm(firstproc);
    firstproc->state = RUNNING;    
    
    swtch(&(c->scheduler), firstproc->context);
    switchkvm();
    
    c->proc = 0;    
    
    release(&ptable.lock);
  }
}

// Priority Scheduler
void
scheduler(void)
{
  struct proc *p;
  struct cpu *c = mycpu();
  c->proc = 0;

  for(;;){
    // Allow the cpu to be interrupted
    sti();

    struct proc *highproc = 0;
    acquire(&ptable.lock);
    for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
      if(p->state != RUNNABLE)
        continue;

      if(highproc == 0)
        highproc = p;
      else if(p->priority < highproc->priority)
        highproc = p;
    }
    
    if(highproc == 0){
      release(&ptable.lock);
      continue;
    }

    c->proc = highproc;
    switchuvm(highproc);
    highproc->state = RUNNING;
    swtch(&(c->scheduler), highproc->context);
    switchkvm();

    c->proc = 0;

    release(&ptable.lock);
  }
}

// MLFQ scheduler
int
mlfqscheduler(void)
{
  struct proc *finalp = 0;
  struct cpu *c = mycpu();
  c->proc = 0;
  for(;;){
    acquire(&ptable.lock);
    for(int i = 0; i < NQUE; i++){
      int pos = ProcQueues[i]->start;
      struct proc *p = ProcQueues[i]->list[pos];
      while (p)
      {
        if(p->killed || p->pid == 0 || p->state != RUNNABLE){ // Check whether the process is dead
          pos = ProcQueues[i]->next[pos];
          p = ProcQueues[i]->list[pos];
          pop(ProcQueues[i]);
        }
        else{
          finalp = p;
          break;
        }
      }
      if(finalp)
        break;
    }

    if(finalp == 0){
      release(&ptable.lock);
      continue;
    }
    
    int qno = getmyqno(finalp);
    if(qno < 0)
      panic("Invalid queue for the process\n");

    removeproc(ProcQueues[qno], finalp);
    finalp->lastqtime = ticks;
    finalp->nruns++;

    c->proc = finalp;
    switchuvm(finalp);
    finalp->state = RUNNING;

    swtch(&(c->scheduler), finalp->context);
    // It comes to here after completing, 
    // The process has to be pushed into the queue again 
    // or pushed to the lower queue
    int qtime = getmyqtime(finalp);

    if((finalp->state == SLEEPING) || 
    (finalp->state == RUNNABLE && qtime < (1 << qno))){
      // pushback to the same queue
      pushback(ProcQueues[qno], finalp);
    }
    else if((finalp->state == RUNNABLE && qtime > (1 << qno))){
      // pushback to lower priority queue
      updatePriority(finalp, 0);
    }
    switchkvm();
    release(&ptable.lock);
  }
}

// Enter scheduler.  Must hold only ptable.lock
// and have changed proc->state. Saves and restores
// intena because intena is a property of this
// kernel thread, not this CPU. It should
// be proc->intena and proc->ncli, but that would
// break in the few places where a lock is held but
// there's no process.
void
sched(void)
{
  int intena;
  struct proc *p = myproc();

  if(!holding(&ptable.lock))
    panic("sched ptable.lock");
  if(mycpu()->ncli != 1)
    panic("sched locks");
  if(p->state == RUNNING)
    panic("sched running");
  if(readeflags()&FL_IF)
    panic("sched interruptible");
  intena = mycpu()->intena;
  swtch(&p->context, mycpu()->scheduler);
  mycpu()->intena = intena;
}

// Give up the CPU for one scheduling round.
void
yield(void)
{
  acquire(&ptable.lock);  //DOC: yieldlock
  myproc()->state = RUNNABLE;
  sched();
  release(&ptable.lock);
}

// A fork child's very first scheduling by scheduler()
// will swtch here.  "Return" to user space.
void
forkret(void)
{
  static int first = 1;
  // Still holding ptable.lock from scheduler.
  release(&ptable.lock);

  if (first) {
    // Some initialization functions must be run in the context
    // of a regular process (e.g., they call sleep), and thus cannot
    // be run from main().
    first = 0;
    iinit(ROOTDEV);
    initlog(ROOTDEV);
  }

  // Return to "caller", actually trapret (see allocproc).
}

// Atomically release lock and sleep on chan.
// Reacquires lock when awakened.
void
sleep(void *chan, struct spinlock *lk)
{
  struct proc *p = myproc();
  
  if(p == 0)
    panic("sleep");

  if(lk == 0)
    panic("sleep without lk");

  // Must acquire ptable.lock in order to
  // change p->state and then call sched.
  // Once we hold ptable.lock, we can be
  // guaranteed that we won't miss any wakeup
  // (wakeup runs with ptable.lock locked),
  // so it's okay to release lk.
  if(lk != &ptable.lock){  //DOC: sleeplock0
    acquire(&ptable.lock);  //DOC: sleeplock1
    release(lk);
  }
  // Go to sleep.
  p->chan = chan;
  p->state = SLEEPING;

  sched();

  // Tidy up.
  p->chan = 0;

  // Reacquire original lock.
  if(lk != &ptable.lock){  //DOC: sleeplock2
    release(&ptable.lock);
    acquire(lk);
  }
}

//PAGEBREAK!
// Wake up all processes sleeping on chan.
// The ptable lock must be held.
static void
wakeup1(void *chan)
{
  struct proc *p;

  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    if(p->state == SLEEPING && p->chan == chan){

#ifdef MLFQ
      pushback(getmyqno(p), p);
#endif

      p->state = RUNNABLE;
    }
}

// Wake up all processes sleeping on chan.
void
wakeup(void *chan)
{
  acquire(&ptable.lock);
  wakeup1(chan);
  release(&ptable.lock);
}

// Kill the process with the given pid.
// Process won't exit until it returns
// to user space (see trap in trap.c).
int
kill(int pid)
{
  struct proc *p;

  acquire(&ptable.lock);
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->pid == pid){
      p->killed = 1;
      // Wake process from sleep if necessary.
      if(p->state == SLEEPING)
        p->state = RUNNABLE;
      release(&ptable.lock);
      return 0;
    }
  }
  release(&ptable.lock);
  return -1;
}

//PAGEBREAK: 36
// Print a process listing to console.  For debugging.
// Runs when user types ^P on console.
// No lock to avoid wedging a stuck machine further.
void
procdump(void)
{
  static char *states[] = {
  [UNUSED]    "unused",
  [EMBRYO]    "embryo",
  [SLEEPING]  "sleep ",
  [RUNNABLE]  "runble",
  [RUNNING]   "run   ",
  [ZOMBIE]    "zombie"
  };
  int i;
  struct proc *p;
  char *state;
  uint pc[10];

  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->state == UNUSED)
      continue;
    if(p->state >= 0 && p->state < NELEM(states) && states[p->state])
      state = states[p->state];
    else
      state = "???";
    cprintf("%d %s %s", p->pid, state, p->name);
    if(p->state == SLEEPING){
      getcallerpcs((uint*)p->context->ebp+2, pc);
      for(i=0; i<10 && pc[i] != 0; i++)
        cprintf(" %p", pc[i]);
    }
    cprintf("\n");
  }
}

// Update the runtime for each of the running processes
void 
updaterunprocs()
{
  struct proc *p;
  acquire(&ptable.lock);
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->state == RUNNING)
      p->rtime++;
  } 
  release(&ptable.lock); 
}

// Print the process details
void
procdet(void)
{
  struct proc *p;
  static char *states[] = {
  [UNUSED]    "unused",
  [EMBRYO]    "embryo",
  [SLEEPING]  "sleep ",
  [RUNNABLE]  "runble",
  [RUNNING]   "run   ",
  [ZOMBIE]    "zombie"
  };
  cprintf("PID Priority State r_time w_time n_run\n");
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->state == UNUSED || p->killed)
      continue;
    cprintf("%d %d %s %d %d %d\n", p->pid, p->priority, states[p->state], p->rtime, p->etime == -1? ticks - p->ctime - p->rtime : p->etime - p->ctime - p->rtime, p->nruns);
  }
}

// Set priority
// Returns pid if successful else returns -1
int 
set_priority(int new_priority, int pid)
{
  short resched = 0;
  struct proc *p;
  acquire(&ptable.lock);
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->pid != pid)
      continue;

    if(new_priority < p->priority)
      resched = 1;
    p->priority = new_priority;
    release(&ptable.lock);
    if(resched)
      sched();
    return p->pid;
  } 
  release(&ptable.lock);
  return -1;
}

// Returns whether a process with a higher priority has arrived
int
yieldhigherprior(int priority)
{
  struct proc* p;
  acquire(&ptable.lock);
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p->priority <= priority){
      release(&ptable.lock);
      return 1;
    }
  }
  release(&ptable.lock);
  return 0;
}


// Age the processes within a queue
void
ageProc(){
  acquire(&ptable.lock);
  struct proc *p;
  for(p = ptable.proc; p < &ptable.proc[NPROC]; p++){
    if(p == 0 || p->pid == 0 || p->killed)
      continue;

    if(p->state == RUNNABLE || p->state == RUNNING){
      int qno = getmyqno(p);
      int qtime = getmyqtime(p);
      if(p->state == RUNNING)
        // Update the processes' running time within the particular queue
        p->stat.actualqtime[qno]++;
      
      uint waitime = 1;
      if(p->state == RUNNABLE && qtime >= waitime){
        // Increase the prirority of the proceess
        updatePriority(p, 1);
      } 
    }
  }
  release(&ptable.lock);
}

// Update the priority of a process
void
updatePriority(struct proc* p, int shouldincrease){
  if(p == 0)
    panic("Cannot update priority of invalid proccess");

  int qno = getmyqno(p);
  if(qno < 0)
    panic("Invalid queue num of the process");

  int newqno;
  if(shouldincrease)
    newqno = qno == 0 ? qno : qno - 1;
  else
    newqno = qno == NQUE? qno : qno + 1;
  
  if(newqno != qno){
    removeproc(ProcQueues[qno], p);
    pushback(ProcQueues[newqno], p);
  }
}


// Push element into the queue
void
pushback(struct Queue *queue, struct proc *p)
{
  int pos = -1;
  for(int i = 0; i < NPROC; i++){
    if(queue->list[i] == 0){
      pos = i;
      break;
    }
  }
  if(pos == -1)
    panic("Tried to add a process when the queue is full");

  queue->list[pos] = p;
  if(queue->start == -1)
    queue->start = pos;
  if(queue->end != -1)
    queue->next[queue->end] = pos;
  queue->size++;
}

// Pop element from the start of the queue
struct proc*
pop(struct Queue *queue)
{
  if(queue->list[queue->start] == 0 || queue->size == 0)
    panic("Tried to remove a process when queue was empty");
  
  struct proc *p = queue->list[queue->start];
  
  queue->list[queue->start] = 0;
  int oldstart = queue->start;
  queue->start = queue->next[queue->start];
  queue->next[oldstart] = -1;
  
  queue->size--;
  if(queue->size == 0)
    queue->end = -1;

  return p;
}

// Deletes element at particular place in queue
int 
deletefromqueue(struct Queue *queue, int id)
{
  if(id >= queue->size || id < 0)
    return -1;

  int posprevelem = -1;
  int poselem = queue->start;
  for(int i = 1; i < id; i++){
    posprevelem = poselem;
    poselem = queue->next[poselem];
  }

  queue->list[poselem] = 0;
  
  if(queue->end == poselem)
    queue->end = posprevelem;
  else if(queue->start == poselem)
    queue->start = queue->next[poselem];

  if(posprevelem != -1)
    queue->next[posprevelem] = queue->next[poselem];
  
  queue->next[poselem] = -1;
  queue->size--;
  if(queue->size == 0)
    queue->end = -1;
  
  return poselem;
}

// Find idx of process given by pid
int
findid(struct Queue *queue, int procpid)
{
  int found = 0;
  int count = 0;
  int pos = queue->start;

  while (queue->list[pos] != 0)
  {
    if(queue->list[pos]->pid == procpid){
      found = 1;
      break;
    }
    pos = queue->next[pos];
    count++;
  }

  if(found)
    return count;
  else
    return -1;
}

// Remove process from queue
int
removeproc(struct Queue *queue, struct proc *p){
  int id = findid(queue, p->pid);
  if(id < 0)
    panic("Cannot find process within the queue");

  return deletefromqueue(queue, id);
}

// Returns the qno of the process
int
getmyqno(struct proc *p){
  for(int i = 0; i < NQUE; i++){
    struct Queue *currq  = ProcQueues[i];
    int id = findid(currq, p->pid);
    if(id >= 0)
      return i;
  }  
  return -1;
}

// Returns the time within the last queue
int
getmyqtime(struct proc *p){
  return ticks - p->lastqtime;
}