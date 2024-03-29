// Per-CPU state
struct cpu {
  uchar apicid;                // Local APIC ID
  struct context *scheduler;   // swtch() here to enter scheduler
  struct taskstate ts;         // Used by x86 to find stack for interrupt
  struct segdesc gdt[NSEGS];   // x86 global descriptor table
  volatile uint started;       // Has the CPU started?
  int ncli;                    // Depth of pushcli nesting.
  int intena;                  // Were interrupts enabled before pushcli?
  struct proc *proc;           // The process running on this cpu or null
};

extern struct cpu cpus[NCPU];
extern int ncpu;

// Per process-Queue state
struct qstats {
  int qtime[NQUE];             // Queue times for each process
  int actualqtime[NQUE];       // Actual Queue times for each process
};

//PAGEBREAK: 17
// Saved registers for kernel context switches.
// Don't need to save all the segment registers (%cs, etc),
// because they are constant across kernel contexts.
// Don't need to save %eax, %ecx, %edx, because the
// x86 convention is that the caller has saved them.
// Contexts are stored at the bottom of the stack they
// describe; the stack pointer is the address of the context.
// The layout of the context matches the layout of the stack in swtch.S
// at the "Switch stacks" comment. Switch doesn't save eip explicitly,
// but it is on the stack and allocproc() manipulates it.
struct context {
  uint edi;
  uint esi;
  uint ebx;
  uint ebp;
  uint eip;
};

enum procstate { UNUSED, EMBRYO, SLEEPING, RUNNABLE, RUNNING, ZOMBIE };

// Per-process state
struct proc {
  uint sz;                     // Size of process memory (bytes)
  pde_t* pgdir;                // Page table
  char *kstack;                // Bottom of kernel stack for this process
  enum procstate state;        // Process state
  int pid;                     // Process ID
  struct proc *parent;         // Parent process
  struct trapframe *tf;        // Trap frame for current syscall
  struct context *context;     // swtch() here to run process
  void *chan;                  // If non-zero, sleeping on chan
  int killed;                  // If non-zero, have been killed
  struct file *ofile[NOFILE];  // Open files
  struct inode *cwd;           // Current directory
  char name[16];               // Process name (debugging)
  uint ctime;                  // Process creation time
  uint etime;                  // Process end time
  uint rtime;                  // Process run time
  uint priority;               // Process priority
  uint lastqtime;              // Process time of joining the latest queue
  struct qstats stat;          // Prrocess stats of queues for MLFQ
  uint nruns;                  // Number of times the process has been scheduled
  int qno;
  int qpos;
};

// #ifdef MLFQ
// Define the Queue struct that is used for MLFQ
struct Queue {
  int start;
  int end;
  uint size;

  struct proc *list[NPROC];
  int next[NPROC];
};

struct Queue ProcQueues[NQUE];

void
pushback(int qno, struct proc *p);

struct proc*
pop(int qno);

// int 
// deletefromqueue(struct Queue *queue, int id);

// int
// findid(struct Queue *queue, int procpid);

// int
// removeproc(struct Queue *queue, struct proc *p);

struct proc*
front(int qno);

int
back(int qno);

int
getmyqpos(struct proc *p);

int
getmyqno(struct proc *p);

int
getmyqtime(struct proc *p);

void
updatePriority(struct proc* p, int shouldincrease);

// #endif



// Process memory is laid out contiguously, low addresses first:
//   text
//   original data and bss
//   fixed-size stack
//   expandable heap

// Updates running processes
void updaterunprocs();

// Returns whether a higher priority process has arrived
int yieldhigherprior(int);

void ageprocs();