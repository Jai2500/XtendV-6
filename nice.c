#include "types.h"
#include "stat.h"
#include "user.h"
#include "fcntl.h"

int main(int argc, char *argv[])
{
    if(argc < 3){
        printf(2, "Usage: nice <pid> <priority>\n");
    }
    
    int pid = atoi(argv[1]);
    int priority = atoi(argv[2]);
    
    if(priority < 0 || priority > 100)
        printf(2, "Invalid priority\n");

    int status = set_priority(priority, pid);
    
    if(status < 0)
        printf(2, "nice: Invalid process\n");
    
    exit();
}