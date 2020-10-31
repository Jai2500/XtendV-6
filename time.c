#include "types.h"
#include "stat.h"
#include "user.h"
#include "fcntl.h"

int main(int argc, char *argv[])
{
    if(argc <= 1){
        printf(2, "Incorrect Usage: time [...args]\n");
        exit();
    }

    int childpid = fork();
    if(childpid < 0){
        printf(2, "fork() failed\n");
        exit();
    }
    else if(childpid == 0) {
        exec(argv[1], &argv[1]);
        printf(2, "exec() failed\n");
        exit();
    }
    else if(childpid > 0){
        int rtime, wtime;
        waitx(&wtime, &rtime);
        printf(1, "rtime = %d, wtime = %d\n", rtime, wtime);
        exit();
    }
    // exit();
}