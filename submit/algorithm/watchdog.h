#ifndef WATCHDOG_H
#define WATCHDOG_H
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <sys/time.h>
#include <stdint.h>
#include <setjmp.h>


struct WatchDog{
    pthread_mutex_t check_mutex;
    pthread_mutex_t terminate_mutex;
    int worker_finished;
};

jmp_buf jumpBuffer;


void handle_signal(int signal);

extern struct WatchDog watchDog_g;

void WatchDogInit();

void WatchDogFunc(pthread_t * thread_id_p, int threshold_us, int granularity_us);

void WatchDogWorkerFinish();

#ifdef __cplusplus
}
#endif

#endif