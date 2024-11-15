#include "watchdog.h"
#include "algorithm.h"

struct WatchDog watchDog_g;

// void handle_signal(int signal) {
//     printf("try exit\n");
//     pthread_exit(NULL);
// }

void handle_signal(int signum) {
    printf("Received signal %d\n", signum);
    longjmp(jumpBuffer, 1);
}

void WatchDogInit(){
    watchDog_g.worker_finished = 0;
}

void WatchDogFunc(pthread_t * thread_id_p, int threshold_us, int granularity_us){
    // 在线程结束，或者超过时间后，发送一个请求结束线程。每granularity_us检测一次。
    while (1) {
        usleep(granularity_us); // sleep for 100ms
        pthread_mutex_lock(&watchDog_g.terminate_mutex);
        if(watchDog_g.worker_finished){
            printf("already finished\n");
            pthread_join(*thread_id_p, NULL);
            pthread_mutex_unlock(&watchDog_g.terminate_mutex);
            return ;
        }
        pthread_mutex_unlock(&watchDog_g.terminate_mutex);


        pthread_mutex_lock(&watchDog_g.check_mutex);
        int duration = getDurationMicroseconds();
        printf("duration: %d\n", duration);
        if (duration > threshold_us) {
            printf("kill\n");
            pthread_kill(*thread_id_p, SIGUSR1);
            pthread_mutex_unlock(&watchDog_g.check_mutex);
            pthread_join(*thread_id_p, NULL);
            return;
        }
        pthread_mutex_unlock(&watchDog_g.check_mutex);
    }
}

void WatchDogWorkerFinish(){
    pthread_mutex_lock(&watchDog_g.terminate_mutex);
    watchDog_g.worker_finished = 1;
    pthread_mutex_unlock(&watchDog_g.terminate_mutex);
}