#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <errno.h>

static int key_fd = -1;
static int mpu_fd = -1;

static bool enable_mpu = false;
static pthread_mutex_t mpu_lock = PTHREAD_MUTEX_INITIALIZER;

/* 按键线程：阻塞等待按键事件 */
void *key_thread_func(void *arg)
{
    int key_event;
    int ret;

    (void)arg;

    while (1) {
        ret = read(key_fd, &key_event, sizeof(key_event));
        if (ret == sizeof(key_event)) {
            pthread_mutex_lock(&mpu_lock);
            enable_mpu = !enable_mpu;
            printf("MPU6050 %s\n", enable_mpu ? "ON" : "OFF");
            pthread_mutex_unlock(&mpu_lock);
        } else if (ret < 0) {
            perror("read key_fd failed");
            usleep(10000);
        }
    }

    return NULL;
}

/* MPU线程：按开关状态周期性读取MPU6050 */
void *mpu_thread_func(void *arg)
{
    short receive_data[6];
    int ret;
    bool local_enable;

    (void)arg;

    while (1) {
        pthread_mutex_lock(&mpu_lock);
        local_enable = enable_mpu;
        pthread_mutex_unlock(&mpu_lock);

        if (local_enable) {
            ret = read(mpu_fd, receive_data, sizeof(receive_data));
            if (ret == sizeof(receive_data)) {
                printf("AX=%d, AY=%d, AZ=%d ",
                       (int)receive_data[0],
                       (int)receive_data[1],
                       (int)receive_data[2]);
                printf("GX=%d, GY=%d, GZ=%d\n\n",
                       (int)receive_data[3],
                       (int)receive_data[4],
                       (int)receive_data[5]);
            } else if (ret < 0) {
                perror("read mpu_fd failed");
            }

            usleep(100000);   // 100ms读取一次
        } else {
            usleep(10000);    // 关闭时小睡，避免空转
        }
    }

    return NULL;
}

int main(int argc, char *argv[])
{
    pthread_t key_tid;
    pthread_t mpu_tid;
    int ret;

    (void)argc;
    (void)argv;

    key_fd = open("/dev/KEY_IRQ", O_RDWR);
    if (key_fd < 0) {
        perror("open /dev/key_irq failed");
        return -1;
    }

    mpu_fd = open("/dev/MPU6050", O_RDWR);
    if (mpu_fd < 0) {
        perror("open /dev/MPU6050 failed");
        close(key_fd);
        return -1;
    }

    ret = pthread_create(&key_tid, NULL, key_thread_func, NULL);
    if (ret != 0) {
        perror("pthread_create key thread failed");
        close(key_fd);
        close(mpu_fd);
        return -1;
    }

    ret = pthread_create(&mpu_tid, NULL, mpu_thread_func, NULL);
    if (ret != 0) {
        perror("pthread_create mpu thread failed");
        close(key_fd);
        close(mpu_fd);
        return -1;
    }

    pthread_join(key_tid, NULL);
    pthread_join(mpu_tid, NULL);

    close(key_fd);
    close(mpu_fd);

    return 0;
}