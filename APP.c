#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
int main(int argc, char *argv[])
{

    short resive_data[6];  //保存收到的 mpu6050转换结果数据，依次为 AX(x轴角度), AY, AZ 。GX(x轴加速度), GY ,GZ
    bool enable_mpu = 0;
    int key_event;
    int error;
    /*打开文件*/
    int key_fd = open("/dev/key_irq", O_RDWR);
    int mpu_fd = open("/dev/MPU6050", O_RDWR);
    

    if(key_fd < 0)
    {
		printf("open file : %s failed !\n", argv[0]);
		return -1;
	}

    if(mpu_fd < 0)
    {
		printf("open file : %s failed !\n", argv[0]);
		return -1;
	}

    while(1){
        /* 阻塞方式检查有没有按键事件 */
        if (read(key_fd, &key_event, sizeof(key_event)) > 0) {
            enable_mpu = !enable_mpu;

            if (enable_mpu)
                printf("MPU6050 ON\n");
            else
                printf("MPU6050 OFF\n");
        }

        if (enable_mpu){
            int ret = read(mpu_fd, resive_data, sizeof(resive_data));
            if (ret == sizeof(resive_data)) {
            printf("AX=%d, AY=%d, AZ=%d ",(int)resive_data[0],(int)resive_data[1],(int)resive_data[2]);
	        printf("GX=%d, GY=%d, GZ=%d \n \n",(int)resive_data[3],(int)resive_data[4],(int)resive_data[5]);
        }
        usleep(100000);   // 100ms打印一次
        }
    }

    /*关闭文件*/
    error = close(key_fd);
    if(error < 0)
    {
        printf("key close file error! \n");
        return error;
    }

    error = close(mpu_fd);
    if(error < 0)
    {
        printf("key close file error! \n");
        return error;
    }
    
    return 0;
}