#include "pico/stdlib.h"
#include "sbus.hpp"
#include "stdio.h"
#include "task_base.hpp"

class SbusSendTask : public TaskBase{
    public:
        SbusSendTask():TaskBase("sbus_send_task",3,256){
            this->createTask();
            SBUS::createInstance(uart1,4,5);
        }
    private:
        void task() override{
            TickType_t last_wake_time;

            SBUS *sbus = SBUS::getInstance();

            last_wake_time = xTaskGetTickCount();

            for(;;){
                for(int j = 0;j<2000;j+=10){
                    for(uint8_t i=0;i<18;i++){
                        sbus->putChannelValue(i,j);
                    }
                    sbus->sendData();

                    gpio_put(18,!gpio_get(18));

                    printf("[SEND]%d\n",j);

                    xTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(10));
                }
                for(int j = 2000;j>0;j-=10){
                    for(uint8_t i=0;i<18;i++){
                        sbus->putChannelValue(i,j);
                    }
                    sbus->sendData();

                    gpio_put(18,!gpio_get(18));

                    printf("[SEND]%d\n",j);

                    xTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(10));
                }
            }
        }
};

class SbusReadTask : public TaskBase{
    public:
        SbusReadTask():TaskBase("sbus_read_task",3,256){
            this->createTask();
        }
    private:
        void task() override{
            SBUS *sbus = SBUS::getInstance();

            uint16_t value[18];

            for(;;){
                sbus->readRaw();

                for(int i=0;i<18;i++){
                    value[i] = sbus->getChannelValue(i);
                }
                printf("[READ]%d\n",value[0]);
            }
        }
};
int main(){
    stdio_init_all();

    gpio_init(18);
    gpio_init(19);
    gpio_init(20);
    gpio_set_dir(18, GPIO_OUT);
    gpio_set_dir(19, GPIO_OUT);
    gpio_set_dir(20, GPIO_OUT);

    gpio_put(18,0);
    gpio_put(19,1);
    gpio_put(20,1);

    SbusSendTask sbus_send_task = SbusSendTask();
    SbusReadTask sbus_read_task = SbusReadTask();

    vTaskStartScheduler();
    for(;;){

    }
}