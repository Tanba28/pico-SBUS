#include "pico/stdlib.h"
#include "sbus.hpp"
#include "stdio.h"
#include "task_base.hpp"

class SbusTask : public TaskBase{
    public:
        SbusTask():TaskBase("sbus_task",3,256){
            this->createTask();
        }
    private:
        void task() override{
            TickType_t last_wake_time;

            SBUS::createInstance(uart1,8,9);
            SBUS *sbus = SBUS::getInstance();

            last_wake_time = xTaskGetTickCount();

            for(;;){
                for(int j = 0;j<2000;j+=10){
                    for(uint8_t i=0;i<18;i++){
                        sbus->putChannelValue(i,j);
                    }
                    sbus->sendData();

                    gpio_put(25,!gpio_get(25));

                    printf("%d\n",j);

                    xTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(50));
                }
                for(int j = 2000;j>0;j-=10){
                    for(uint8_t i=0;i<18;i++){
                        sbus->putChannelValue(i,j);
                    }
                    sbus->sendData();

                    gpio_put(25,!gpio_get(25));

                    printf("%d\n",j);

                    xTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(50));
                }
            }
        }
};
int main(){
    stdio_init_all();

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    gpio_put(25,1);

    SbusTask sbus_task = SbusTask();

    vTaskStartScheduler();
    for(;;){

    }
}