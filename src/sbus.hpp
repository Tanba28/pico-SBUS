#ifndef __PICO_SBUS__
#define __PICO_SBUS__

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

class SBUS{
    public:
        static void createInstance(uart_inst_t *_uart_id,uint _gpio_tx,uint _gpio_rx);
        static SBUS* getInstance();

        void putChannelValue(uint8_t channel_num,uint16_t value);
        uint16_t getChannelValue(uint8_t channel_num);
        void sendData();
        void readRaw();

        void txIrqHandler();
        void rxIrqHandler();
        
    private:
        SBUS(uart_inst_t *_uart_id,uint _gpio_tx,uint _gpio_rx);
        static SBUS *sbus;

        struct Data{
            uint16_t channel[18];
        };
        struct Raw{
            uint8_t byte[25];
        };

        Raw encode(Data data);
        Data decode(Raw raw);

        Data send_data;
        Data read_data;
        uart_inst_t *uart_id;

        int tx_dma_chan;
        int rx_dma_chan;

        SemaphoreHandle_t rx_irq_semaphor;
        SemaphoreHandle_t tx_irq_semaphor;
};
#endif