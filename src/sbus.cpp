#include "sbus.hpp"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/irq.h"


SBUS *SBUS::sbus = NULL;

static void txDmaHandler(){
    SBUS::getInstance()->txIrqHandler();
}
static void rxDmaHandler(){
    SBUS::getInstance()->rxIrqHandler();
}

void SBUS::txIrqHandler(){
    if(dma_hw->ints1 & 1u << tx_dma_chan){
        dma_hw->ints1 = 1u << tx_dma_chan;
        BaseType_t higher_priority_task_woken;
        
        higher_priority_task_woken = pdFALSE;   
        
        xSemaphoreGiveFromISR(tx_irq_semaphor, &higher_priority_task_woken);
        
        portEND_SWITCHING_ISR( higher_priority_task_woken );    
    }
}
void SBUS::rxIrqHandler(){
    if(dma_hw->ints1 & 1u << rx_dma_chan){
        dma_hw->ints1 = 1u << rx_dma_chan;
        BaseType_t higher_priority_task_woken;

        higher_priority_task_woken = pdFALSE;   

        xSemaphoreGiveFromISR(rx_irq_semaphor, &higher_priority_task_woken);

        portEND_SWITCHING_ISR( higher_priority_task_woken );
    } 
}

void SBUS::createInstance(uart_inst_t *_uart_id,uint _gpio_tx,uint _gpio_rx){
    if(SBUS::sbus == NULL){
        SBUS::sbus = new SBUS(_uart_id,_gpio_tx,_gpio_rx);
    }
}
SBUS* SBUS::getInstance(){
    return SBUS::sbus;
}

SBUS::SBUS(uart_inst_t *_uart_id,uint _gpio_tx,uint _gpio_rx):
uart_id(_uart_id){
    uart_init(uart_id, 100*1000);
    gpio_set_function(_gpio_tx,GPIO_FUNC_UART);
    gpio_set_function(_gpio_rx,GPIO_FUNC_UART);

    uart_set_baudrate(uart_id,9600);
    uart_set_format(uart_id,8,2,UART_PARITY_EVEN);

    rx_irq_semaphor = xSemaphoreCreateBinary();
    tx_irq_semaphor = xSemaphoreCreateBinary();

    /* TX DMA SETTINGS */
    tx_dma_chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_transfer_data_size(&c,DMA_SIZE_8);
    channel_config_set_read_increment(&c,true);
    channel_config_set_write_increment(&c,false);
    channel_config_set_dreq(&c,uart_get_dreq(uart_id,true));

    dma_channel_configure(
        tx_dma_chan,
        &c,
        &uart_get_hw(uart_id)->dr,
        NULL,
        25,
        false
    );

    dma_channel_set_irq1_enabled(tx_dma_chan,true);
    irq_add_shared_handler(DMA_IRQ_1,txDmaHandler,PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    /* RX DMA SETTINGS */
    rx_dma_chan = dma_claim_unused_channel(true);

    c = dma_channel_get_default_config(rx_dma_chan);
    channel_config_set_transfer_data_size(&c,DMA_SIZE_8);
    channel_config_set_read_increment(&c,false);
    channel_config_set_write_increment(&c,true);
    channel_config_set_dreq(&c,uart_get_dreq(uart_id,false));

    dma_channel_configure(
        rx_dma_chan,
        &c,
        NULL,
        &uart_get_hw(uart_id)->dr,
        25,
        false
    );

    dma_channel_set_irq1_enabled(rx_dma_chan,true);
    irq_add_shared_handler(DMA_IRQ_1,rxDmaHandler,PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_1,true);

}

void SBUS::putChannelValue(uint8_t channel_num,uint16_t value){
    send_data.channel[channel_num] = value;
}
uint16_t SBUS::getChannelValue(uint8_t channel_num){
    return read_data.channel[channel_num];
}

void SBUS::sendData(){
    Raw raw;
    raw = encode(send_data);
    // for(uint8_t i = 0;i<25;i++){
    //     uart_putc_raw(uart_id,raw.byte[i]);
    // }
    dma_channel_set_read_addr(tx_dma_chan,raw.byte,true);
    xSemaphoreTake(tx_irq_semaphor,portMAX_DELAY);
}

void SBUS::readRaw(){
    Raw raw;
    dma_channel_set_write_addr(rx_dma_chan,&raw.byte[0],true);
    xSemaphoreTake(rx_irq_semaphor,portMAX_DELAY);

    read_data = decode(raw);
}

SBUS::Raw SBUS::encode(Data data){
    Raw raw;
    raw.byte[0] = 0x0F;
    
    raw.byte[1] =   data.channel[0];
    raw.byte[2] = ((data.channel[0] >> 8) & 0x07) | (data.channel[1] << 3);
    raw.byte[3] = ((data.channel[1] >> 5) & 0x3F) | (data.channel[2] << 6);
    raw.byte[4] =   data.channel[2] >> 2;
    raw.byte[5] = ((data.channel[2] >>10) & 0x01) | (data.channel[3] << 1);
    raw.byte[6] = ((data.channel[3] >> 7) & 0x0F) | (data.channel[4] << 4);
    raw.byte[7] = ((data.channel[4] >> 4) & 0x7F) | (data.channel[5] << 7);
    raw.byte[8] =   data.channel[5] >> 1;
    raw.byte[9] = ((data.channel[5] >> 9) & 0x03) | (data.channel[6] << 2);
    raw.byte[10]= ((data.channel[6] >> 2) & 0x1F) | (data.channel[7] << 5);
    raw.byte[11]=   data.channel[7] >> 3;    
    raw.byte[12]=   data.channel[8];
    raw.byte[13]= ((data.channel[8] >> 8) & 0x07) | (data.channel[9] << 3);
    raw.byte[14]= ((data.channel[9] >> 5) & 0x3F) | (data.channel[10] << 6);
    raw.byte[15]=   data.channel[10] >> 2;
    raw.byte[16]= ((data.channel[10] >>10) & 0x01) | (data.channel[11] << 1);
    raw.byte[17]= ((data.channel[11] >> 7) & 0x0F) | (data.channel[12] << 4);
    raw.byte[18]= ((data.channel[12] >> 4) & 0x7F) | (data.channel[13] << 7);
    raw.byte[19]=   data.channel[13] >> 1;
    raw.byte[20]= ((data.channel[13] >> 9) & 0x03) | (data.channel[14] << 2);
    raw.byte[21]= ((data.channel[14] >> 2) & 0x1F) | (data.channel[15] << 5);
    raw.byte[22]=   data.channel[15] >> 3;
    
    raw.byte[23] = (data.channel[16]) | (data.channel[17] << 1);
    
    raw.byte[24] = 0x00; 
    
    return raw;
}
SBUS::Data SBUS::decode(Raw raw){
    Data data;
    
    data.channel[0]  = (((uint16_t)raw.byte[2] &0x07)<<8)  | (((uint16_t)raw.byte[1]&0xFF));
    data.channel[1]  = (((uint16_t)raw.byte[3] &0x3F)<<5)  | (((uint16_t)raw.byte[2]&0xF8)>>3);
    data.channel[2]  = (((uint16_t)raw.byte[5] &0x01)<<10) | (((uint16_t)raw.byte[4]&0xFF)<<2) | (((uint16_t)raw.byte[3]&0xC0)>>6);
    data.channel[3]  = (((uint16_t)raw.byte[6] &0x0F)<<7)  | (((uint16_t)raw.byte[5]&0xFE)>>1);
    data.channel[4]  = (((uint16_t)raw.byte[7] &0x7F)<<4)  | (((uint16_t)raw.byte[6]&0xF0)>>4);
    data.channel[5]  = (((uint16_t)raw.byte[9]&0x03)<<9)  | (((uint16_t)raw.byte[8]&0xFF))    | (((uint16_t)raw.byte[7]&0x80)>>7);
    data.channel[6]  = (((uint16_t)raw.byte[10]&0x1F)<<6)  | (((uint16_t)raw.byte[9]&0xFC)>>2);
    data.channel[7]  = (((uint16_t)raw.byte[11]&0xFF)<<3)  | (((uint16_t)raw.byte[10]&0xE0)>>5);
    data.channel[8]  = (((uint16_t)raw.byte[13]&0x07)<<8)  | (((uint16_t)raw.byte[12]&0xFF));
    data.channel[9] = (((uint16_t)raw.byte[14]&0x3F)<<5)  | (((uint16_t)raw.byte[13]&0xF8)>>3);
    data.channel[10] = (((uint16_t)raw.byte[16]&0x01)<<10) | (((uint16_t)raw.byte[15]&0xFF)<<2) | (((uint16_t)raw.byte[14]&0xC0)>>6);
    data.channel[11] = (((uint16_t)raw.byte[17]&0x0F)<<7)  | (((uint16_t)raw.byte[16]&0xFE)>>1);
    data.channel[12] = (((uint16_t)raw.byte[18]&0x7F)<<4)  | (((uint16_t)raw.byte[17]&0xF0)>>4);
    data.channel[13] = (((uint16_t)raw.byte[20]&0x03)<<9)  | (((uint16_t)raw.byte[19]&0xFF))    | (((uint16_t)raw.byte[18]&0x80)>>7);
    data.channel[14] = (((uint16_t)raw.byte[21]&0x1F)<<6)  | (((uint16_t)raw.byte[20]&0xFC)>>2);
    data.channel[15] = (((uint16_t)raw.byte[22]&0xFF)<<3)  | (((uint16_t)raw.byte[21]&0xE0)>>5);
    data.channel[16] = (raw.byte[23]&0x80)>>7;
    data.channel[17] = (raw.byte[23]&0x40)>>6;
    
    return data;       
}