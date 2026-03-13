#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/adc.h"


// For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi


// Gyro SPI (spi0 bus)
#define GYRO_SPI spi0
#define GYRO_MISO 16
#define GYRO_CS   17
#define GYRO_SCK  18
#define GYRO_MOSI 19


//Magnometer SPI (spi1 bus)
#define MAG_SPI spi1
#define MAG_MISO 12
#define MAG_CS   13
#define MAG_SCK  10
#define MAG_MOSI 11

#define ACC_X 26
#define ACC_Y 27
#define ACC_Z 28

#define BUFFER_SIZE 3*64
uint16_t adc_buffer[BUFFER_SIZE];
int current_adc_channel = 0;


uint16_t gyro[3];
int accel[3];
int angle[3];
uint16_t mag[3];

int dma_acc, dma0_tx, dma0_rx, dma1_tx, dma1_rx;

void dma_acc_handler() {
    // Clear interrupt
    dma_hw->ints0 = 1u << dma_acc;

    // Rotate ADC channel
    current_adc_channel = (current_adc_channel + 1) % 3;
    adc_select_input(current_adc_channel);

    // Restart DMA for next batch
    dma_channel_set_read_addr(dma_acc, &adc_hw->fifo, true);
}


int main()
{
    stdio_init_all();


    // Gyro at 8MHz, Mag at 10MHz
    spi_init(GYRO_SPI, 8000*1000);
    spi_init(MAG_SPI, 10000*1000);

    adc_init();

    // Setup all the SPI Pins for the Gyro and Mag
    gpio_set_function(GYRO_MISO, GPIO_FUNC_SPI);
    gpio_set_function(GYRO_CS,   GPIO_FUNC_SIO);
    gpio_set_function(GYRO_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(GYRO_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MAG_MISO, GPIO_FUNC_SPI);
    gpio_set_function(MAG_CS,   GPIO_FUNC_SIO);
    gpio_set_function(MAG_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(MAG_MOSI, GPIO_FUNC_SPI);
   
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(GYRO_CS, GPIO_OUT);
    gpio_set_dir(MAG_CS, GPIO_OUT);
    gpio_put(GYRO_CS, 1);
    gpio_put(MAG_CS, 1);

    //Setup ADC
    adc_gpio_init(ACC_X);
    adc_gpio_init(ACC_Y);
    adc_gpio_init(ACC_Z);
    adc_select_input(current_adc_channel);

    // Setup DMA to do parallel SPI transfers
    dma0_tx = dma_claim_unused_channel(true);
    dma0_rx = dma_claim_unused_channel(true);
    dma1_tx = dma_claim_unused_channel(true);
    dma1_rx = dma_claim_unused_channel(true);

    dma_channel_config dma0_tx_config = dma_channel_get_default_config(dma0_tx);
    dma_channel_config dma0_rx_config = dma_channel_get_default_config(dma0_rx);
    dma_channel_config dma1_tx_config = dma_channel_get_default_config(dma1_tx);
    dma_channel_config dma1_rx_config = dma_channel_get_default_config(dma1_rx);

    channel_config_set_transfer_data_size(&dma0_tx_config, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&dma0_rx_config, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&dma1_tx_config, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&dma1_rx_config, DMA_SIZE_8);

    channel_config_set_dreq(&dma0_tx_config, DREQ_SPI0_TX);
    channel_config_set_dreq(&dma0_rx_config, DREQ_SPI0_RX);
    channel_config_set_dreq(&dma1_tx_config, DREQ_SPI1_TX);
    channel_config_set_dreq(&dma1_rx_config, DREQ_SPI1_RX);


    // Setup DMA for Accelerometer to get all 3 sensor values in parallel
    dma_acc = dma_claim_unused_channel(true);

    dma_channel_config_t dma_acc_config = dma_channel_get_default_config(dma_acc);
    channel_config_set_transfer_data_size(&dma_acc_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_acc_config, false);
    channel_config_set_write_increment(&dma_acc_config, true);
    channel_config_set_dreq(&dma_acc_config, DREQ_ADC);

    dma_channel_configure(dma_acc, &dma_acc_config, adc_buffer, &adc_hw->fifo, BUFFER_SIZE, true);

    // Enable DMA IRQ
    dma_channel_set_irq0_enabled(dma_acc, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_acc_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // --- Setup ADC FIFO ---
    adc_fifo_setup(true, true, 1, false, false);
    adc_run(true); // start free-running ADC


}

uint8_t SPItransfer(int dma_tx, int dma_rx, uint8_t address, spi_inst_t* spi){
    uint8_t data;

    dma_channel_config_t dma_rx_config = dma_channel_get_default_config(dma_rx);
    dma_channel_config_t dma_tx_config = dma_channel_get_default_config(dma_tx);

    dma_channel_configure(dma_rx, &dma_rx_config, &data, &spi_get_hw(spi)->dr, 1, false);
    dma_channel_configure(dma_tx, &dma_tx_config, &spi_get_hw(spi)->dr, &address, 1, true);

    return data;
}




void getGyroAxes(uint16_t* gyro){
    gpio_put(GYRO_CS, 0);
    uint8_t GYRO_ADDRESS = 0x43;
    for(int i = 0; i < 3; i++){
        //Get 8 MSB bits
        gyro[i] = SPItransfer(dma0_tx, dma0_rx, GYRO_ADDRESS, spi0) << 8;
        GYRO_ADDRESS++;

        //Get 8 LSB bits 
        gyro[i] |= SPItransfer(dma0_tx, dma0_rx, GYRO_ADDRESS, spi0);
        GYRO_ADDRESS++;
   } 

   gpio_put(GYRO_CS, 1);
}


void getAccelAxes(int* accel){
    //Assuming no external Vref
    float Vref = 3.3;
    float resolution = 65535.0f;


    float Vx = ((float) adc_buffer[0] * Vref) / resolution;
    float Vy = ((float) adc_buffer[1] * Vref) / resolution;
    float Vz = ((float) adc_buffer[2] * Vref) / resolution;

    float zeroG = Vref/2;
    float sensitivity = 0.3;

    float gravity = 9.81;
    accel[0] = ((Vx - zeroG) / sensitivity)*gravity;
    accel[1] = ((Vx - zeroG) / sensitivity)*gravity;
    accel[2] = ((Vx - zeroG) / sensitivity)*gravity;
}


void getMagAxes(uint16_t* mag){
    uint16_t time = 0x00;
    uint8_t status = SPItransfer(dma1_tx, dma1_rx, 0x4F, spi1);
    time = SPItransfer(dma1_tx, dma1_rx, 0x00, spi1) << 8;
    time |= SPItransfer(dma1_tx, dma1_rx, 0x00, spi1); 
    for(int i = 0; i < 3; i++){
        mag[i] = SPItransfer(dma1_tx, dma1_rx, 0x00, spi1) << 8;
        mag[i] |= SPItransfer(dma1_tx, dma1_rx, 0x00, spi1);
    }
}
