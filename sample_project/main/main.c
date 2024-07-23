#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

// difinando o valor das entradas pela configuração do menu
#define GPIO_INPUT_IO_0     CONFIG_GPIO_INPUT_0
#define GPIO_INPUT_IO_1     CONFIG_GPIO_INPUT_1
// agrupar o mapeamento binario das portas de entrada
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))

// difinando o valor das saidas pela configuração do menu
#define GPIO_OUTPUT_IO_0    CONFIG_GPIO_OUTPUT_0
#define GPIO_OUTPUT_IO_1    CONFIG_GPIO_OUTPUT_1
#define GPIO_OUTPUT_IO_2    CONFIG_GPIO_OUTPUT_2
// agrupar o mapeamento binario das portas de saida
#define GPIO_OUTPUT_PIN_SEL  (((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1)) | (1ULL<<GPIO_OUTPUT_IO_2))

#define ESP_INTR_FLAG_DEFAULT 0

// Inicializando uma 'Queue'
static QueueHandle_t gpio_evt_queue = NULL;

// inicia TAG para logs
    static const char* TAG = "MyModule";

// Instanciando a leitura de interrupção
static void IRAM_ATTR gpio_isr_handler(void* arg){

    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Instanciando a Tasks para checar as interupções
static void gpio_task_isrcheck(void* arg){
    
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG,"GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            gpio_set_level(GPIO_OUTPUT_IO_2, io_num-21);
        }   
    }
}

void app_main(void){
    // inicializando a estrutura de configuração
    gpio_config_t io_conf = {};

    // cria constante de delay em ms
    const TickType_t xDelay = 5000 / portTICK_PERIOD_MS;

    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // interrupção na borda de descida
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    // mapeamento binario dos pinos
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // setando pinos como entradas
    io_conf.mode = GPIO_MODE_INPUT;
    // ativa o resistor de pull-up
    io_conf.pull_up_en = 1;
    // configura as gpio com os valores setados
    gpio_config(&io_conf);

    // criado uma 'Queue' para interrupções
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // inicando a task para leitura das interrupções
    xTaskCreate(gpio_task_isrcheck, "Pilha de interrupções", 2048, NULL, 10, NULL);

    // instanciando o serviço de isr
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // conecta o interrupor à um pino gpio especifico
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    // conecta o interrupor à um pino gpio especifico
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

    

    int cnt = 0;
    while (1){
        printf("cnt: %d\n", cnt++);
        gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        gpio_set_level(GPIO_OUTPUT_IO_1, (cnt+1) % 2);
        vTaskDelay(xDelay);
    }
}
