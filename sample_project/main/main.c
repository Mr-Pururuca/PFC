#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
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

// Instanciando estrutura para elementos da 'Queue'
typedef struct {
    uint64_t event_count;
    uint64_t alarm_value;
} Alarm_element_t;
// Instanciando estrutura Relogio
typedef struct {
    uint64_t hora;
    uint8_t mim;
    uint8_t seg;
} Relogio;

// iniciando TAGs para LOGs
static const char* TAG = "Projeto exemplo"; 
static const char* TIMER = "Timer"; 

// Iniciando estrutura goblal relogio
static Relogio relogio = {};

// Instanciando a leitura de interrupção
static void IRAM_ATTR gpio_isr_handler(void* arg){
    // retransformando o argumento passado a função pro tipo correto ('uint32_t')
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
// Instanciando função de evento do timer
static bool IRAM_ATTR Alarme_1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data){
    
    BaseType_t high_task_awoken = pdFALSE;
    // retransformando o argumento passado a função pro tipo correto ('Queue')
    QueueHandle_t queue = (QueueHandle_t)user_data;
    // Retrieve count value and send to queue
    Alarm_element_t ele = {
        .event_count = edata->count_value,
        .alarm_value = edata->alarm_value,
    };
    xQueueSendFromISR(queue, &ele, &high_task_awoken);
    // reconfigurando o valor do alarme
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + 100000, // alarm in next 1s
    };
    gptimer_set_alarm_action(timer, &alarm_config);
    // atualizando horario do relogio.
    relogio.seg = (edata->count_value/1000000) % 60;
    relogio.mim = ((edata->count_value/1000000) / 60) % 60;
    relogio.hora = ((edata->count_value/1000000) / 3600);
    // return whether we need to yield at the end of ISR
    return(high_task_awoken == pdTRUE);
}
// Instanciando task do timer
static void IRAM_ATTR timer_task(void *agr){
    // criado uma 'Queue' para Alarmes
    QueueHandle_t Alarms = xQueueCreate(10, sizeof(Alarm_element_t));
    // check para criação da 'Queue'
    if (!Alarms) {
        ESP_LOGE(TAG, "Falha ao criar Queue");
        return;
    }

    // instanciando o timer
    gptimer_handle_t gptimer = NULL;
    // Inciando estrutura de configuração do timer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    // Criação do timer
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    // Inciando estrutura de eventos do timer
    gptimer_event_callbacks_t cbs = {
        .on_alarm = Alarme_1,
    };
    // Inciando função de eventos no timer
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, Alarms));
    // Liberando o timer
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    // Inciando contagem do timer
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 100000, // periodo = 100ms
    };
    // Criando evento no timer
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    // Inciando contagem do timer
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    Alarm_element_t ele = {};
    while (1){
        xQueueReceive(Alarms, &ele, pdMS_TO_TICKS(500));
        ESP_LOGI(TIMER,"Relogio - %"PRIu64":%i:%i -", relogio.hora,relogio.mim,relogio.seg);
        ESP_LOGI(TIMER,"Contagem do timer:%"PRIu64", Valor do alarme:%"PRIu64"",ele.event_count,ele.alarm_value);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}

void app_main(void){
    // inicando a task do timer
    xTaskCreate(timer_task, "Inicia o timer", 2048, NULL, 10, NULL);

    // inicializando a estrutura de configuração
    gpio_config_t io_conf = {};

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
    // check para criação da 'Queue'
    if (!gpio_evt_queue) {
        ESP_LOGE(TAG, "Falha ao criar Queue");
        return;
    }
    // inicando a task para leitura das interrupções
    xTaskCreate(gpio_task_isrcheck, "Pilha de interrupções", 2048, NULL, 10, NULL);

    // instanciando o serviço de isr
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // conecta o interrupor à um pino gpio especifico
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    // conecta o interrupor à um pino gpio especifico
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

    /*
    int cnt = 0;
    while (1){
        //printf("cnt: %d\n", cnt++);
        //gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        //gpio_set_level(GPIO_OUTPUT_IO_1, (cnt+1) % 2);
        vTaskDelay(xDelay);
    }
    */
}
