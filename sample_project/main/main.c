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
#include "driver/ledc.h"
#include "esp_err.h"

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

// Definindo estrutura para elementos da 'Queue'
typedef struct {
    uint64_t event_count;
    uint64_t alarm_value;
} Alarm_element_t;
// Definindo estrutura Relogio
typedef struct {
    uint64_t hora;
    uint8_t mim;
    uint8_t seg;
} Relogio;

// Iniciando TAGs para LOGs
static const char* TAG = "Projeto exemplo";
static const char* TIMER = "Timer";
// Iniciando estrutura goblal relogio
static Relogio relogio = {};
// Inicializando uma 'Queue' global
static QueueHandle_t gpio_evt_queue = NULL;
// Iniciando semaforo global
static SemaphoreHandle_t semaphore_pwm = NULL;
// Indica mode automatico do PWM
static bool auto_mode = false;

// Definindo a função de leitura de interrupção
static void IRAM_ATTR gpio_isr_handler(void* arg){
    // retransformando o argumento passado a função pro tipo correto ('uint32_t')
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
// Definindo função de evento do timer
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
// definição da função de configurar GPIO
static void config_gpio(uint64_t mask, gpio_mode_t mode, gpio_pullup_t pull_up, gpio_pulldown_t pull_down, gpio_int_type_t itr_type){
    // inicializando a estrutura de configuração
    gpio_config_t io_conf = {};

    // mapeamento binario dos pinos
    io_conf.pin_bit_mask = mask;
    // setando o modo dos pinos saida ou entrada
    io_conf.mode = mode;
    // configura o modo de pull-up
    io_conf.pull_up_en = pull_up;
    // configura o modo pull-down
    io_conf.pull_down_en = pull_down;
    //desativa interrupção
    io_conf.intr_type = itr_type;
    
    // configura as gpio com os valores setados
    gpio_config(&io_conf);
}

// Definindo task do timer
static void IRAM_ATTR timer_task(void *agr){
    ESP_LOGE(TAG, "Task timer iniciou");
    // criado uma 'Queue' para Alarmes
    QueueHandle_t Alarms = xQueueCreate(10, sizeof(Alarm_element_t));
    // check para criação da 'Queue'
    if (!Alarms) {
        ESP_LOGE(TAG, "Falha ao criar Queue");
        return;
    }

    // Definindo o timer
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
    // loop da task
    while (1){
        // Espera semaforo ser liberado pela task PWM
        // le a queue de alarmes e salva em uma variavel
        if(xQueueReceive(Alarms, &ele, pdMS_TO_TICKS(500))){
            ESP_LOGI(TIMER,"Contagem do timer:%"PRIu64", Valor do alarme:%"PRIu64"",ele.event_count,ele.alarm_value);
        }
        ESP_LOGI(TIMER,"Relogio - %"PRIu64":%i:%i -", relogio.hora,relogio.mim,relogio.seg);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (auto_mode){
            // Libera o semaforo usado para sincronisar o LEDC PWM com o timer
            xSemaphoreGive(semaphore_pwm);
        }
    }
    
}
// Definindo task para checar as interupções
static void gpio_task_isrcheck(void* arg){
    ESP_LOGE(TAG, "Task isrcheck iniciou");
    uint32_t io_num;
    uint64_t LEDC_DUTY = 0;
    // loop da task
    for (;;) {
        // checa se alguma interução foi adicionada a fila
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // plota na tela qual a interrução encontrada
            ESP_LOGI(TAG,"GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            //modo do PWM durante a interrupção
            if (io_num == 21){
                // ativa o modo manual
                auto_mode = false;
                // desliga o led IO2
                gpio_set_level(GPIO_OUTPUT_IO_2, 0);

                // incrementa o duty até o maximo e zera
                if(LEDC_DUTY < 8192) {
                LEDC_DUTY += 256;
                ESP_LOGE(TAG, "Duty incrementado %"PRIu64"",LEDC_DUTY );
                }
                else {
                LEDC_DUTY = 0;
                ESP_LOGE(TAG, "Duty resetado %"PRIu64"",LEDC_DUTY );
                }
                // seta o duty incrementalmente
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_DUTY));
                // atualiza o valor do duty
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
                ESP_LOGE(TAG, "Duty atualizado %"PRIu64"",LEDC_DUTY );
            }
            else{
                // ativa o modo automatico
                auto_mode = true;
                // liga o led IO2
                gpio_set_level(GPIO_OUTPUT_IO_2, 1);
            }
        }   
    }
}
// Definindo task do LEDC PWM
static void IRAM_ATTR PWM_task(void* arg){
    ESP_LOGE(TAG, "Task PWM iniciou");
   // Configuração do timer do LEDC PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 5000,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configuração do channel do LEDC PWM
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = CONFIG_GPIO_OUTPUT_0,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    
    // tempo em 1 do PWM
    uint64_t LEDC_DUTY = 0;
    // loop da task
    while (1){
        // Espera semaforo ser liberado pela task timer e checa o modo do PWM
        if (xSemaphoreTake(semaphore_pwm, portMAX_DELAY)){
            // incrementa o duty até o maximo e resata
            if(LEDC_DUTY <= 8192) {
                LEDC_DUTY += 256;
                ESP_LOGE(TAG, "Duty incrementado %"PRIu64"",LEDC_DUTY );
            }
            else {
                LEDC_DUTY = 0;
                ESP_LOGE(TAG, "Duty resetado %"PRIu64"",LEDC_DUTY );
            }
            // seta o duty incrementalmente
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_DUTY));
            // atualiza o valor do duty
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
            ESP_LOGE(TAG, "Duty atualizado %"PRIu64"",LEDC_DUTY );
        }
    }
    
}

void app_main(void){
    // criação do semaphore binario 
    semaphore_pwm = xSemaphoreCreateBinary();

    // Configurando pinos de saida do esp32
    config_gpio(GPIO_OUTPUT_PIN_SEL,GPIO_MODE_OUTPUT,0,0,GPIO_INTR_DISABLE);
    // Configurando pinos de entrada do esp32
    config_gpio(GPIO_INPUT_PIN_SEL,GPIO_MODE_INPUT,1,0,GPIO_INTR_POSEDGE);

    // criado uma 'Queue' para interrupções
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // check para criação da 'Queue'
    if (!gpio_evt_queue) {
        ESP_LOGE(TAG, "Falha ao criar Queue");
        return;
    }
    
    // inicando a task do timer
    xTaskCreate(timer_task, "Inicia o timer", 2048, NULL, 10, NULL);
    // inicando a task do PWM
    xTaskCreate(PWM_task, "Inicia o PWM", 2048, NULL, 10, NULL);
    // inicando a task para leitura das interrupções
    xTaskCreate(gpio_task_isrcheck, "Pilha de interrupções", 2048, NULL, 10, NULL);

    // Definindo o serviço de isr
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // conecta o interrupor à um pino gpio especifico
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    // conecta o interrupor à um pino gpio especifico
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
}
