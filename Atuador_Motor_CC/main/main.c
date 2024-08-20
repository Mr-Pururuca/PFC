//---------------------------------------Bibliotecas----------------------------------------//
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
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

//------------------------------------------Defines-----------------------------------------//
// difinindo o valor das entradas pela configuração do menu
#define GPIO_INPUT_IO_0     CONFIG_GPIO_INPUT_0
#define GPIO_INPUT_IO_1     CONFIG_GPIO_INPUT_1
// agrupar o mapeamento binario das portas de entrada
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
// difinindo o valor das saidas pela configuração do menu
#define GPIO_OUTPUT_IO_0    CONFIG_GPIO_OUTPUT_0
#define GPIO_OUTPUT_IO_1    CONFIG_GPIO_OUTPUT_1
#define GPIO_OUTPUT_IO_2    CONFIG_GPIO_OUTPUT_2
// agrupar o mapeamento binario das portas de saida
#define GPIO_OUTPUT_PIN_SEL  (((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1)) | (1ULL<<GPIO_OUTPUT_IO_2))
// definindo flag de interrupção
#define ESP_INTR_FLAG_DEFAULT 0
// definindo valor do de tensão de armadura maxima do motor
#define V_A_MAX CONFIG_V_A_MAX
// definindo o algulo de atuação do motor
#define GAMMA CONFIG_GOMMA

//-----------------------------------Estruturas de dados------------------------------------//
// Definindo estrutura de leitura do ADC
typedef struct{
    int adc_raw[10];
    int voltage[10];
}ADC_data;

//-------------------------------------------TAGs-------------------------------------------//
static const char* TAG = "Projeto";
static const char* PWM = "Atuador do Motor";
static const char* ADC = "ADC";

//-----------------------------------------Drivers------------------------------------------//
// Fazer drives de comunição para leitura do angulo

//-----------------------------------------Handles------------------------------------------//
// Inicializando 'Queues' no contexto global
static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t adc_read_queue = NULL;
// Iniciando semaforos no contexto global
static SemaphoreHandle_t semaphore_pwm = NULL;

//------------------------------------Variaveis Globais-------------------------------------//
// Variavel que liga a função de atuação do motor
static bool Motor_on = false;

//------------------------------------Funções genericas-------------------------------------//
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
// definição da função de calibração do ADC
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle){
    // Iniciando um calibrador ADC
    adc_cali_handle_t handle = NULL;
    // Inicando estrutura de codigos de erro do ESP
    esp_err_t ret = ESP_FAIL;
    // Iniciando estado da calibração
    bool calibrated = false;
    
    ESP_LOGI(TAG, "Esquema de calibração utilizado é Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        calibrated = true;
    }
    // aponta o calibrador passado como parametro para calibrador configurado na função.
    *out_handle = handle;
    // testa e plota o resoltado da calibração
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}
// definindo função de deletar a calibração do ADC
static void adc_calibration_deinit(adc_cali_handle_t handle){
    ESP_LOGI(TAG, "Deletando schema de calibração Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle)); 
}

//------------------------------------------Tasks-------------------------------------------//
// Definição da task para elitura do do ADC
static void Leitura_ADC(void *arg){
    ESP_LOGE(TAG, "Task ADC iniciou");
    // iniciando um modulo ADC do modo oneshot
    adc_oneshot_unit_handle_t adc_handle;
    // configuração do modulo ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    // configuração do canal do modulo ADC
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_3, &config));
    // Calibração do modulo ADC
    adc_cali_handle_t adc_cali_handle = NULL;
    bool do_calibration = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_3, ADC_ATTEN_DB_12, &adc_cali_handle);
    
    ADC_data adc_read = {};
    // loop da task
   while (1){ 
        // faz a leitura do chanal ADC
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_3, &adc_read.adc_raw[0]));
        // Checa se a calibração foi feita
        if (do_calibration) {
            // faz a conversão do valor bruto para a variavel
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle,  adc_read.adc_raw[0], &adc_read.voltage[0]));
        }
        // insere os valores brutos e convertidos na 'queue'
        xQueueSend(adc_read_queue, &adc_read, NULL);
        // libera semaforo pra sincornizar leitura com a atuação
        xSemaphoreGive(semaphore_pwm);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle));
    if (do_calibration) {
        adc_calibration_deinit(adc_cali_handle);
    }
}
// Definindo task para acionamento do motor via PWM
static void PWM_task(void* arg){
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
        .gpio_num       = CONFIG_GPIO_OUTPUT_1,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    
    // tensão aplicada na armadura
    uint64_t V_a = 20;
    uint64_t LEDC_DUTY;
    ADC_data adc_read = {};
    // loop da task
    while (1){
        // Espera semaforo ser liberado pela task timer e checa o modo do PWM
        if (xSemaphoreTake(semaphore_pwm, portMAX_DELAY)){
            if(xQueueReceive(adc_read_queue, &adc_read, 0)){
                if(adc_read.voltage < GAMMA){
                    LEDC_DUTY = (V_a/V_A_MAX)*8191;
                }
                else LEDC_DUTY = 0;
                // seta o duty do PWM
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_DUTY));
                // atualiza o valor do duty
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
                ESP_LOGE(PWM, "Tensão aplicada no motor: %"PRIu64"V",LEDC_DUTY );
            }
        }
    }
}

//---------------------------------------Interrupções---------------------------------------//
// Definindo da função de leitura de interrupção
static void IRAM_ATTR gpio_isr_Motor_on(void* arg){
    if(!Motor_on){
        ESP_LOGI(TAG,"Excitação do sistema ligada");
        Motor_on = true;
        gpio_set_level(GPIO_OUTPUT_IO_2, 1);
    }
    else{
        ESP_LOGI(TAG,"Excitação do sistema desligada");
        Motor_on = false;
        gpio_set_level(GPIO_OUTPUT_IO_2, 0);
    }
}

//--------------------------------------Função primaria-------------------------------------//
void app_main(void){
    // criação do semaphore binario 
    semaphore_pwm = xSemaphoreCreateBinary();

    // Configurando pinos de saida do esp32
    config_gpio(GPIO_OUTPUT_PIN_SEL,GPIO_MODE_OUTPUT,0,0,GPIO_INTR_DISABLE);
    // Configurando pinos de entrada do esp32
    config_gpio(GPIO_INPUT_PIN_SEL,GPIO_MODE_INPUT,1,0,GPIO_INTR_POSEDGE);

     // criado 'Queue' para interrupções
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // check para criação da 'Queue'
    if (!gpio_evt_queue) {
        ESP_LOGE(TAG, "Falha ao criar Queue");
        return;
    }
    // criado 'Queue' para a leitura do ADC
    adc_read_queue = xQueueCreate(10, sizeof(ADC_data));
    // check para criação da 'Queue'
    if (!adc_read_queue) {
        ESP_LOGE(TAG, "Falha ao criar Queue");
        return;
    }
    
    xTaskCreate(Leitura_ADC, "Inicia a leitura do angulo theta1", 2048, NULL, 10, NULL);
    xTaskCreate(PWM_task, "Inicia a atuação do motor", 2048, NULL, 10, NULL);

    // Definindo o serviço de isr
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // Conecta o interrupor à um pino gpio especifico
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
}
