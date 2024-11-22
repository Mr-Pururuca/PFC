//---------------------------------------Bibliotecas----------------------------------------//
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "protocol_examples_common.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

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
#define V_A_MAX   CONFIG_V_A_MAX
// definindo o algulo de atuação do motor
#define GAMMA   CONFIG_GAMMA

//-----------------------------------Estruturas de dados------------------------------------//
// Definindo estrutura de leitura do ADC
typedef struct{
    int adc_raw[10];
    int voltage[10];
}ADC_data;

//-------------------------------------------TAGs-------------------------------------------//
static const char* TAG = "Projeto";
//static const char* PWM = "Atuador do Motor";
static const char* ADC = "Letura de dados";
static const char* MQTT = "MQTT";
static const char* PUB = "Armazenando dados";

//-----------------------------------------Handles------------------------------------------//

// Inicializando 'Queues' no contexto global
//static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t data_pub_queue = NULL;
static QueueHandle_t adc_read_queue = NULL;
// Iniciando semaforos no contexto global
// static SemaphoreHandle_t semaphore_motor = NULL;
static SemaphoreHandle_t semaphore_Pub = NULL;
// Iniciando handle da conexão MQTT
esp_mqtt_client_handle_t client;

//------------------------------------Variaveis Globais-------------------------------------//

// Variavel que liga a função de atuação do motor
static bool Motor_on = false;

//--------------------------------------Funções Gerais--------------------------------------//

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
// MQTT log error
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(MQTT, "Last error %s: 0x%x", message, error_code);
    }
}

//-----------------------------------------Callbacks----------------------------------------//

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(MQTT, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    int msg_id;
    // uint16_t duty_mqtt = 0;
    char *texto_mqtt;                // Cria o espaço para receber o texto

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(MQTT, "MQTT_EVENT_CONNECTED");

        msg_id = esp_mqtt_client_subscribe(client, "/topic/comandos", 1);    
        ESP_LOGI(MQTT, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "/topic/Theta_1", 1);     
        ESP_LOGI(MQTT, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "/topic/Vel_angular_1", 1);     
        ESP_LOGI(MQTT, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "/topic/Tensao_armadura", 1);      
        ESP_LOGI(MQTT, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(MQTT, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/reset", "data", 0, 0, 0);
        ESP_LOGI(MQTT, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        asprintf(&texto_mqtt, "%.*s", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(MQTT, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(MQTT, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(MQTT, "Other event id:%d", event->event_id);
        break;
    }
    // free(texto_mqtt);
}

//------------------------------------------Tasks-------------------------------------------//

// Definição da task para leitura do do ADC
static void Leitura_ADC(void *arg){
    ESP_LOGI(TAG, "Task ADC iniciou");
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
            ESP_LOGI(ADC, "Valor lido de posição: %d",adc_read.voltage[0]);
        }
        // insere os valores brutos e convertidos na 'queue'
        xQueueSend(adc_read_queue, &adc_read, NULL);
        // insere os valores brutos e convertidos na 'queue' de publicação
        xQueueSend(data_pub_queue, &adc_read, NULL);
        // libera semaforo pra sincornizar a de publicação de dados
        xSemaphoreGive(semaphore_Pub);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle));
    if (do_calibration) {
        adc_calibration_deinit(adc_cali_handle);
    }
}
/*// Definindo task para acionamento do motor via PWM
static void PWM_task(void* arg){
    ESP_LOGI(TAG, "Task PWM iniciou");
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
    // tempo em on do sinal PWM
    uint64_t LEDC_DUTY;
    ADC_data adc_read = {};
    // loop da task
    while (1){
        // Espera semaforo ser liberado pela task timer e checa o modo do PWM
        if (!xSemaphoreTake(semaphore_pwm, portMAX_DELAY)){
            continue;
        }
        if(!xQueueReceive(adc_read_queue, &adc_read, 0)){
            ESP_LOGI(PWM, "valor lido:%d < %d",adc_read.voltage[0], GAMMA*100);
            continue;
        }
        if((adc_read.voltage[0] < GAMMA*100) && Motor_on){
            LEDC_DUTY = (V_a*8191/V_A_MAX);
            // seta o duty do PWM
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_DUTY));
            // atualiza o valor do duty
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
            ESP_LOGI(PWM, "Tensão aplicada no motor: %"PRIu64"V",LEDC_DUTY );
        }
        else {
            LEDC_DUTY = 0;
            // seta o duty do PWM
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_DUTY));
            // atualiza o valor do duty
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
        }
    }
}
*/
// Definindo da função de conexão com o Broaker MQTT
static void MQTT_Connect(){
    ESP_LOGI(TAG, "Task MQTT iniciou");
    ESP_LOGI(MQTT, "[APP] Startup..");
    ESP_LOGI(MQTT, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(MQTT, "[APP] IDF version: %s", esp_get_idf_version());
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper functon configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .credentials.username = "ESP_32",
        .credentials.authentication.password = "PFC123",
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}
// Definição da task de publicação de dados assincrona
static void Data_pub(){
    ESP_LOGI(TAG, "Task Data_pub iniciou");
    char *texto_mqtt;
    ADC_data adc_read = {};
    while (1){
        
        if(xQueueReceive(data_pub_queue, &adc_read, 0)){
            ESP_LOGI(PUB, "Theta_1:%d",adc_read.voltage[0]);
            asprintf(&texto_mqtt, "%.2d", adc_read.voltage[0]);
            esp_mqtt_client_publish(client, "/topic/Theta_1", texto_mqtt, 0, 1, 1);    
    }
   }
}

//---------------------------------------Interrupções---------------------------------------//

// Definindo da função de leitura de interrupção
static void IRAM_ATTR gpio_isr_Motor_on(void* arg){
    if(!Motor_on){
        //ESP_LOGI(TAG,"Excitação do sistema ligada");
        Motor_on = true;
        gpio_set_level(GPIO_OUTPUT_IO_2, 1);
    }
    else{
        //ESP_LOGI(TAG,"Excitação do sistema desligada");
        Motor_on = false;
        gpio_set_level(GPIO_OUTPUT_IO_2, 0);
    }
}

//--------------------------------------Função primaria-------------------------------------//

void app_main(void){
    // criação do semaphore binario 
    semaphore_Pub = xSemaphoreCreateBinary();
    
    // Configurando pinos de saida do esp32
    config_gpio(GPIO_OUTPUT_PIN_SEL,GPIO_MODE_OUTPUT,0,0,GPIO_INTR_DISABLE);
    // Configurando pinos de entrada do esp32
    config_gpio(GPIO_INPUT_PIN_SEL,GPIO_MODE_INPUT,1,0,GPIO_INTR_POSEDGE);

    /*// criado 'Queue' para interrupções
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // check para criação da 'Queue'
    if (!gpio_evt_queue) {
        ESP_LOGE(TAG, "Falha ao criar Queue");
        return;
    }//*/
    // criado 'Queue' para a leitura do ADC
    adc_read_queue = xQueueCreate(10, sizeof(ADC_data));
    // check para criação da 'Queue'
    if (!adc_read_queue) {
        ESP_LOGE(TAG, "Falha ao criar Queue");
        return;
    }
    // criado 'Queue' para publicação no broker MQTT
    data_pub_queue = xQueueCreate(10, sizeof(ADC_data));
    // check para criação da 'Queue'
    if (!data_pub_queue) {
        ESP_LOGE(TAG, "Falha ao criar Queue");
        return;
    }

    // inicia conexão com Broker MQQTT
    MQTT_Connect();
    
    // Inicindo as tasks
    xTaskCreate(Leitura_ADC, "Inicia a leitura do angulo theta1", 2048, NULL, 10, NULL);
    //xTaskCreate(PWM_task, "Inicia a atuação do motor", 2048, NULL, 10, NULL);
    xTaskCreate(Data_pub,"Inicia gravação dos dados no broker MQTT", 2048, NULL, 10, NULL);

    // Definindo o serviço de isr
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // Conecta o interrupor à um pino gpio especifico
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_Motor_on, (void*) GPIO_INPUT_IO_0);
}
