
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_intr_alloc.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/rmt.h"

#include "soc/cpu.h"
#include "soc/dport_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc_cntl_reg.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"


#include "sdkconfig.h"
#include "ESP32Remote.h"



using namespace std;

extern "C" {
void app_main(void);
}

#define GPIO_INPUT_IO_0     GPIO_NUM_18
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)
#define nUART	(UART_NUM_0)

static const int RX_BUF_SIZE = 1024;
static const char *TX_TASK_TAG = "TX_TASK";
static const char *RX_TASK_TAG = "RX_TASK";
static xQueueHandle gpio_evt_queue = NULL;

ESP32_RMT_Rx irrecv(22, 0);
ESP32_RMT_Tx irtrans(23,1);

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

static uint8_t command=0;

void rtmInput(void * parameters) {

	irrecv.init();
	while(true)
	{


		command=irrecv.readIRrec();
		if (command!=0) {
			printf("NEC Command received : %i\n",command);
		}

	}
	vTaskDelete(NULL);
}

void rtmOutput( void * parameters) {

	vector<BaseRMTClass*> ProtocolDescriptions;
	irtrans.init();

	ProtocolData_t *ChineseLED = new ProtocolData_t();
	ChineseLED->_name = "Chinese LED";
	ChineseLED->_length = 32;
	ChineseLED->_headerHigh = 9375;
	ChineseLED->_headerLow = 4500;
	ChineseLED->_highTimeHigh = 600;
	ChineseLED->_highTimeLow = 1650;
	ChineseLED->_lowTimeHigh = 600;
	ChineseLED->_lowTimeLow = 560;
	ChineseLED->_isAddress = true;
	ChineseLED->_isInvertedAddressRequired = false;
	ChineseLED->_addressLength = 16;
	ChineseLED->_isDataInverseAddedRequired = true;
	ChineseLED->_tolerance = 50;
	ChineseLED->_isStop = true;
	ChineseLED->_stopSignHigh = 560;
	ChineseLED->_stopSignLow = 1000;

	// for testing purposes... the NEC protocol
	ProtocolData_t *NEC = new ProtocolData_t();
	NEC->_name = "NEC";
	NEC->_length = 32;
	NEC->_headerHigh = 9000;
	NEC->_headerLow = 4500;
	NEC->_highTimeHigh = 560;
	NEC->_highTimeLow = 1690;
	NEC->_lowTimeHigh = 560;
	NEC->_lowTimeLow = 560;
	NEC->_isAddress = true;
	NEC->_isInvertedAddressRequired = true;
	NEC->_addressLength = 8;
	NEC->_isDataInverseAddedRequired = true;
	NEC->_tolerance = 50;
	NEC->_isStop = true;
	NEC->_stopSignHigh = 560;
	NEC->_stopSignLow = 20;


	ProtocolData_t *ALDILED = new ProtocolData_t();
	ALDILED->_name = "ALDILED";
	ALDILED->_length = 32;
	ALDILED->_headerHigh = 9000;
	ALDILED->_headerLow = 4500;
	ALDILED->_highTimeHigh = 600;
	ALDILED->_highTimeLow = 1650;
	ALDILED->_lowTimeHigh = 600;
	ALDILED->_lowTimeLow = 530;
	ALDILED->_isAddress = true;
	ALDILED->_isInvertedAddressRequired = true;
	ALDILED->_addressLength = 8;
	ALDILED->_isDataInverseAddedRequired = true;
	ALDILED->_tolerance = 50;
	ALDILED->_isStop = true;
	ALDILED->_stopSignHigh = 600;
	ALDILED->_stopSignLow = 20;

	rmt_item32_t* item = new rmt_item32_t[ALDILED->_length + 2]; // normaly it must be the longest.....
//	rmt_item32_t* item2 = new rmt_item32_t[ALDILED->_length + 2];
	PulseDistanceCoding * AldiLEDDriver = new PulseDistanceCoding(ALDILED);

	for(;;) {
		AldiLEDDriver->GenerateOutput(item,0x00, 0xE2);
		irtrans.sendIR(item,ALDILED->_length + 2);
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);


}

void uartRxTask(void * parameters) {
	static const char *RX_TASK_TAG = "RX_TASK";
	esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

	uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
	string sdata;
	while (1) {
		sdata.clear();
		const int rxBytes = uart_read_bytes(nUART, data, RX_BUF_SIZE, 20 / portTICK_RATE_MS);
		if (rxBytes > 0) {
			data[rxBytes] = 0;
			ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
			ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
			sdata += (char *)data;
			printf("RxTask: #%s#",sdata.c_str());
		}
	}

}

void initUART() {
	const uart_config_t uart_config = {
	        .baud_rate = 115200,
	        .data_bits = UART_DATA_8_BITS,
	        .parity = UART_PARITY_DISABLE,
	        .stop_bits = UART_STOP_BITS_1,
	        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	    };
	uart_param_config(nUART, &uart_config);
	uart_set_pin(nUART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	// We won't use a buffer for sending data.
	uart_driver_install(nUART, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(nUART, data, len);
    ESP_LOGI(logName, "Wrote %d bytes\n", txBytes);
    ESP_LOGI(logName, "#%s#\n", data);
    return txBytes;
}



void app_main(void)
{
    nvs_flash_init();
   // tcpip_adapter_init();
    initUART();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t) );

    gpio_config_t io_conf;
    	io_conf.intr_type = GPIO_INTR_NEGEDGE;
    	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    	io_conf.mode = GPIO_MODE_INPUT;
    	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    	gpio_config(&io_conf);

 /*   ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );
*/
//    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
//    int level = 0;
//    while (true) {
//        gpio_set_level(GPIO_NUM_4, level);
//        level = !level;
//        vTaskDelay(300 / portTICK_PERIOD_MS);
//    }
    xTaskCreate(rtmInput, "test task", 2048, NULL , 10, NULL);
    xTaskCreate(&uartRxTask,"UARTRX",2000,NULL,5,NULL);
    xTaskCreate(&rtmOutput, "IR task", 2048, NULL , 10, NULL);
    while(1) {
    		vTaskDelay(1000 / portTICK_PERIOD_MS);
    	}
}

