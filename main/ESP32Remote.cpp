/*
 * ESP32Remote.cpp
 *
 *  Created on: 2020. dec. 15.
 *      Author: Dell
 */
#include <cstdint>
#include <stdio.h>
#include <string>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include "cJSON.h"

#include "ESP32Remote.h"

#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */

static RingbufHandle_t ringBuf;

#define NEC_HEADER_HIGH_US    9000                         /*!< NEC protocol header: positive 9ms */
#define NEC_HEADER_LOW_US     4500                         /*!< NEC protocol header: negative 4.5ms*/
#define NEC_BIT_ONE_HIGH_US    560                         /*!< NEC protocol data bit 1: positive 0.56ms */
#define NEC_BIT_ONE_LOW_US    (2250-NEC_BIT_ONE_HIGH_US)   /*!< NEC protocol data bit 1: negative 1.69ms */
#define NEC_BIT_ZERO_HIGH_US   560                         /*!< NEC protocol data bit 0: positive 0.56ms */
#define NEC_BIT_ZERO_LOW_US   (1120-NEC_BIT_ZERO_HIGH_US)  /*!< NEC protocol data bit 0: negative 0.56ms */
#define NEC_BIT_END            560                         /*!< NEC protocol end: positive 0.56ms */
#define NEC_BIT_MARGIN         20                          /*!< NEC parse margin time */

#define NEC_BITS          32
#define NEC_HDR_MARK    9000
#define NEC_HDR_SPACE   4500
#define NEC_BIT_MARK     560
#define NEC_ONE_SPACE   1690
#define NEC_ZERO_SPACE   560
#define NEC_RPT_SPACE   2250

#define TAG "ESP32Remote"

ESP32Remote::ESP32Remote() {
	// TODO Auto-generated constructor stub

}

ESP32Remote::~ESP32Remote() {
	// TODO Auto-generated destructor stub
}

ESP32_RMT_Rx::ESP32_RMT_Rx(int gpiopin, int channelnumber)
{
	if (gpiopin>=GPIO_NUM_0 && gpiopin<GPIO_NUM_MAX) {
		gpionum = gpiopin;
	} else {
		gpionum = (int)GPIO_NUM_22;
	}
	if (channelnumber<=RMT_CHANNEL_0 && channelnumber<RMT_CHANNEL_MAX) {
		rmtport = channelnumber;
	} else {
		rmtport = (int)RMT_CHANNEL_0;
	}
	ESP32_RMT_Rx::initDone = false;
}

ESP32_RMT_Rx::~ESP32_RMT_Rx() {
	rmt_driver_uninstall((rmt_channel_t)ESP32_RMT_Rx::rmtport);
}

void ESP32_RMT_Rx::init(){


	rmt_config_t rmt_rx;
	rmt_rx.channel = (rmt_channel_t)ESP32_RMT_Rx::rmtport;
	rmt_rx.gpio_num = (gpio_num_t)ESP32_RMT_Rx::gpionum;
	rmt_rx.clk_div = RMT_CLK_DIV;
	rmt_rx.mem_block_num = 1;
	rmt_rx.rmt_mode = RMT_MODE_RX;
	rmt_rx.rx_config.filter_en = true;
	rmt_rx.rx_config.filter_ticks_thresh = 100;
	rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
	rmt_config(&rmt_rx);
	rmt_driver_install(rmt_rx.channel, 1000, 0);
	rmt_get_ringbuf_handle(rmt_rx.channel, &ringBuf);

	rmt_rx_start(rmt_rx.channel, true);
	ESP32_RMT_Rx::initDone = true;
}

uint8_t ESP32_RMT_Rx::readIRrec(void){
	size_t itemSize;
	uint8_t command = 0;

	rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive((RingbufHandle_t)ringBuf, (size_t *)&itemSize, (TickType_t)portMAX_DELAY);

	int numItems = itemSize / sizeof(rmt_item32_t);
	int i;
	rmt_item32_t *p = item;
	for (i=0; i<numItems; i++) {
		p++;
	}
	ESP32_RMT_Rx::printTiming(item, numItems);
	command=decodeNEC(item, numItems);
	vRingbufferReturnItem(ringBuf, (void*) item);

	return command;
}

bool ESP32_RMT_Rx::isInRange(rmt_item32_t item, int lowDuration, int highDuration, int tolerance) {
	uint32_t highValue = item.duration0 * 10 / RMT_TICK_10_US;
	uint32_t lowValue = item.duration1 * 10 / RMT_TICK_10_US;

	//ESP_LOGI(TAG, "lowValue=%d, highValue=%d, lowDuration=%d, highDuration=%d", lowValue, highValue, lowDuration, highDuration);

	if (lowValue < (lowDuration - tolerance) || lowValue > (lowDuration + tolerance) ||
			(highValue != 0 &&
					(highValue < (highDuration - tolerance) || highValue > (highDuration + tolerance)))) {
		return false;
	}
	return true;
}

bool ESP32_RMT_Rx::NEC_is0(rmt_item32_t item) {
	return isInRange(item, NEC_BIT_MARK, NEC_BIT_MARK, 100);
}

bool ESP32_RMT_Rx::NEC_is1(rmt_item32_t item) {
	return isInRange(item, NEC_BIT_MARK, NEC_ONE_SPACE, 100);
}

uint8_t ESP32_RMT_Rx::decodeNEC(rmt_item32_t *data, int numItems) {
	if (!isInRange(data[0], NEC_HDR_MARK, NEC_HDR_SPACE, 200)) {
		ESP_LOGD(TAG, "Not an NEC");
		return 0;
	}
	int i;
	uint8_t address = 0, notAddress = 0, command = 0, notCommand = 0;
	int accumCounter = 0;
	uint8_t accumValue = 0;
	for (i=1; i<numItems; i++) {
		if (NEC_is0(data[i])) {
			ESP_LOGD(TAG, "%d: 0", i);
			accumValue = accumValue >> 1;
		} else if (NEC_is1(data[i])) {
			ESP_LOGD(TAG, "%d: 1", i);
			accumValue = (accumValue >> 1) | 0x80;
		} else {
			ESP_LOGD(TAG, "Unknown");
		}
		if (accumCounter == 7) {
			accumCounter = 0;
			ESP_LOGD(TAG, "Byte: 0x%.2x", accumValue);
			if (i==8) {
				address = accumValue;
			} else if (i==16) {
				notAddress = accumValue;
			} else if (i==24) {
				command = accumValue;
			} else if (i==32) {
				notCommand = accumValue;
			}
			accumValue = 0;
		} else {
			accumCounter++;
		}
	}
	ESP_LOGD(TAG, "Address: 0x%.2x, NotAddress: 0x%.2x", address, notAddress ^ 0xff);
	if (address != (notAddress ^ 0xff) || command != (notCommand ^ 0xff)) {
		// Data mis match
		return 0;
	}
	// Serial.print("Address: ");
	// Serial.print(address);
	// Serial.print(" Command: ");
	// Serial.println(command);

	return command;
}

void ESP32_RMT_Rx::printTiming(rmt_item32_t * data, int numItems) {
	ESP_LOGI(TAG,"-------------------------");
	for ( int i = 0; i< numItems; i++){
		uint32_t lowValue = data[i].duration0 * 10 / RMT_TICK_10_US;
		uint32_t highValue = data[i].duration1 * 10 / RMT_TICK_10_US;
		ESP_LOGI(TAG,"%d, %d",lowValue,highValue);
	}
	return;
}


ESP32_RMT_Tx::ESP32_RMT_Tx(int gpiopin, int channelnumber) {
	ESP32_RMT_Tx::gpionum = gpiopin;
	ESP32_RMT_Tx::rmtport = channelnumber;

	if (gpiopin>=GPIO_NUM_0 && gpiopin<GPIO_NUM_MAX) {
		gpionum = gpiopin;
	} else {
		gpionum = (int)GPIO_NUM_23;
	}
	if (channelnumber<=RMT_CHANNEL_0 && channelnumber<RMT_CHANNEL_MAX) {
		rmtport = channelnumber;
	} else {
		rmtport = (int)RMT_CHANNEL_1;
	}
	ESP32_RMT_Tx::initDone = false;
}

/*
 * @brief Initialize the RMT driver for transmitting
 */
void ESP32_RMT_Tx::init() {
	rmt_config_t rmt_tx;
	rmt_tx.channel = (rmt_channel_t)ESP32_RMT_Tx::rmtport;
	rmt_tx.gpio_num = (gpio_num_t)ESP32_RMT_Tx::gpionum;
	rmt_tx.clk_div = RMT_CLK_DIV;
	rmt_tx.mem_block_num = 1;
	rmt_tx.rmt_mode = RMT_MODE_TX;
	rmt_tx.tx_config.loop_en = false;
	rmt_tx.tx_config.carrier_duty_percent = 50;
	rmt_tx.tx_config.carrier_freq_hz = 38000;

	rmt_tx.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
	rmt_tx.tx_config.carrier_en = true;
	rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
	rmt_tx.tx_config.idle_output_en = true;

	rmt_config(&rmt_tx);
	rmt_driver_install(rmt_tx.channel, 0, 0);

	this->initDone = true;
}

/*
 * @brief Build register value of waveform for one data bit
 */
void ESP32_RMT_Tx::generateRMTItem(rmt_item32_t * item, int high_us, int low_us) {
	item->level0 = 1;
	item->duration0 = (high_us) / 10 * RMT_TICK_10_US;
	item->level1 = 0;
	item->duration1 = (low_us) / 10 * RMT_TICK_10_US;
}

/*
 * @brief get back if the driver initialized
 */
bool ESP32_RMT_Tx::isInit(void) {
	return this->initDone;
}

/*
 * @brief get back the GPIO number if the driver initialized
 */
int ESP32_RMT_Tx::getGpioNum(void) {
	if( this->initDone) {
		return this->gpionum;
	}
	return 0xff;
}

/*
 * @brief get back the RMT channel if the driver initialized
 */

int ESP32_RMT_Tx::getRMTPort(void) {
	if( this->initDone) {
		return this->rmtport;
	}
	return 0xff;
}

void ESP32_RMT_Tx::sendIR(rmt_item32_t* item, int itemNum) {
	rmt_write_items(static_cast<rmt_channel_t>(this->rmtport), item,itemNum, true);
	rmt_wait_tx_done(static_cast<rmt_channel_t>(this->rmtport), portMAX_DELAY);
}

bool BaseRMTClass::CheckCommand(ProtocolCommands_t & command) {
	std::vector<ProtocolCommands_t>::iterator it;
	it = std::find(this->_commands.begin(), this->_commands.end(),command);
	return (it != this->_commands.end());
}

bool BaseRMTClass::AddCommand(ProtocolCommands_t & command) {
	if(this->CheckCommand(command))
		return true;
	this->_commands.push_back(command);
	return true;
}

void BaseRMTClass::DeleteCommand(ProtocolCommands_t & command) {
	this->_commands.erase(std::remove(this->_commands.begin(), this->_commands.end(),command),this->_commands.end());
}

bool BaseRMTClass::UpdateCommand(ProtocolCommands_t & command) {
	for (std::vector<ProtocolCommands_t>::iterator it = this->_commands.begin(); it!= this->_commands.end(); it++) {
		if(it->_name == command._name) {
			*(it) = command;
			return true;
		}
	}
	return false;
}

ProtocolCommands_t * BaseRMTClass::GetCommand(std::string name) {
	ProtocolCommands_t * l_rP = nullptr;
	for (std::vector<ProtocolCommands_t>::iterator it = this->_commands.begin(); it!= this->_commands.end(); it++) {
		if(it->_name == name) {
			l_rP = &(*it);
			return l_rP;
		}
	}
	return nullptr;
}




bool PulseDistanceCoding::CheckInput(rmt_item32_t * item) {
	bool l_bResult = false;
	// Header check
	l_bResult = ESP32_RMT_Rx::isInRange(item[0],  this->_data->_headerLow, this->_data->_headerHigh, this->_data->_tolerance);

	return l_bResult;
}



/*
 * Check the item for the right format. In case of any error according to the protocol return failed.
 * In case of everything fine it should return true and fill the address and the data variable.
 */
ErrData PulseDistanceCoding::DecodeInput(rmt_item32_t * item, uint& address, uint& data) {
	bool l_bResult = false;
	int counter = 0;
	uint32_t lengthOfProtocol = 0;
	address = 0;
	data = 0;

	// checking the Header Data
	l_bResult = ESP32_RMT_Rx::isInRange(item[counter],  this->_data->_headerLow, this->_data->_headerHigh, this->_data->_tolerance);
	if (l_bResult == false) {
		printf("Header Error");
		return ErrData::HeaderNoMatch;// not this protocol based on Header info
	}
	counter++;
	//Let's check what is in the Protocol...

	if(this->_data->_isAddress) {
		for(int i = 0; i< this->_data->_addressLength; i++, counter++) {

			//Check if the bit is High
			l_bResult = ESP32_RMT_Rx::isInRange(item[counter],this->_data->_highTimeLow,this->_data->_highTimeHigh, this->_data->_tolerance);

			// Do the trick check if reverse address in it here...
			// Purpose : minus one cycle to implement
			if(l_bResult && this->_data->_isInvertedAddressRequired) {
				l_bResult &= ESP32_RMT_Rx::isInRange(item[counter + this->_data->_addressLength],this->_data->_lowTimeLow,this->_data->_lowTimeHigh, this->_data->_tolerance);
			}

			if (l_bResult)
			{
				//Shift and set the last bit :-)
				address = address << 1;
				address |= 1;
			} else {

				// or low
				l_bResult = ESP32_RMT_Rx::isInRange(item[counter],this->_data->_lowTimeLow,this->_data->_lowTimeHigh, this->_data->_tolerance);
				if(l_bResult && this->_data->_isInvertedAddressRequired) {
					l_bResult &= ESP32_RMT_Rx::isInRange(item[counter + this->_data->_addressLength],this->_data->_highTimeLow,this->_data->_highTimeHigh, this->_data->_tolerance);
				}
				if (l_bResult) {
					// just shift
					address = address << 1;
				} else {
					// error....
					//maybe address clearing necessary?
					printf("AddressError");
					return ErrData::AddressError;
				}
			}
		}

		// Index info requires check because of the inverting address if requires
		if(this->_data->_isInvertedAddressRequired) {
			counter += this->_data->_addressLength;
		}
	}
	/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	 * Data Check start here
	 *------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	 */
	// idea check the whole length and substract the addresses lenght and do not forget the header....
	lengthOfProtocol = this->_data->_length - counter + 1;

	if(this->_data->_isDataInverseAddedRequired) {
		lengthOfProtocol /= 2;
	}
	//printf("lengthOfProtocol calculation check %i", lengthOfProtocol);
	for(int i = 0; i< lengthOfProtocol; i++, counter++) {
		l_bResult = ESP32_RMT_Rx::isInRange(item[counter],this->_data->_highTimeLow,this->_data->_highTimeHigh, this->_data->_tolerance);

		if(l_bResult && this->_data->_isDataInverseAddedRequired )
		{
			l_bResult &= ESP32_RMT_Rx::isInRange(item[counter + lengthOfProtocol], this->_data->_lowTimeLow, this->_data->_lowTimeHigh, this->_data->_tolerance);
		}
		if(l_bResult) {
			data = data << 1;
			data |= 1;
		} else {
			// check of low

			l_bResult  = ESP32_RMT_Rx::isInRange(item[counter],this->_data->_lowTimeLow,this->_data->_lowTimeHigh, this->_data->_tolerance);

			if( l_bResult && this->_data->_isDataInverseAddedRequired) {
				l_bResult &= ESP32_RMT_Rx::isInRange(item[counter + lengthOfProtocol] , this->_data->_highTimeLow, this->_data->_highTimeHigh, this->_data->_tolerance);
			}

			if(l_bResult) {
				data = data << 1;
			} else {
				return ErrData::DataError;
			}
		}
	}
	// Stop bit check???
	return ErrData::NoError;
}

/*
 * Purpose: Generate the output structure based on the Protocol data.
 * Input  : Pointer to the rmt structure, Unsigned Int for address and data
 * Return : void...
 */

void PulseDistanceCoding::GenerateOutput(rmt_item32_t* item, uint address, uint data) {
	//Generate Head
	int counter = 0;
	int remainingLength = 0;

	ESP32_RMT_Tx::generateRMTItem( & item[counter], static_cast<int>(this->_data->_headerHigh), static_cast<int>(this->_data->_headerLow) );
	// Check next item
	remainingLength = this->_data->_length;
	counter++;

	//if address avail
	if(this->_data->_isAddress) {
		for(int i = 0; i< this->_data->_addressLength; i++) {
			// swipe through the data

			if( (address >> (this->_data->_addressLength - i - 1)) & 0x1) {
				// Done : recalculate the address
				// Done : inverse address
				ESP32_RMT_Tx::generateRMTItem(& item[counter] , this->_data->_highTimeHigh, this->_data->_highTimeLow);
				if(this->_data->_isInvertedAddressRequired) {
					ESP32_RMT_Tx::generateRMTItem(& item[counter + this->_data->_addressLength] , this->_data->_lowTimeHigh, this->_data->_lowTimeLow);
				}
			}else{
				ESP32_RMT_Tx::generateRMTItem(& item[counter] , this->_data->_lowTimeHigh, this->_data->_lowTimeLow);
				if(this->_data->_isInvertedAddressRequired) {
					ESP32_RMT_Tx::generateRMTItem(& item[counter + this->_data->_addressLength] , this->_data->_highTimeHigh, this->_data->_highTimeLow);
				}
			}
			counter++;
		}

		//Task done: calculate the remaining length....
		// full minus the address length
		remainingLength -= this->_data->_addressLength;

		// minus the address length again --> inverse address
		// counter correction also required.
		if(this->_data->_isInvertedAddressRequired) {
			remainingLength -= this->_data->_addressLength;
			counter += this->_data->_addressLength;
		}

	}

	//Task done: calculate the remaining length....
	if (this->_data->_isDataInverseAddedRequired ) {
		remainingLength /= 2;
	}

	//Done: Generate Data
	for( int i=0; i< remainingLength; i++) {

		if((data >> (remainingLength - i -1)) & 0x1) {
			// Done : recalculate the Data
			// Done : inverse data
			ESP32_RMT_Tx::generateRMTItem(& item[counter] , this->_data->_highTimeHigh, this->_data->_highTimeLow);

			if(this->_data->_isDataInverseAddedRequired) {
				ESP32_RMT_Tx::generateRMTItem(& item[counter + remainingLength] , this->_data->_lowTimeHigh, this->_data->_lowTimeLow);
			}

		}else{
			ESP32_RMT_Tx::generateRMTItem(& item[counter] , this->_data->_lowTimeHigh, this->_data->_lowTimeLow);
			if(this->_data->_isDataInverseAddedRequired) {
				ESP32_RMT_Tx::generateRMTItem(& item[counter + remainingLength] , this->_data->_highTimeHigh, this->_data->_highTimeLow);
			}
		}
		counter++;
	}
	if(this->_data->_isDataInverseAddedRequired) {
		counter += remainingLength;
		remainingLength -= this->_data->_addressLength;

	}


	//Done: Stop bit
	if(this->_data->_isStop)
	{
		ESP32_RMT_Tx::generateRMTItem(& item[counter] , this->_data->_stopSignHigh, this->_data->_stopSignLow);
	}

	//return

}

/*
 * Purpose : Generate output from the name of the Protocol Command...
 * Make our life easier
 */

void PulseDistanceCoding::GenerateOutput(rmt_item32_t* item, std::string ProtocolCommandName) {
	ProtocolCommands_t *p = nullptr;
	p =	this->GetCommand(ProtocolCommandName);
	if( p!=nullptr)
	{
		this->GenerateOutput(item,p->_address, p->_data);
	}
}


/*
 * Purpose: Generate output items without to check details
 * data shall be stored in a proper way.
 */
void PulseDistanceCoding::GenerateOutPutRaw(rmt_item32_t* item, uint data) {

	int counter = 0;
	int remainingLength = 0;

	ESP32_RMT_Tx::generateRMTItem( & item[counter], static_cast<int>(this->_data->_headerHigh), static_cast<int>(this->_data->_headerLow) );
	// Check next item
	remainingLength = this->_data->_length;
	counter++;

	for(int i=0; i < remainingLength; i++ ) {
		if((data >> (remainingLength - i -1)) & 0x1) {
			// Done : recalculate the Data
			// Done : inverse data
			ESP32_RMT_Tx::generateRMTItem(& item[counter] , this->_data->_highTimeHigh, this->_data->_highTimeLow);

			if(this->_data->_isDataInverseAddedRequired) {
				ESP32_RMT_Tx::generateRMTItem(& item[counter + remainingLength] , this->_data->_lowTimeHigh, this->_data->_lowTimeLow);
			}
		}else{
			ESP32_RMT_Tx::generateRMTItem(& item[counter] , this->_data->_lowTimeHigh, this->_data->_lowTimeLow);
		}
		counter++;
	}
	if(this->_data->_isStop)
	{
		ESP32_RMT_Tx::generateRMTItem(& item[counter] , this->_data->_stopSignHigh, this->_data->_stopSignLow);
	}
	//return
}

/*
 * Purpose create a JSON object from the object
 * IDEA to send over MQTT or store via SDCARD ....
 * */

cJSON * PulseDistanceCoding::GetJSONOut() {
	cJSON * _object = NULL;
	cJSON * _data = NULL;
	_object = cJSON_CreateObject();
	cJSON_AddItemToObject(_object, "name", cJSON_CreateString(this->_data->_name.c_str()));
	cJSON_AddItemToObject(_object, "type", cJSON_CreateString("PulseDistanceCoding"));
	cJSON_AddItemToObject(_object, "format", _data = cJSON_CreateObject());


	cJSON_AddNumberToObject(_data, "headerHigh", this->_data->_headerHigh);
	cJSON_AddNumberToObject(_data, "headerLow", this->_data->_headerLow);
	cJSON_AddNumberToObject(_data, "length", this->_data->_length);
	cJSON_AddNumberToObject(_data, "hightTimeHigh", this->_data->_highTimeHigh);
	cJSON_AddNumberToObject(_data, "hightTimeLow", this->_data->_highTimeLow);
	cJSON_AddNumberToObject(_data, "lowTimeHigh", this->_data->_lowTimeHigh);
	cJSON_AddNumberToObject(_data, "lowTimeLow", this->_data->_lowTimeLow);
	if( this->_data->_isAddress) {
		cJSON_AddTrueToObject(_data, "isAddress");
	}else{
		cJSON_AddFalseToObject(_data, "isAddress");
	}
	cJSON_AddNumberToObject(_data, "addressLength", this->_data->_addressLength);


	return _object;
}


