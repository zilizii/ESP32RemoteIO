/*
 * ESP32Remote.h
 *
 *  Created on: 2020. dec. 15.
 *      Author: Dell
 */

#include <cstdint>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <map>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_intr.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include "cJSON.h"

#ifndef MAIN_ESP32REMOTE_H_
#define MAIN_ESP32REMOTE_H_

struct ProtocolData_t {
	std::string _name;

	uint _headerHigh;
	uint _headerLow;

	// 8 bit + 8bit reverse + 8bit command + 8bit inverse of bit
	uint _length; // NEC = 32 bit long
	bool _isAddress;
	uint _addressLength;  // length of the address 8 bit in NEC
	bool _isInvertedAddressRequired;

	uint _highTimeHigh;    //High pulse high Time
	uint _highTimeLow; //High pulse low Time
	uint _lowTimeHigh;	   //Low pulse high Time
	uint _lowTimeLow; //Low pulse low Time

	bool _isDataInverseAddedRequired;
	uint _tolerance;
	bool _isStop;
	uint _stopSignHigh;
	uint _stopSignLow;
};

struct ProtocolCommands_t {
	std::string _name; // hmm may not be here....
	uint _address;
	uint _data;
	ProtocolCommands_t();
	ProtocolCommands_t(std::string name, uint address, uint data) : _name(name), _address(address), _data(data) {}

	bool operator==(const ProtocolCommands_t& rhs) const {return this->_name == rhs._name && this->_address == rhs._address && this->_data == rhs._data ;}
};

enum ErrData : uint {NoError = 0, HeaderNoMatch, AddressError, DataError };

class ESP32Remote {
public:
	ESP32Remote();
	virtual ~ESP32Remote();
};

/****************
 * Description of a class : Create a IR Receive handler class
 * TODO: remove the protocol element functions and create IR protocol specific ones.
 */

class ESP32_RMT_Rx {
public:
	ESP32_RMT_Rx(int gpiopin, int channelnumber);
	virtual ~ESP32_RMT_Rx();
	void init();
	uint8_t readIRrec();
	static bool isInRange(rmt_item32_t item, int lowDuration, int highDuration, int tolerance);
private:
	bool initDone;
	int gpionum;
	int rmtport;

	bool NEC_is0(rmt_item32_t item);
	bool NEC_is1(rmt_item32_t item);
	uint8_t decodeNEC(rmt_item32_t *data, int numItems);
	void printTiming(rmt_item32_t *data, int numItems);
};


/****************
 * Description of a class : Create a IR handler class, no protocol specific implementation allowed (those should be in a protocol specific class)
 */
class ESP32_RMT_Tx {
public:
	ESP32_RMT_Tx(int gpiopin, int channelnumber);
	void init(void);
	static void generateRMTItem(rmt_item32_t* item, int high_us, int low_us);
	bool isInit(void);
	int getGpioNum(void);
	int getRMTPort(void);
	void sendIR(rmt_item32_t* item, int itemNum);
private:
	bool initDone;
	int gpionum;
	int rmtport;
};


/*
 * Description of a class : Base (abstract) class for the protocols
 * Responsible for the Creation/identification/Read functionalities in high level
 * Commands as well needs to store, because of not independent from the Protocol....
 */
class BaseRMTClass {
protected:
	ProtocolData_t * _data = NULL;
	std::vector<ProtocolCommands_t> _commands;
public:
	BaseRMTClass(ProtocolData_t *Protocol) : _data(Protocol) {};

	std::string getName()
	{
		if(this->_data != NULL)
			return this->_data->_name;
		else
			return nullptr;
	};

	uint getTolerance() {return this->_data->_tolerance;};
	void setTolerance(uint tolerance)
	{
		if (this->_data != NULL)
				this->_data->_tolerance = tolerance;
	};

	virtual ~BaseRMTClass(){};
	virtual void GenerateOutput(rmt_item32_t* item, uint address, uint data) = 0;
	virtual void GenerateOutput(rmt_item32_t* item, std::string ProtocolCommandName) = 0;
	// just send out the data
	virtual void GenerateOutPutRaw(rmt_item32_t* item, uint data)=0;
	virtual bool CheckInput(rmt_item32_t* item) =0;
	bool AddCommand(ProtocolCommands_t &);
	bool CheckCommand(ProtocolCommands_t &);
	void DeleteCommand(ProtocolCommands_t &);
	bool UpdateCommand(ProtocolCommands_t &);
//	virtual bool SendCommand(std::string command) = 0;
//	virtual bool SendCommand(ProtocolCommands_t command) = 0;
	ProtocolCommands_t * GetCommand(std::string command);
	virtual cJSON * GetJSONOut() = 0;
};

class PulseDistanceCoding : public BaseRMTClass {
private:

public:
	PulseDistanceCoding(ProtocolData_t *Protocol) : BaseRMTClass(Protocol) {};
	uint getHighTime() {return this->_data->_highTimeHigh;};
	void setHighTime(uint HighTime) {this->_data->_highTimeHigh = HighTime;};
	uint getHighTimeLow() {return this->_data->_highTimeLow;};
	void setHighTimeLow(uint HighLowTime) {this->_data->_highTimeLow = HighLowTime;};

	uint getLowTimeHigh() {return this->_data->_lowTimeHigh;};
	void setLowTimeHigh(uint LowTimeHigh) {this->_data->_lowTimeHigh = LowTimeHigh;};
	uint getLowTimeLow() {return this->_data->_lowTimeLow;};
	void setLowTimeLow(uint LowTimeLow) {this->_data->_lowTimeLow = LowTimeLow;};

	bool CheckInput(rmt_item32_t* item);
	ErrData DecodeInput(rmt_item32_t * item, uint& address, uint& data);
	void GenerateOutput(rmt_item32_t* item, uint address, uint data);
	void GenerateOutput(rmt_item32_t* item, std::string ProtocolCommandName);
	void GenerateOutPutRaw(rmt_item32_t* item, uint data);
	cJSON * GetJSONOut();
};


#endif /* MAIN_ESP32REMOTE_H_ */
