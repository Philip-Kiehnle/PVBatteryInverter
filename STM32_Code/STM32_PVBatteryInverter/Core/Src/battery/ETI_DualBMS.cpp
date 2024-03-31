
#include "battery/BaseBMS.hpp"
#include "battery/ETI_DualBMS.hpp"

//#include <iostream>

ETI_DualBMS::ETI_DualBMS(uint16_t address) : BaseBMS(address)
{
    // battery_cells = 80;
    // //Vcharge_stop = battery_cells*4.134;  // ~99% SoC
    // Vmax_protect = battery_cells*4.18;  // ~100% SoC
    // Vdischarge_stop = battery_cells*3.2;  // 3.2V ~13.5% SoC
    // Icharge_max = 7.0;  // Samsung INR21700-40T allows 6.0A rated charge -> 12.0A for two cells in parallel
    // Idischarge_max = 7.5;  // BMS limit 8A
}


/******************/
/* config section */
/******************/

//#define DUAL_BATTERY  // if two battery packs are used

#define R_CELL_MILLIOHM (8+1) // internal resistance of cell and cell connector in mOhm
#define PARALLEL_CELLS 1

// LiitoKala 32700 7000mAh LiFePO4 battery: 35A cont. discharge, 55A max
// curve was not available -> using Liitokala 32700 5.5Ah data from
// https://www.kirich.blog/obzory/akkumulyatory/862-liitokala-32700-lifepo4-i-sravnenie-ih-s-analogichnymi-akkumulyatorami-varicore.html
// the curve with 0.5C/3.25A was fitted with Engauge Digitizer
// postprocessing with python included compensation with 3.5A*8mOhm, because our cell has 7Ah
// this is used as the OCV curve for now:
uint16_t SoC_lookup_table[100] = {
	2647,2754,2837,2902,2948,2985,3012,3035,3052,3068,  // 1%, 2%, ...
	3081,3092,3100,3108,3115,3123,3130,3136,3142,3148,
	3152,3156,3160,3164,3168,3172,3175,3177,3179,3182,
	3184,3186,3188,3190,3193,3195,3197,3199,3201,3202,
	3204,3205,3207,3208,3210,3211,3212,3214,3215,3216,
	3217,3218,3219,3220,3221,3223,3224,3225,3227,3228,
	3229,3230,3231,3232,3233,3235,3236,3237,3238,3240,
	3241,3243,3244,3246,3247,3248,3249,3250,3251,3252,
	3254,3256,3258,3259,3260,3261,3262,3264,3265,3266,
	3267,3268,3270,3271,3272,3273,3274,3279,3312,3649  // ... 99%, 100%
};


void ETI_DualBMS::estimateSoC()
{
	int16_t current_mA = dual_bms.battery[0].current_mA;
	// simple 5 second relaxation model
#define I_CHRG_AVG_SEC 5
	static int i_chrg_sum_mA[I_CHRG_AVG_SEC] = {0};
	i_chrg_sum_mA[0] += current_mA - i_chrg_sum_mA[I_CHRG_AVG_SEC-1];
	for (int i=0; i<I_CHRG_AVG_SEC-1; i++){
		i_chrg_sum_mA[I_CHRG_AVG_SEC-1-i] = i_chrg_sum_mA[I_CHRG_AVG_SEC-2-i];
	}
	// voltage increased when charging -> todo: Kalman filter
	//int i_chrg_avg_mA = i_chrg_sum_mA[0]/I_CHRG_AVG_SEC;
	// 5A for last 5sec, max. 100mV increase
	uint16_t charge_comp_mV = 0;//(100 * std::clamp(i_chrg_avg_mA, 0, 5000)) / 5000;

	uint16_t avgVcell_mV = batteryStatus.voltage_100mV*100/battery_cells;  // 100mV resolution causes 5% SoC step at 69%

	avgVcell_mV -= (R_CELL_MILLIOHM * (int32_t)current_mA)/PARALLEL_CELLS/1000;  // current compensation: I>0 means charging
	for (int soc=1; soc<=100; soc++) {
		if (avgVcell_mV >= (SoC_lookup_table[soc-1]+charge_comp_mV)) {
			batteryStatus.soc = soc;  // todo max soc change 1
		}
	}
}


int ETI_DualBMS::get_summary()
{
    std::string cmd("binrS\r");  // (binary read summary)
    cmd.copy((char*)&tx_msg.data, cmd.length());
    tx_msg.dLen = cmd.length();

    if (custom_query(sizeof(dual_bms)) == 0) {

        DualBMS* dual_bms_tmp = (DualBMS*)rx_msg.data;  // temporary to check CRC first

        // calc CRC of BMS struct without the CRC itself
        uint8_t crc = 0;
        for (unsigned int k=0; k<(sizeof(dual_bms)-1); k++) {

            uint8_t data = ((uint8_t*)dual_bms_tmp)[k];

            crc = crc ^ data;

            for (uint8_t i = 0; i < 8; i++)
            {
                if ((crc & 0x80) != 0)
                {
                    crc <<= 1;
                    crc ^= 0x07;
                }
                else
                {
                    crc <<= 1;
                }
            }
        }

        if (crc != dual_bms_tmp->crc) {
            //std::cout << "[ETI_DualBMS::get_summary] crc=" << crc << " != " << dual_bms_tmp->crc << "=dual_bms_tmp->crc" << std::endl;
            return -2;
        } else {
            //std::cout << "[ETI_DualBMS::get_summary] crc=" << crc << " == " << dual_bms_tmp->crc << "=dual_bms_tmp->crc" << std::endl;
            // copy if checksum of received struct is okay
            for (unsigned int i=0; i<sizeof(dual_bms); i++) {
                ((char*)&dual_bms)[i] = rx_msg.data[i];
            }
        }

        batteryStatus.voltage_100mV = (dual_bms.battery[0].voltage_mV+dual_bms.battery[1].voltage_mV)/100;
        batteryStatus.soc = (dual_bms.battery[0].soc + dual_bms.battery[1].soc) / 2;
        batteryStatus.power_W = (float)dual_bms.battery[0].voltage_mV /1000 * ((float)dual_bms.battery[0].current_mA) /1000
                                +(float)dual_bms.battery[1].voltage_mV /1000 * ((float)dual_bms.battery[1].current_mA) /1000;
        if (dual_bms.battery[1].vCell_min_mV == 0) {  // todo find better solution for single pack
        	batteryStatus.minVcell_mV = dual_bms.battery[0].vCell_min_mV;
        } else {
            batteryStatus.minVcell_mV = std::min(dual_bms.battery[0].vCell_min_mV, dual_bms.battery[1].vCell_min_mV);
        }
        batteryStatus.maxVcell_mV = std::max(dual_bms.battery[0].vCell_max_mV, dual_bms.battery[1].vCell_max_mV);

        int8_t temperature_bat0 = dual_bms.battery[0].temperatureBattery;
#ifdef DUAL_BATTERY
        int8_t temperature_bat1 = dual_bms.battery[1].temperatureBattery;

        // if only one sensor is connected
        if (temperature_bat0 == -100)
            temperature_bat0 = temperature_bat1;
        if (temperature_bat1 == -100)
            temperature_bat1 = temperature_bat0;

        batteryStatus.minTemp = std::min(temperature_bat0, temperature_bat1);
        batteryStatus.maxTemp = std::max(temperature_bat0, temperature_bat1);
#else
        batteryStatus.minTemp = temperature_bat0;
        batteryStatus.maxTemp = temperature_bat0;
#endif //DUAL_BATTERY

        return 0;
    }

    return -1;
}

int ETI_DualBMS::query_data(char* target_addr, unsigned int len)
{

    if (custom_query(len) == 0) {

        for (unsigned int i=0; i<len; i++) {
            target_addr[i] = rx_msg.data[i];
        }
        return 0;
    }

    return -1;
}

int ETI_DualBMS::get_cellStack(uint8_t pack)
{
    std::string cmd("binrCS");  // (binary read cell stack)
    if (pack == 0) {
        cmd.append("a\r");
    } else if (pack == 1) {
        cmd.append("b\r");
    } else {
        return -1;
    }
    cmd.copy((char*)&tx_msg.data, cmd.length());
    tx_msg.dLen = cmd.length();

    return query_data((char*)&cellStack[pack], sizeof(cellStack[pack]));
}


int ETI_DualBMS::get_temperatureSensors(uint8_t pack)
{
    std::string cmd("binrTS");  // (binary read temperature sensors)
    if (pack == 0) {
        cmd.append("a\r");
    } else if (pack == 1) {
        cmd.append("b\r");
    } else {
        return -1;
    }

    cmd.copy((char*)&tx_msg.data, cmd.length());
    tx_msg.dLen = cmd.length();

    return query_data((char*)&temperatureSensors[pack], sizeof(temperatureSensors[pack]));  // todo:received5 1 sensor but 15 for 3 ...
}

// custom query does not transmit this fields: ctrl, fct, d_len and checksum
int ETI_DualBMS::custom_query(unsigned int response_len)
{
    //serial_baud(serial_fd_, B115200);
    serial_write(serial_fd_, (char*)&tx_msg, 6);        
    serial_write(serial_fd_, (char*)&tx_msg.data, tx_msg.dLen);

#define RX_BUFLEN 6
    uint8_t rx_buf[RX_BUFLEN+1];  // without +1 android causes "stack corruption detected"
    uint16_t len = serial_read(serial_fd_, (char*)&rx_buf, RX_BUFLEN, 100*1000);  // 100ms

    bool header_detected = false;

    if (len == RX_BUFLEN) {

        // search msg start sequence
        for (int i = 0; i<(RX_BUFLEN-1); i++ ) {
            //printf("i%d\n", i);
            if (rx_buf[i] == 0xAA && rx_buf[i+1] == 0xAA) {
                header_detected = true;
                memcpy((uint8_t*)&rx_msg, rx_buf+i, RX_BUFLEN-i);
                if (i != 0) {
                    serial_read(serial_fd_, (char*)&rx_msg+RX_BUFLEN-i, i, 20*1000);  // 20ms
                }
                break;
            }
        }
    }

    if (header_detected) {
        len = serial_read(serial_fd_, (char*)&rx_msg+9, response_len, 100*1000);  // 100ms

        if (len == response_len) {
            return 0;
//        } else {
//            std::cout << "[ETI_DualBMS::custom_query] Invalid packetLen! received" << len << " expected " << response_len << std::endl;
        }
//    } else {
//        std::cout << "[ETI_DualBMS::custom_query] Invalid header received!" << std::endl;
    }

    return -1;
}

int ETI_DualBMS::shipmode(uint8_t mask)
{
    std::string cmd;
    if (mask == 0b00111111) {
        cmd = "a\r";
    } else if ( mask == 0b00001001) {
        cmd = "a1a4\r";
    } else {
//        std::cout << "[ETI_DualBMS::shipmode] Mask not supported!" << std::endl;
        return -1;
    }
    cmd.copy((char*)&tx_msg.data, cmd.length());
    tx_msg.dLen = cmd.length();

    if (custom_query(3) != 0) {
//        std::cout << "[ETI_DualBMS::shipmode] no response received!" << std::endl;
        return -1;
    }
    return 0;
}

int ETI_DualBMS::resetErrors()
{
    std::string cmd("resetErrors\r");
    cmd.copy((char*)&tx_msg.data, cmd.length());
    tx_msg.dLen = cmd.length();

    if (custom_query(3) != 0) {
//        std::cout << "[ETI_DualBMS::resetErrors] no response received!" << std::endl;
        return -1;
    }
    return 0;
}

int ETI_DualBMS::custom_string_query(std::string custom_string)
{
    std::string cmd = custom_string + "\r";
    cmd.copy((char*)&tx_msg.data, cmd.length());
    tx_msg.dLen = cmd.length();

    if (custom_query(3) != 0) {
//        std::cout << "[ETI_DualBMS::custom_string_query] no ACK response received!" << std::endl;
        return -1;
    }
    return 0;
}

int ETI_DualBMS::batteryOn()
{
    std::string cmd("e\r");  // (enable FETs)
    cmd.copy((char*)&tx_msg.data, cmd.length());
    tx_msg.dLen = cmd.length();

    if (custom_query(3) != 0) {
//        std::cout << "[ETI_DualBMS::on] no response received!" << std::endl;
        return -1;
    }
    return 0;
}

int ETI_DualBMS::batteryOff()
{
    std::string cmd("d\r");  // (disable FETs)
    cmd.copy((char*)&tx_msg.data, cmd.length());
    tx_msg.dLen = cmd.length();

    if (custom_query(3) != 0) {
//        std::cout << "[ETI_DualBMS::off] no response received!" << std::endl;
        return -1;
    }
    return 0;
}


bool ETI_DualBMS::tempLowWarn()
{
	return false;  // TODO
	//return (batteryStatus.minTemp <= T_CELL_MIN_WARN());
}

bool ETI_DualBMS::tempHighWarn()
{
	return false; //TODO
	//return (batteryStatus.maxTemp >= T_CELL_MAX_WARN());
}

bool ETI_DualBMS::tempWarn()
{
	return false;//TODO
	//return (tempLowWarn() || tempHighWarn());
}


