
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

int ETI_DualBMS::get_summary()
{
    std::string cmd("binrS\r");  // (binary read summary)
    cmd.copy((char*)&tx_msg.data, cmd.length());
    tx_msg.dLen = cmd.length();

    if (custom_query(sizeof(dual_bms)) == 0) {

        for (int i=0; i<sizeof(dual_bms); i++) {
            ((char*)&dual_bms)[i] = rx_msg.data[i];
        }

        // calc CRC of BMS struct without the CRC itself
        uint8_t crc = 0;
        for (int k=0; k<(sizeof(dual_bms)-1); k++) {

            uint8_t data = ((uint8_t*)&dual_bms)[k];

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

        if (crc != dual_bms.crc) {
            //std::cout << "[ETI_DualBMS::get_summary] crc=" << crc << "!=" << dual_bms.crc << "dual_bms.crc" << std::endl;
            return -1;
        }

        batteryStatus.soc = (dual_bms.battery[0].soc + dual_bms.battery[1].soc) / 2;
        batteryStatus.minVcell_mV = std::min(dual_bms.battery[0].vCell_min_mV, dual_bms.battery[1].vCell_min_mV);
        batteryStatus.maxVcell_mV = std::max(dual_bms.battery[0].vCell_max_mV, dual_bms.battery[1].vCell_max_mV);
        batteryStatus.power_W = (float)dual_bms.battery[0].voltage_mV /1000 * ((float)dual_bms.battery[0].current_mA) /1000
                                +(float)dual_bms.battery[1].voltage_mV /1000 * ((float)dual_bms.battery[1].current_mA) /1000;

        voltage_mV = dual_bms.battery[0].voltage_mV+dual_bms.battery[1].voltage_mV;
        int8_t temperature_bat0 = dual_bms.battery[0].temperatureBattery;
        int8_t temperature_bat1 = dual_bms.battery[1].temperatureBattery;

        // if only one sensor is connected
        if (temperature_bat0 == -100)
            temperature_bat0 = temperature_bat1;
        if (temperature_bat1 == -100)
            temperature_bat1 = temperature_bat0;

        batteryStatus.minTemp = std::min(temperature_bat0, temperature_bat1);
        batteryStatus.maxTemp = std::max(temperature_bat0, temperature_bat1);
        
        return 0;
    }

    return -1;
}

int ETI_DualBMS::query_data(char* target_addr, unsigned int len)
{

    if (custom_query(len) == 0) {

        for (int i=0; i<len; i++) {
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

    return query_data((char*)&temperatureSensors[pack], sizeof(temperatureSensors[pack]));  // got 15 ecpected 24
    //return query_data((char*)&temperatureSensors[pack], 3*sizeof(TemperatureSensor));   // got 15 ecpected 24 too
    //return query_data((char*)&temperatureSensors[pack],
}

// custom query does not transmit this fields: ctrl, fct, d_len and checksum
int ETI_DualBMS::custom_query(unsigned int response_len)
{
    //serial_baud(serial_fd_, B115200);
    serial_write(serial_fd_, (char*)&tx_msg, 6);        
    serial_write(serial_fd_, (char*)&tx_msg.data, tx_msg.dLen);

#define RX_BUFLEN 6
    uint8_t rx_buf[RX_BUFLEN+1];  // without +1 android causes "stack corruption detected"
    int len = serial_read(serial_fd_, (char*)&rx_buf, RX_BUFLEN, 100*1000);  // 100ms

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


