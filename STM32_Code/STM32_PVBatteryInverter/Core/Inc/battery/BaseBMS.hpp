#ifndef BMS_BaseBMS_H_
#define BMS_BaseBMS_H_

#include <stdint.h>
#include <string.h>

extern "C" {
    #include "serial.h"
    #include "bms_types.h"
}

class BaseBMS
{
public:
    virtual ~BaseBMS() = 0;

    virtual void set_serial_fd(int serial_fd);
    virtual int get_summary() = 0;
    virtual int get_cellStack(uint8_t pack) = 0;
    virtual int get_temperatureSensors(uint8_t pack) = 0;
    virtual int shipmode(uint8_t mask) = 0;
    virtual int resetErrors() = 0;
    virtual int batteryOn() = 0;
    virtual int batteryOff() = 0;
    virtual void estimateSoC() = 0;
    // virtual int query_data(char* target_addr, unsigned int len) = 0;
    // virtual int custom_query(unsigned int response_len) = 0;
    // virtual int custom_string_query(std::string custom_string) = 0;

    batteryStatus_t batteryStatus = {0};

    // virtual float get_Vcharge_stop()     const { return 1; }
    // virtual float get_Vmax_protect()     const { return Vmax_protect; }
    // virtual float get_Vdischarge_stop()  const { return Vdischarge_stop; }
    // virtual float get_Icharge_max()      const { return Icharge_max; }
    // virtual float get_Idischarge_max()   const { return Idischarge_max; }

    //virtual constexpr unsigned int Vcharge_stop() const = 0;
    virtual uint16_t V_CELL_MIN_PROTECT_mV() const = 0;
    virtual uint16_t V_CELL_MIN_POWER_REDUCE_mV() const = 0;
    virtual uint16_t V_CELL_MAX_POWER_REDUCE_mV() const = 0;
    virtual uint16_t V_CELL_MAX_PROTECT_mV() const = 0;

    virtual float V_CHARGE_STOP() const = 0;
    virtual float V_MAX_PROTECT() const = 0;
    virtual float V_DISCHARGE_STOP() const = 0;
    virtual float V_MIN_PROTECT() const = 0;
    virtual float I_CHARGE_MAX() const = 0;
    virtual float I_DISCHARGE_MAX() const = 0;

protected:
    BaseBMS(uint16_t address);

    uint16_t address;
    int serial_fd_;

    msg_t tx_msg;
    msg_t rx_msg;

    // uint32_t battery_cells = 1;
    // //float Vcharge_stop = battery_cells*3.4;
    // float Vmax_protect = battery_cells*3.5;
    // float Vdischarge_stop = battery_cells*2.9;
    // float Icharge_max = 0.1;
    // float Idischarge_max = 0.1;

};

#endif  // BMS_BaseBMS_H_
