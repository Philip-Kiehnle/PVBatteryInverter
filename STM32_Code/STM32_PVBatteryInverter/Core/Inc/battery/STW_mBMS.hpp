#ifndef BMS_STW_mBMS_H_
#define BMS_STW_mBMS_H_

#include <stdint.h>
#include <string.h>

#include "BaseBMS.hpp"

class STW_mBMS : public BaseBMS
{
public:
    STW_mBMS(uint16_t address);

    virtual int get_summary() override;
    virtual int get_cellStack(uint8_t pack) override;
    virtual int get_temperatureSensors(uint8_t pack) override;
    virtual int shipmode(uint8_t mask) override;
    virtual int resetErrors() override;
    virtual int batteryOn() override;
    virtual int batteryOff() override;
    // virtual int query_data(char* target_addr, unsigned int len) override = 0;
    // virtual int custom_query(unsigned int response_len) override = 0;
    // virtual int custom_string_query(std::string custom_string) override = 0;


    //DualBMS dual_bms;
    //CellStack cellStack[2];
    // TemperatureSensors temperatureSensors[2];

    const uint32_t batID = 40328804;

    const uint32_t battery_cells = 96;

    // for battery
    uint16_t V_CELL_MIN_PROTECT_mV() const override {
        return 3300;
    }
    uint16_t V_CELL_MIN_POWER_REDUCE_mV() const override {
        return 3400;
    }
    uint16_t V_CELL_MAX_POWER_REDUCE_mV() const override {
        return 3950;
    }
    uint16_t V_CELL_MAX_PROTECT_mV() const override {
        return 4050;
    }

    // for inverter limits
    float V_CHARGE_STOP() const override {
        return ( battery_cells * (V_CELL_MAX_POWER_REDUCE_mV()+V_CELL_MAX_PROTECT_mV())/2 ) /1000;
    }
    float V_MAX_PROTECT() const override {
        return (battery_cells*V_CELL_MAX_PROTECT_mV())/1000;
    }
    float V_DISCHARGE_STOP() const override {
        return ( battery_cells * (V_CELL_MIN_PROTECT_mV()+V_CELL_MIN_POWER_REDUCE_mV())/2 ) /1000;
    }
    float V_MIN_PROTECT() const override {
        return (battery_cells*V_CELL_MIN_PROTECT_mV())/1000;
    }
    float I_CHARGE_MAX() const override {
        return 4.0;
    }
    float I_DISCHARGE_MAX() const override {
        return 4.0;
    }

    int balancingEnable();

};


typedef struct {
    uint16_t cell_voltage_max_mV;
    uint16_t cell_voltage_max_index;
    uint16_t cell_voltage_min_mV;
    uint16_t cell_voltage_min_index;
} __attribute__((__packed__)) csc_cell_volt_min_max_t;
//BO_ 1125 CSC04_CELL_VOLT_MIN_MAX: 8 CSC04
// SG_ CSC04_Cell_Voltage_Min_Index : 48|16@1+ (1,0) [0|60] ""  SC_CAN2,MC_CAN4
// SG_ CSC04_Cell_Voltage_Min : 32|16@1+ (1,0) [0|6000] "mV"  SC_CAN2,MC_CAN4
// SG_ CSC04_Cell_Voltage_Max_Index : 16|16@1+ (1,0) [0|60] ""  SC_CAN2,MC_CAN4
// SG_ CSC04_Cell_Voltage_Max : 0|16@1+ (1,0) [0|6000] "mV"  SC_CAN2,MC_CAN4

#endif  // BMS_STW_mBMS_H_
