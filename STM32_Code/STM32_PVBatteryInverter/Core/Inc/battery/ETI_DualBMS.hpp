#ifndef BMS_ETI_DualBMS_H_
#define BMS_ETI_DualBMS_H_

#include <stdint.h>
#include <string>

#include "BaseBMS.hpp"

class ETI_DualBMS : public BaseBMS
{
public:
    ETI_DualBMS(uint16_t address);

    virtual int get_summary() override;
    virtual int get_cellStack(uint8_t pack) override;
    virtual int get_temperatureSensors(uint8_t pack) override;
    virtual int shipmode(uint8_t mask) override;
    virtual int resetErrors() override;
    virtual int batteryOn() override;
    virtual int batteryOff() override;

    int custom_string_query(std::string custom_string);

    DualBMS dual_bms;
    CellStack cellStack[1];
    TemperatureSensors temperatureSensors[1];


    const uint32_t battery_cells = 15;

    float Vcharge_stop() const override {
        return battery_cells*3.5;  // ~99% SoC
    }
    float Vmax_protect() const override {
        return battery_cells*3.6;  // ~100% SoC
    }
    float Vdischarge_stop() const override {
        return battery_cells*2.9;  // LiitoKala discharge cutoff 2.5V
    }
    float Icharge_max() const override {
        return 3.0;  // LiitoKala allows 3.0A rated charge
    }
    float Idischarge_max() const override {
        return 7.5;  // BMS limit 8A;  LiitoKala allows 30A continuous, 36A max
    }

    uint16_t Vcell_MAX_P_REDUCE_mV() const override {
        return 3400; // BMS OV_THRESHOLD_mV 3450mV
    }
    uint16_t Vcell_MAX_PROTECT_mV() const override {
        return 3440;
    }

    // const uint32_t battery_cells = 80;
    // const float Vcharge_stop = battery_cells*4.134;  // ~99% SoC
    // const float Vmax_protect = battery_cells*4.18;  // ~100% SoC
    // const float Vdischarge_stop = battery_cells*3.2;  // 3.2V ~13.5% SoC
    // const float Icharge_max = 7.0;  // Samsung INR21700-40T allows 6.0A rated charge -> 12.0A for two cells in parallel
    // const float Idischarge_max = 7.5;  // BMS limit 8A

    //     uint32_t battery_cells = 96;
    // float Vcharge_stop = battery_cells*3.8;
    // float Vmax_protect = battery_cells*3.85;
    // float Vdischarge_stop = battery_cells*3.3;
    // float Icharge_max = 5.0;
    // float Idischarge_max = 7.0;

private:
    int query_data(char* target_addr, unsigned int len);
    int custom_query(unsigned int response_len);

};

#endif  // BMS_ETI_DualBMS_H_
