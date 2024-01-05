#include "battery/STW_mBMS.hpp"


STW_mBMS::STW_mBMS(uint16_t address) : BaseBMS(address)
{
}

int STW_mBMS::get_summary()
{

//    batteryStatus_t batteryStatus_temp = {0};
//    write(sockfd, &batteryStatus_temp, 1);  // write one byte to start reception
//    int status = read(sockfd, &batteryStatus_temp, sizeof(batteryStatus_temp));
//
//    // close the socket
//    close(sockfd);
//
//    if (status != sizeof(batteryStatus_temp)){
//        std::cout << "[STW_mBMS::get_summary] batteryStatus_temp size missmatch: got " << status << " expected " << sizeof(batteryStatus_temp) << std::endl;
//        return -1;
//    }
//
//    if (batID != batteryStatus_temp.batID) {
//        std::cout << "[STW_mBMS::get_summary] Battery ID missmatch: got " << batteryStatus_temp.batID << " expected " << batID << std::endl;
//        return -1;
//    }
//
//    batteryStatus = batteryStatus_temp;
    return 0;
}

int STW_mBMS::get_cellStack(uint8_t pack)
{
    return -1;
}

int STW_mBMS::get_temperatureSensors(uint8_t pack)
{
    return -1;
}

int STW_mBMS::shipmode(uint8_t mask)
{
    return 0;
}

int STW_mBMS::balancingEnable()
{
//BO_ 272 MC_FUNCTION_REQUEST: 8 MC_CAN4
// SG_ MC_Bal_Enable_Threshold : 40|1@1+ (1,0) [0|1] ""  CSC16,CSC15,CSC14,CSC13,CSC12,CSC11,CSC10,CSC09,CSC08,CSC07,CSC06,CSC05,CSC04,CSC03,CSC02,CSC01
// SG_ MC_Balancing_Target_Voltage : 48|16@1+ (1,0) [0|6000] "mV"  CSC16,CSC15,CSC14,CSC13,CSC12,CSC11,CSC10,CSC09,CSC08,CSC07,CSC06,CSC05,CSC04,CSC03,CSC02,CSC01
// SG_ MC_Balancing_Enable_CSC16 : 15|1@1+ (1,0) [0|1] ""  CSC16
// SG_ MC_Balancing_Enable_CSC15 : 14|1@1+ (1,0) [0|1] ""  CSC15
// SG_ MC_Balancing_Enable_CSC14 : 13|1@1+ (1,0) [0|1] ""  CSC14
// SG_ MC_Balancing_Enable_CSC13 : 12|1@1+ (1,0) [0|1] ""  CSC13
// SG_ MC_Balancing_Enable_CSC12 : 11|1@1+ (1,0) [0|1] ""  CSC12
// SG_ MC_Balancing_Enable_CSC11 : 10|1@1+ (1,0) [0|1] ""  CSC11
// SG_ MC_Balancing_Enable_CSC10 : 9|1@1+ (1,0) [0|1] ""  CSC10
// SG_ MC_Balancing_Enable_CSC09 : 8|1@1+ (1,0) [0|1] ""  CSC09
// SG_ MC_Balancing_Enable_CSC08 : 7|1@1+ (1,0) [0|1] ""  CSC08
// SG_ MC_Balancing_Enable_CSC07 : 6|1@1+ (1,0) [0|1] ""  CSC07
// SG_ MC_Balancing_Enable_CSC06 : 5|1@1+ (1,0) [0|1] ""  CSC06
// SG_ MC_Balancing_Enable_CSC05 : 4|1@1+ (1,0) [0|1] ""  CSC05
// SG_ MC_Balancing_Enable_CSC04 : 3|1@1+ (1,0) [0|1] ""  CSC04
// SG_ MC_Balancing_Enable_CSC03 : 2|1@1+ (1,0) [0|1] ""  CSC03
// SG_ MC_Balancing_Enable_CSC02 : 1|1@1+ (1,0) [0|1] ""  CSC02
// SG_ MC_Balancing_Enable_CSC01 : 0|1@1+ (1,0) [0|1] ""  CSC01
    return 0;
}

int STW_mBMS::resetErrors()
{
    return 0;
}

int STW_mBMS::batteryOn()
{
    return 0;
}

int STW_mBMS::batteryOff()
{
    return 0;
}


