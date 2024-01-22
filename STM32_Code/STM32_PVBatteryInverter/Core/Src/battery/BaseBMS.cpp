#include "battery/BaseBMS.hpp"


// Must be implemented!
BaseBMS::~BaseBMS() { }

BaseBMS::BaseBMS(uint16_t address)
{
    tx_msg.header = 0xAAAA;
    tx_msg.srcAddr = 1<<8;
    tx_msg.dstAddr = address<<8;
}

void BaseBMS::set_serial_fd(int serial_fd)
{
    serial_fd_ = serial_fd;
}
