#include <hybridinverter_modbus.h>
#include <BatteryManagement/bms_types.h>
#include <common.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern          "C"
{
#endif

struct CellStack cellstack;
modbus_param_rw_t modbus_param_rw;


bool modbus_p_ac_soft_update;
bool modbus_p_ac_hard_update;


//Function for initialization modbus on you device
mbus_t mbus_hybridinverter_open(Modbus_Conf_t *pconf)
{
	 mbus_t mb;

	 pconf->devaddr = 10;
	 pconf->device = (void*) 0;

	//Define read callback function
    pconf->read = &mbus_hybridinverter_read;
	
	//Define write callback function
    pconf->write = &mbus_hybridinverter_write;

    //Open modbus context
	mb = mbus_open(pconf);

	modbus_param_rw = MODBUS_PARAM_RW_DEFAULT;


    if (mb < 0 ) return (mbus_t)MBUS_ERROR;

    return mb;
}


/* It's modbus request on read register by logical address (la) */
uint16_t mbus_hybridinverter_read(uint32_t la)
{
	// read only registers
	constexpr uint16_t OFFSET = 30001;
	if (la >= OFFSET && la < OFFSET+sizeof(cellstack.vCell_mV)/2) {
		int addr = la-OFFSET;

		cellstack.vCell_mV[0] = 100;
		cellstack.vCell_mV[1] = 1;
		cellstack.vCell_mV[2] = 2;
		cellstack.vCell_mV[3] = 3;

		constexpr uint16_t CELLSTACK_START_ADDR = 0;
		if (addr >= CELLSTACK_START_ADDR) {
			return cellstack.vCell_mV[addr-CELLSTACK_START_ADDR];
		}

	// read and write registers
	} else if (la >= 40001 && la < 40001+sizeof(modbus_param_rw)/2) {
		int addr = la-40001;
		uint16_t* regs = (uint16_t*) &modbus_param_rw;
		return regs[addr];
	}

	//return mbus_response(mb_context, MBUS_RESPONSE_ILLEGAL_DATA_ADDRESS);
	return 0;
}


//Function for write to some register by logic address
uint16_t mbus_hybridinverter_write(uint32_t la, uint16_t value)
{
    //printf("We write: %d %d\n",la, value);
	constexpr uint16_t OFFSET = 40001;

	if (la >= OFFSET && la < OFFSET+sizeof(modbus_param_rw)/2) {
		int addr = la-OFFSET;

		uint16_t* regs = (uint16_t*)&modbus_param_rw;
		regs[addr] = value;

		if (addr == ((uint32_t)&modbus_param_rw.p_ac_soft_W-(uint32_t)&modbus_param_rw)/2) {
			modbus_p_ac_soft_update = true;
		} else if (addr == ((uint32_t)&modbus_param_rw.p_ac_hard_W-(uint32_t)&modbus_param_rw)/2) {
			modbus_p_ac_hard_update = true;
		}
	}

    return value;
}



#ifdef __cplusplus
}
#endif
