
#include <sml/sml_parser.h>
#include <sml/sml_crc16.h>

//#include <byteswap.h>
/* Swap bytes in 16 bit value.  */
#ifndef __bswap_constant_16
#define __bswap_constant_16(x)					\
  ((__uint16_t) ((((x) >> 8) & 0xff) | (((x) & 0xff) << 8)))
#endif

// /* Swap bytes in 32 bit value.  */
#ifndef __bswap_constant_32
#define __bswap_constant_32(x) \
     ((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) |                      \
      (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24))
#endif


int parse_sml(const char* data, int size, meterdata_t* meterdata)
{
    int status = SML_PARSE_FAIL;
    uint16_t sml_start_idx = 0;
    uint16_t sml_stop_idx = 0;

//    for (int j = 0; j<size; j++) {
//        printf("0x%.2X, ", (uint8_t)data[j]);
//    }
//#define BUFLEN 151
//uint8_t buf[BUFLEN] = {0x76, 0x05, 0x00, 0x0F, 0xB5, 0xCC, 0x62, 0x00, 0x62, 0x00, 0x72, 0x63, 0x07, 0x01, 0x77, 0x01, 0x0B, 0x0A, 0x01, 0x49, 0x53, 0x4B, 0x00, 0x05, 0x02, 0x12, 0x45, 0x07, 0x01, 0x00, 0x62, 0x0A, 0xFF, 0xFF, 0x72, 0x62, 0x01, 0x65, 0x00, 0x05, 0x3A, 0x46, 0x75, 0x77, 0x07, 0x01, 0x00, 0x60, 0x32, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x49, 0x53, 0x4B, 0x01, 0x77, 0x07, 0x01, 0x00, 0x60, 0x01, 0x00, 0xFF, 0x01, 0x01, 0x01, 0x01, 0x0B, 0x0A, 0x01, 0x49, 0x53, 0x4B, 0x00, 0x05, 0x02, 0x12, 0x45, 0x01, 0x77, 0x07, 0x01, 0x00, 0x01, 0x08, 0x00, 0xFF, 0x65, 0x00, 0x1C, 0x19, 0x04, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x65, 0x00, 0x01, 0xB2, 0xFC, 0x01, 0x77, 0x07, 0x01, 0x00, 0x02, 0x08, 0x00, 0xFF, 0x01, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x65, 0x00, 0x10, 0xC9, 0x92, 0x01, 0x77, 0x07, 0x01, 0x00, 0x10, 0x07, 0x00, 0xFF, 0x01, 0x01, 0x62, 0x1B, 0x52, 0x00, 0x53, 0xEF, 0xFB, 0x01, 0x01, 0x01, 0x63, 0x76, 0x2C};

    for (int i = 0; i < size-8; i++) {
        if ( (i > 53) && *((uint16_t*)(data+i)) == 0x0576 ) { // second SML-Message
            sml_start_idx = i;
        }

        if ( *((uint64_t*)(data+i)) == 0x65FF000801000107) {  // 1.8.1 Wirkarbeit Bezug (in 0.1 Wh) //0x07010001 0800FF65 -> bigend

            if ( *(uint8_t*) (data+i+17) == 0x65) {  // 0x65 -> datatype is uint32
                meterdata->e_consumed =  (uint32_t) __bswap_constant_32( *(uint32_t*) (data+i+18) );
            } else {
                return SML_PARSE_FAIL;
            }
        }

        if ( *((uint64_t*)(data+i)) == 0x01FF000802000107) {  // 2.8.1 Wirkarbeit Lieferung (in 0.1 Wh) //0x07010002 0800FF01 -> bigend

            if ( *(uint8_t*) (data+i+13) == 0x65) {  // 0x65 -> datatype is uint32
                meterdata->e_produced =  (uint32_t) __bswap_constant_32( *(uint32_t*) (data+i+14) );
            } else {
                return SML_PARSE_FAIL;
            }
        }

        if ( *((uint64_t*)(data+i)) == 0x01FF000710000107) {  // 16.7.0 Wirkleistung (Momentanwert in kW) //0x07010010 0700FF01 -> bigend

            if ( *(uint8_t*) (data+i+13) == 0x52) {  // 0x52 -> datatype is int8
                meterdata->power =  *(int8_t*) (data+i+14);
                sml_stop_idx = i+18;
                break;
            } else {  // 0x53 -> datatype is int16
                meterdata->power =  (int16_t) __bswap_constant_16( *(uint16_t*) (data+i+14) );
                sml_stop_idx = i+19;
                break;
            }
        }
    }

    uint16_t crc16_rx;

    if ( *(uint8_t*)&data[sml_stop_idx] == 0x62) {  // 0x62 -> datatype is uint8
        crc16_rx = *(uint8_t*)&data[sml_stop_idx+1];
    } else {  // 0x63 -> datatype is uint16
        crc16_rx = __bswap_constant_16( *(uint16_t*)&data[sml_stop_idx+1] );
    }

    if (sml_stop_idx < sml_start_idx) {  // prevents memory fault in case of comm errors
        return SML_PARSE_FAIL;
    }

    uint16_t crc16_calc = sml_crc16_calculate((unsigned char *) &data[sml_start_idx], sml_stop_idx-sml_start_idx);

//    printf("\nCRC16_rx: %.2X\n", crc16_rx);
//    printf("CRC16_sml: %.2X\n", crc16_calc);

//    int k = 0;
//    for (int j = sml_start_idx; j<sml_stop_idx+3; j++) {
//        printf("0x%.2X, ", (uint8_t)data[j]);
//    }

    if (crc16_rx == crc16_calc)
        status=SML_PARSE_OKAY;

    return status;
}
