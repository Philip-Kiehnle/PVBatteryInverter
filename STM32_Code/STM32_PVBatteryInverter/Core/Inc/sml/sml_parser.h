#ifndef SML_PARSER_H_
#define SML_PARSER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SML_PARSE_FAIL 0
#define SML_PARSE_OKAY 1

typedef struct {
	int power;
	unsigned int e_consumed;
	unsigned int e_produced;
} meterdata_t;

int parse_sml(const char* data, int size, meterdata_t* meterdata);


#ifdef __cplusplus
}
#endif

#endif /* SML_PARSER_H_ */
