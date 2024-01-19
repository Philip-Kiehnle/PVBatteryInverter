#ifndef INC_MPPTRACKER_HPP_
#define INC_MPPTRACKER_HPP_

#include <stdint.h>

#define MPP_PRINTF 1
#ifdef MPP_PRINTF
	#define debug_printf(fmt, ...) \
		do { if (MPP_PRINTF) printf(fmt, ##__VA_ARGS__); } while (0)
#else
	#define debug_printf(fmt, ...){}
#endif

extern const unsigned int MPPT_DUTY_ABSMAX;
extern const unsigned int MPPT_DUTY_MIN_BOOTSTRAP;
extern const unsigned int MPPT_FREQ;
extern const unsigned int INTERVAL_GLOB_MPPT_REGULAR_SEC;
extern const unsigned int INTERVAL_GLOB_MPPT_TRIG_EVENT_SEC;

typedef struct {
	float vin_min;
	float vin_max;
	float vout_min;
    float vout_max;
    uint16_t nr_pv_modules;
	uint16_t nr_substring_search_per_interval;
} mpptParams_t;

typedef struct {
	float v_mp;
	float i_mp;
	float coef_v_temp;
    float v_bypassDiode;
    uint16_t nr_bypassDiodes;
} pvModule_t;

typedef struct {
	float p_avg;
	float v;
    float v_start;
    uint16_t duty_raw;
    uint16_t nr_bypassDiodes_conducting;
} gmpp_t;

enum mppTrackerMode_t {MPP_localSearch, MPP_found, MPP_globalSearch, MPP_shortLocalSearch};

class MPPTracker
{
public:
    MPPTracker(const mpptParams_t MPPTPARAMS, const pvModule_t PVMODULE);

    mppTrackerMode_t mode() { return mode_; }

    void step(float p, float v, float vdc);


    uint16_t duty_raw;

    uint16_t nr_bypassDiodes_total;


private:

    void nextMode(mppTrackerMode_t nextMode);
    void setDuty(int duty_raw_unclamped);

    void step_localMPPT(float p, float v);

    mppTrackerMode_t mode_ = MPP_localSearch;
    gmpp_t gmpp;

    unsigned int modeRuntime;

    uint16_t DUTY_ABSMAX;

    float substring_v_mp;
    float i_mp;
    float v_bypassDiode;
    float coef_v_temp;
    unsigned int nr_substring_search_per_interval;

    float pv_temperature = 25;

    unsigned int nr_bypassDiodes_conducting = 0;
    float substring_voltage_estim;

	float v_prev;
	float p_prev;

	unsigned int duty_min;
	unsigned int duty_max;

};


#endif /* INC_MPPTRACKER_HPP_ */
