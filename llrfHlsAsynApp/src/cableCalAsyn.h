#ifndef     _CABLECALASYN_H
#define     _CABLECALASYN_H

#include <asynPortDriver.h>
#include <epicsEvent.h>
#include <epicsTypes.h>
#include <epicsTime.h>

#include <cpsw_api_user.h>
#include <calDspFw.h>
#include <vector>
#include <string>
#include <dlfcn.h>

#include <stdio.h>
#include <sstream>
#include <fstream>




class cableCalAsynDriver
    : asynPortDriver {
    public:
        cableCalAsynDriver(void *pDrv, const char *portName, const char *pathString, const char *named_root = NULL);
        ~cableCalAsynDriver();
        asynStatus writeInt32(asynUser   *pasynUser, epicsInt32   value);
        asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

        void poll(void);
        void report(int interest);

    private:
        void        *pDrv;
        char        *port;
        char        *path;
        calDspFw    calDsp;
        char        slowDac_string[128];
        void ParameterSetup(void);


    protected:
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int firstCableCalParam;
#define FIRST_CABLECAL_PARAM   firstCableCalParam
#endif /* ASYN VERSION CHECK under 4.32 */

// PV parameters for engineering values
        int p_cal_pulse_start;
        int p_cal_pulse_end;
        int p_cal_window_start;
        int p_cal_window_end;
        int p_cal_dac_enable;
        int p_slow_dac_select;
        int p_slow_dac_string;
        int p_cal_pulse_seq_delay;

        int p_cal_loop_delay[NUM_CAL_ADC];
        int p_cal_phase[NUM_CAL_PULSE][NUM_CAL_ADC];
        int p_cal_ampl[NUM_CAL_PULSE][NUM_CAL_ADC];
        int p_cordic_phase;
        int p_cordic_ampl;
        int p_cal_freq_offset[NUM_CAL_PULSE];

// PV parameters for raw values        
        int p_raw_pulse_start;
        int p_raw_pulse_end;
        int p_raw_window_start;
        int p_raw_window_end;
        int p_raw_pulse_seq_delay;

        int p_raw_loop_delay[NUM_CAL_ADC];
        int p_raw_phase[NUM_CAL_PULSE][NUM_CAL_ADC];
        int p_raw_ampl[NUM_CAL_PULSE][NUM_CAL_ADC];
        int p_raw_cordic_phase;
        int p_raw_cordic_ampl;
        int p_raw_freq[NUM_CAL_PULSE];

#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)      
        int lastCableCalParam;
#define LAST_CABLECAL_PARAM   lastCableCalParam
#endif /* asyn version check, under 4.32 */
};


#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
#define NUM_CABLECAL_DET_PARAMS ((int)(&LAST_CABLECAL_PARAM - &FIRST_CABLECAL_PARAM-1))
#endif /* asyn version check, under 4.32 */

// enginerring values
// single instance
#define CAL_PULSE_START_STR        "calPulse_start"
#define CAL_PULSE_END_STR          "calPulse_end"
#define CAL_WINDOW_START_STR       "calWindow_start"
#define CAL_WINDOW_END_STR         "calWindow_end"
#define CAL_DAC_ENABLE_STR         "calDac_enable"
#define SLOWDAC_SEL_STR            "slowDac_select"
#define SLOWDAC_STRING_STR         "slowDac_string"
#define CAL_PULSE_SEQ_DELAY_STR    "calPulseSeqDelay"
// per ADC cannel instance
#define CAL_LOOP_DELAY_STR         "calLoopDelay_C%d"
// per pulse and per ADC instances
#define CAL_PHASE_STR              "calPhase_P%dC%d"
#define CAL_AMPL_STR               "calAmpl_P%dC%d"
// per pulse interface
#define CAL_FREQ_OFFSET_STR        "calFreqOffset_P%d"

// raw values
// single instace
#define RAW_PULSE_START_STR        "rawPulse_start"
#define RAW_PULSE_END_STR          "rawPulse_end"
#define RAW_WINDOW_START_STR       "rawWindow_start"
#define RAW_WINDOW_END_STR         "rawWindow_end"
#define RAW_PULSE_SEQ_DELAY_STR    "rawPulseSeqDelay"
// per ADC channel instace
#define RAW_LOOP_DELAY_STR         "rawLoopDelay_C%d"
// per pulse and per ADC instance
#define RAW_PHASE_STR              "rawPhase_P%dC%d"
#define RAW_AMPL_STR               "rawAmpl_P%dC%d"
// per pulse instance
#define RAW_FREQ_STR               "rawFreq_P%d"

#define CORDIC_PHASE_STR           "cordicPhase"
#define CORDIC_AMPL_STR            "cordicAmpl"
#define RAW_CORDIC_PHASE_STR       "rawCordicPhase"
#define RAW_CORDIC_AMPL_STR        "rawCordicAmpl"

#endif   /* _CABLECALASYN_H    */
