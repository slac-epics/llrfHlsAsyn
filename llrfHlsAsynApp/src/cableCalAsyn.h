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

    private:
        void        *pDrv;
        char        *port;
        char        *path;
        calDspFw    calDsp;
        void ParameterSetup(void);


    protected:
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int firstCableCalParam;
#define FIRST_CABLECAL_PARAM   firstCableCalParam
#endif /* ASYN VERSION CHECK under 4.32 */

        int p_cal_pulse_start;
        int p_cal_pulse_end;
        int p_cal_window_start;
        int p_cal_window_end;
        int p_cal_dac_enable;
        int p_cal_pulse_seq_delay;

        int p_cal_loop_delay[NUM_CAL_ADC];
        int p_cal_phase[NUM_CAL_PULSE][NUM_CAL_ADC];
        int p_cal_ampl[NUM_CAL_PULSE][NUM_CAL_ADC];
        int p_cal_freq_offset[NUM_CAL_PULSE];
        

#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)      
        int lastCableCalParam;
#define LAST_CABLECAL_PARAM   lastCableCalParam
#endif /* asyn version check, under 4.32 */
};


#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
#define NUM_CABLECAL_DET_PARAMS ((int)(&LAST_CABLECAL_PARAM - &FIRST_CABLECAL_PARAM-1))
#endif /* asyn version check, under 4.32 */


// single instance
#define CAL_PULSE_START_STR        "calPulse_start"
#define CAL_PULSE_END_STR          "calPulse_end"
#define CAL_WINDOW_START_STR       "calWindow_start"
#define CAL_WINDOW_END_STR         "calWindow_end"
#define CAL_DAC_ENABLE_STR         "calDac_enable"
#define CAL_PULSE_SEQ_DELAY_STR    "calPulseSeqDelay"
// per ADC cannel instance
#define CAL_LOOP_DELAY_STR         "calLoopDelay_C%d"
// per pulse and per ADC instances
#define CAL_PHASE_STR              "calPhase_P%dC%d"
#define CAL_AMPL_STR               "calAmpl_P%dC%d"
// per pulse interface
#define CAL_FREQ_OFFSET_STR        "calFreqOffset_P%d"


#endif   /* _CABLECALASYN_H    */
