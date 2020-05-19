#ifndef _LLRFHLSASYN_H
#define _LLRFHLSASYN_H


#include <asynPortDriver.h>
#include <epicsEvent.h>
#include <epicsTypes.h>
#include <epicsTime.h>

#include <cpsw_api_user.h>
#include <llrfFw.h>
#include <vector>
#include <string>
#include <dlfcn.h>

#include <stdio.h>
#include <sstream>
#include <fstream>


#define NUM_CH           10        // number of channels
#define NUM_TIMESLOT     18        // number of timeslot
#define MAX_SAMPLES      4096      // number of samples in a waveform



class llrfHlsAsynDriver
    :asynPortDriver {
    public:
        llrfHlsAsynDriver(const char *portName, const char *pathString, const char *named_root = NULL);
        ~llrfHlsAsynDriver();
        asynStatus writeInt32(asynUser *pasynUser,   epicsInt32 value);
        asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    private:
        char *port;
        char *path;
        llrfFw  llrfHls;
        void ParameterSetup(void);


    protected:
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int firstLlrfHlsParam;
#define FIRST_LLRFHLS_PARAM   firstLlrfHlsParam
#endif /* ASYN VERSION CHECK under 4.32 */

        int p_p_ref_offset;                   // phase offset of reference
        int p_p_fb_offset;                    // phase offset for feedback
        int p_p_gain;                         // gain for phase loop
        int p_a_gain;                         // gain for amplitude loop
        int p_ref_subtraction_enable;         // reference subtraction control enable/disable
        int p_fb_phase_enable;                // phase loop control, enable/disable
        int p_fb_ampl_enable;                 // amplitude loop control, enable/disable
        int p_p_corr_upper;                   // phase correction upper limit
        int p_p_corr_lower;                   // phase correction lower limit
        int p_a_corr_upper;                   // amplitude correction upper limit
        int p_a_corr_lower;                   // amplitude correction lower limit
        int p_a_drv_upper;                    // amplitude drive upper limit
        int p_a_drv_lower;                    // amplitude drive lower limit
        int p_ref_weight_ch[NUM_CH];          // channel weight for reference
        int p_fb_weight_ch[NUM_CH];           // channel weight for feedback
        int p_p_offset_ch[NUM_CH];            // phase offset for each channel
        int p_p_des_ts[NUM_TIMESLOT];         // dessired phase for each timeslot
        int p_a_des_ts[NUM_TIMESLOT];         // desired amplitude for each timeslot
        int p_p_ch[NUM_CH];                   // actual phase for each channel
        int p_a_ch[NUM_CH];                   // actual amplitude for each channel
        int p_p_fb_ts[NUM_TIMESLOT];          // phase for feedback for each timeslot
        int p_a_fb_ts[NUM_TIMESLOT];          // amplitude for feedback for each timeslot
        int p_p_ref_ts[NUM_TIMESLOT];         // reference phase for each timeslot
        int p_a_ref_ts[NUM_TIMESLOT];         // reference amplitude for each timeslot
        int p_p_set_ts[NUM_TIMESLOT];         // phase ser value for each timeslot
        int p_a_set_ts[NUM_TIMESLOT];         // amplitude set value for each timeslot
        int p_avg_window;                     // average window
        int p_i_wf_ch[NUM_CH];                // i waveform for each channel
        int p_q_wf_ch[NUM_CH];                // q waveform for each channel

#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)      
        int lastLlrfHlsParam;
#define LAST_LLRFHLS_PARAM   lastLlrfHlsParam
#endif /* asyn version check, under 4.32 */

};


#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
#define NUM_LLRFHLS_DET_PARAMS ((int)(&LAST_LLRFHLS_PARAM - &FIRST_LLRFHLS_PARAM-1))
#endif /* asyn version check, under 4.32 */


#define P_REF_OFFSET_STR             "p_ref_offset"      // phase offset for reference, in degree
#define P_FB_OFFSET_STR              "p_fb_offset"       // phase offset for feedback, in degree
#define P_GAIN_STR                   "p_gain"            // gain for phase loop
#define A_GAIN_STR                   "a_gain"            // gain for amplitude loop
#define REF_SUBTRACTION_ENABLE_STR   "p_subtraction_enable"  // phase substraction contgrol enable/disable
#define FB_PHASE_ENABLE_STR          "fb_phase_enable"   // phase feedback control, enable/disable
#define FB_AMPL_ENABLE_STR           "fb_ampl_enable"    // amplitude feedback control, enable/disable
#define P_CORR_UPPER_STR             "p_corr_upper"      // phase correction upper limit
#define P_CORR_LOWER_STR             "p_corr_lower"      // phase correction lower limit
#define A_CORR_UPPER_STR             "a_corr_upper"      // amplitude correction upper limit
#define A_CORR_LOWER_STR             "a_corr_lower"      // amplitude correction lower limit
#define A_DRV_UPPER_STR              "a_drv_upper"       // amplitude drive upper limit
#define A_DRV_LOWER_STR              "a_drv_lower"       // amplitude drive lower limit
#define REF_WEIGHT_STR               "ref_weight_ch%d"   // weight average for reference, for each channel, array[10]
#define FB_WEIGHT_STR                "fb_weight_ch%d"    // weight average for feedback, for each channel,  array[10]
#define P_OFFSET_STR                 "p_offset_ch%d"     // phase offset, for each channel, array[10]
#define P_DES_STR                    "p_des_ts%d"        // desired phase, for each timeslot, array[18]
#define A_DES_STR                    "a_des_ts%d"        // desired amplitude, for each timeslot, array[18]
#define P_CH_STR                     "p_act_ch%d"        // actual phase, for each channel, array[10]
#define A_CH_STR                     "a_act_ch%d"        // actual amplitude, for each channel, array[10]
#define P_FB_STR                     "p_fb_ts%d"         // actual phase for feedback, for each timeslot, array[18]
#define A_FB_STR                     "a_fb_ts%d"         // actual amplitude for feedback, for each timeslot, array[18]
#define P_REF_STR                    "p_ref_ts%d"        // actual phase for reference, for each timeslot,  array[18]
#define A_REF_STR                    "a_ref_ts%d"        // actual amplitude for reference, for each timeslot, array[18]
#define P_SET_STR                    "p_set_ts%d"        // phase set value, for each timeslot, array[18]
#define A_SET_STR                    "a_set_ts%d"        // amplitude set value, for each timeslot, array[18]
#define AVG_WINDOW_STR               "avg_window"        // average window, length = 4096
#define I_WF_STR                     "i_wf_ch%d"         // i waveform, for each channel, array[10], length = 4096
#define Q_WF_STR                     "q_wf_ch%d"         // q waveform, for each channel, array[10], length = 4096


#endif /* _LLRFHLSASYN_H */
