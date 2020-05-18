#ifndef _LLRFHLSASYN_H
#define _LLRFHLSASYN_H


#include <asynPortDriver.h>
#include <epicsEvent.h>
#include <epicsTypes.h>
#include <epicsTime.h>

#include <cpsw_api_user.h>
#include <atcaCommon.h>
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

    private:
        char *port;
        char *path;


    protected:
};


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
#define REF_WIGHT_STR                "ref_weight_ch%d"   // weight average for reference, for each channel, array[10]
#define FB_WEIGHT_STR                "fb_weight_ch%d"    // weight average for feedback, for each channel,  array[10]
#define P_OFFSET_STR                 "p_offset_ch%d"     // phase offset, for each channel, array[10]
#define P_DES_STR                    "p_des_ts%d"        // desired phase, for each timeslot, array[18]
#define A_DES_STR                    "a_des_ts%d"        // desired amplitude, for each timeslot, array[18]
#define P_CH_STR                     "p_act_ch%d"        // actual phase, for each channel, array[10]
#define A_CH_STR                     "a_act_ch%d"        // actual amplitude, for each channel, array[10]
#define P_FB_STR                     "p_fb_ts%d"         // actual phase for feedback, for each timeslot, array[18]
#define A_FB_STR                     "a_fb_ts%d"         // actual amplitude for feedback, for each timeslot, array[18]
#define P_REF_STR                    "p_ref_ts%d",       // actual phase for reference, for each timeslot,  array[18]
#define A_REF_STR                    "a_ref_ts%d"        // actual amplitude for reference, for each timeslot, array[18]
#define P_SET_STR                    "p_set_ts%d"        // phase set value, for each timeslot, array[18]
#define A_SET_STR                    "a_set_ts%d"        // amplitude set value, for each timeslot, array[18]
#define AVG_WINDOW_STR               "avg_window"        // average window, length = 4096
#define I_WF_STR                     "i_wf_ch%d"         // i waveform, for each channel, array[10], length = 4096
#define Q_WF_STR                     "q_wf_ch%d"         // q waveform, for each channel, array[10], length = 4096


#endif /* _LLRFHLSASYN_H */
