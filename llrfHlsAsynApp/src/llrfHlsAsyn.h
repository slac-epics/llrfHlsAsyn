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


#define NUM_FB_CH        10        // number of feedback channels
#define NUM_WINDOW        3        // number of window
#define NUM_TIMESLOT     18        // number of timeslot
#define MAX_SAMPLES      4096      // number of samples in a waveform



class llrfHlsAsynDriver
    :asynPortDriver {
    public:
        llrfHlsAsynDriver(const char *portName, const char *pathString, const char *hlsStream, const char *named_root = NULL);
        ~llrfHlsAsynDriver();
        asynStatus writeInt32(asynUser *pasynUser,   epicsInt32 value);
        asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
        asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
        void report(int interest);
        void poll(void);
        void pollStream(void);
        void updatePVs(void);
        void updatePhasePVsforAllChannels(void);
        void updateAmplPVsforAllChannels(void);
        void updateFeedbackPhasePVsforAllTimeslots(void);
        void updateFeedbackAmplPVsforAllTimeslots(void);
        void updateReferencePhasePVsforAllTimeslots(void);
        void updateReferenceAmplPVsforAllTimeslots(void);
        void updatePhaseSetPVsforAllTimeslots(void);
        void updateAmplSetPVsforAllTimeslots(void);
        void getIQWaveform(int channel);
        void flushIQWaveformsforAllChannels(void);
        void getFirmwareInformation(void);


    private:
        char *port;
        char *path;
        char *stream;
        llrfFw  llrfHls;
        Stream hls_stream_;

        epicsUInt32 version_;
        epicsUInt32 num_timeslot_;
        epicsUInt32 num_channel_;
        epicsUInt32 num_window_;
        epicsUInt32 max_pulse_len_;
        epicsUInt32 counter_;
        epicsUInt32 drop_counter_;

        epicsUInt32 stream_read_count;
        epicsUInt32 stream_read_size;

        uint8_t p_buf[1024*4];

        /* internal buffers */
        epicsFloat64  phase_wnd_ch[NUM_WINDOW][NUM_FB_CH];    // phase reading for all channels
        epicsFloat64  ampl_wnd_ch[NUM_WINDOW][NUM_FB_CH];     // amplitude reading for all channels
        epicsFloat64  fb_phase_ts[NUM_TIMESLOT];   // phase for feedback loop for all timeslots
        epicsFloat64  fb_ampl_ts[NUM_TIMESLOT];    // amplitude for feedback loop for all timeslots
        epicsFloat64  ref_phase_ts[NUM_TIMESLOT];  // phase for reference for all timeslots
        epicsFloat64  ref_ampl_ts[NUM_TIMESLOT];   // amplitude for reference for all timeslots
        epicsFloat64  phase_set_ts[NUM_TIMESLOT];  // phase set values for all timeslots
        epicsFloat64  ampl_set_ts[NUM_TIMESLOT];   // amplitude set values for all timeslots

        // epicsFloat64  avg_window[NUM_WINDOW][MAX_SAMPLES];     // average window

        epicsFloat64  i_wf_ch[NUM_FB_CH][MAX_SAMPLES];    // i waveform for all channels
        epicsFloat64  q_wf_ch[NUM_FB_CH][MAX_SAMPLES];    // q waveform for all channels

        void ParameterSetup(void);


    protected:
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int firstLlrfHlsParam;
#define FIRST_LLRFHLS_PARAM   firstLlrfHlsParam
#endif /* ASYN VERSION CHECK under 4.32 */

        int p_stream_enable;                  // steram enable: 1 , disable: 0
        int p_mode_config;                    // trigger mode disable:0, accel: 1, stdby: 2, accel_or_stdby: 3

        int p_version;
        int p_num_timeslot;
        int p_num_channel;
        int p_num_window;
        int p_max_pulse_len;

        int p_counter;
        int p_drop_counter;

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
        int p_ref_weight_ch[NUM_FB_CH];          // channel weight for reference
        int p_fb_weight_ch[NUM_FB_CH];           // channel weight for feedback
        int p_p_offset_ch[NUM_FB_CH];            // phase offset for each channel
        int p_p_des_ts[NUM_TIMESLOT];         // dessired phase for each timeslot
        int p_a_des_ts[NUM_TIMESLOT];         // desired amplitude for each timeslot
        int p_p_wnd_ch[NUM_WINDOW][NUM_FB_CH];                   // actual phase for each channel
        int p_a_wnd_ch[NUM_WINDOW][NUM_FB_CH];                   // actual amplitude for each channel
        int p_p_fb_ts[NUM_TIMESLOT];          // phase for feedback for each timeslot
        int p_a_fb_ts[NUM_TIMESLOT];          // amplitude for feedback for each timeslot
        int p_p_ref_ts[NUM_TIMESLOT];         // reference phase for each timeslot
        int p_a_ref_ts[NUM_TIMESLOT];         // reference amplitude for each timeslot
        int p_p_set_ts[NUM_TIMESLOT];         // phase ser value for each timeslot
        int p_a_set_ts[NUM_TIMESLOT];         // amplitude set value for each timeslot
        int p_avg_window[NUM_WINDOW];                     // average window
        int p_i_wf_ch[NUM_FB_CH];                // i waveform for each channel
        int p_q_wf_ch[NUM_FB_CH];                // q waveform for each channel

        int p_get_iq_wf_ch[NUM_FB_CH];        // get IQWaveform per channel

#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)      
        int lastLlrfHlsParam;
#define LAST_LLRFHLS_PARAM   lastLlrfHlsParam
#endif /* asyn version check, under 4.32 */

};


#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
#define NUM_LLRFHLS_DET_PARAMS ((int)(&LAST_LLRFHLS_PARAM - &FIRST_LLRFHLS_PARAM-1))
#endif /* asyn version check, under 4.32 */

#define STREAM_ENABLE_STR            "stream_enable"     // stream enable, disable control
#define MODE_CONFIG_STR              "mode_config"       // trigger mode configuraiton

#define VERSION_STR                  "version"
#define NUM_TIMESLOT_STR             "num_timeslot"
#define NUM_CHANNEL_STR              "num_channel"
#define NUM_WINDOW_STR               "num_window"
#define MAX_PULSE_LEN_STR            "max_pulse_len"
#define COUNTER_STR                  "counter"
#define DROP_COUNTER_STR             "drop_counter"

#define P_REF_OFFSET_STR             "p_ref_offset"      // phase offset for reference, in degree
#define P_FB_OFFSET_STR              "p_fb_offset"       // phase offset for feedback, in degree
#define P_GAIN_STR                   "p_gain"            // gain for phase loop
#define A_GAIN_STR                   "a_gain"            // gain for amplitude loop
#define REF_SUBTRACTION_ENABLE_STR   "ref_subtraction_enable"  // phase substraction contgrol enable/disable
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
#define P_WND_CH_STR                 "p_act_w%dch%d"        // actual phase, for each channel, array[3][10]
#define A_WND_CH_STR                 "a_act_w%dch%d"        // actual amplitude, for each channel, array[3][10]
#define P_FB_STR                     "p_fb_ts%d"         // actual phase for feedback, for each timeslot, array[18]
#define A_FB_STR                     "a_fb_ts%d"         // actual amplitude for feedback, for each timeslot, array[18]
#define P_REF_STR                    "p_ref_ts%d"        // actual phase for reference, for each timeslot,  array[18]
#define A_REF_STR                    "a_ref_ts%d"        // actual amplitude for reference, for each timeslot, array[18]
#define P_SET_STR                    "p_set_ts%d"        // phase set value, for each timeslot, array[18]
#define A_SET_STR                    "a_set_ts%d"        // amplitude set value, for each timeslot, array[18]
#define AVG_WINDOW_STR               "avg_window%d"      // average window, length = 4096
#define I_WF_STR                     "i_wf_ch%d"         // i waveform, for each channel, array[10], length = 4096
#define Q_WF_STR                     "q_wf_ch%d"         // q waveform, for each channel, array[10], length = 4096

#define GET_IQ_WF_STR                "get_iq_wf_ch%d"    // get iq waveform per channel


#endif /* _LLRFHLSASYN_H */
