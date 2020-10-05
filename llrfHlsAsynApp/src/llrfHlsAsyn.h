#ifndef _LLRFHLSASYN_H
#define _LLRFHLSASYN_H


#include <asynPortDriver.h>
#include <epicsEvent.h>
#include <epicsTypes.h>
#include <epicsTime.h>

#include <cpsw_api_user.h>
#include <llrfFw.h>
#include <dacSigGenFw.h>
#include <vector>
#include <string>
#include <dlfcn.h>

#include <stdio.h>
#include <sstream>
#include <fstream>

#include "BsaApi.h"


#define NUM_FB_CH        10        // number of feedback channels
#define NUM_WINDOW        3        // number of window
#define NUM_TIMESLOT     18        // number of timeslot
#define MAX_SAMPLES      4096      // number of samples in a waveform


#define MAX_BSABUF      36         // BSA buffer node

#define NUM_HARMONICS          3
#define HARMONICS_DIX_CS(X)    ((X) * NUM_HARMONICS)
#define HARMONICS_IDX_SN(X)    ((X) * NUM_HARMONICS + 1)
#define ALPHA_DIM              (NUM_HARMONICS*2 + 1)
#define ALPHA_HARMO            (ALPHA_DIM -1)
#define ALPHA_IDX_DC           (ALPHA_DIM -1)



typedef struct {
    epicsUInt32     counter;
    epicsTimeStamp  time;
    epicsUInt32     time_slot;
    struct {
        epicsFloat32    phase;
        epicsFloat32    ampl;
    } ap_wch [NUM_WINDOW][NUM_FB_CH];
    epicsFloat32    phase_fb;
    epicsFloat32    ampl_fb;
    epicsFloat32    phase_ref;
    epicsFloat32    ampl_ref;
    epicsFloat32    phase_set;
    epicsFloat32    ampl_set;
    union {
        epicsUInt32     u32_terminator;
        struct {
            uint16_t u16lower;
            uint16_t u16upper;
        } u16u16_terminator;
    } terminator;
    char dummy_buf[128];
} bsa_packet_t;


class llrfHlsAsynDriver
    :asynPortDriver {
    public:
        llrfHlsAsynDriver(void *pDrv, const char *portName, const char *pathString, const char *hlsStream, const char *bsa_prefix,  const char *named_root = NULL);
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
        void getIQWaveform(void);
        void getIQWaveform(int channel);
        void flushIQWaveformsforAllChannels(void);
        void getFirmwareInformation(void);

        void getPhaseJitterforAllTimeslots(void);
        void getAmplJitterforAllTimeslots(void);
        void getBeamPkVoltforAllTimeslots(void);
        void getPhaseJitterforAllChannels(void);
        void getAmplitudeJitterforAllChannels(void);


    private:
        void *pDrv;
        char *port;
        char *path;
        char *stream;
        llrfFw  llrfHls;
        dacSigGenFw dacSigGen;
        Stream hls_stream_;

        bool        need_to_read;
        epicsUInt32 version_;
        epicsUInt32 num_timeslot_;
        epicsUInt32 num_channel_;
        epicsUInt32 num_window_;
        epicsUInt32 max_pulse_len_;
        epicsUInt32 counter_;
        epicsUInt32 drop_counter_;

        epicsUInt32 stream_read_count;
        epicsUInt32 stream_read_size;

          
        bsa_packet_t p_bsa_buf[MAX_BSABUF];
        int          current_bsa;
        uint8_t*     p_buf;

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

        epicsFloat64  convBeamPeakVolt;
        struct {
            epicsUInt32     raw;
            epicsFloat64    val;
        } beam_peak_volt[NUM_TIMESLOT];


        char*      bsa_name;
        BsaChannel BsaChn_pact;
        BsaChannel BsaChn_aact;
        BsaChannel BsaChn_bvolt;
        BsaChannel BsaChn_phase[NUM_WINDOW][NUM_FB_CH];
        BsaChannel BsaChn_amplitude[NUM_WINDOW][NUM_FB_CH];


        void ParameterSetup(void);
        void bsaSetup(void);
        void beamPeakVoltageProcessing(bsa_packet_t *p);
        void bsaProcessing(bsa_packet_t *p);
        void fastPVProcessing(bsa_packet_t *p);


    protected:
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int firstLlrfHlsParam;
#define FIRST_LLRFHLS_PARAM   firstLlrfHlsParam
#endif /* ASYN VERSION CHECK under 4.32 */

        int p_stream_enable;                  // steram enable: 1 , disable: 0
        int p_timeslot_enable;                // timeslot feedback enable:1, disable: 0
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
        int p_iwf_avg_window[NUM_WINDOW];     // complex average window, i waveform
        int p_qwf_avg_window[NUM_WINDOW];     // complex average window, q waveform
        int p_i_wf_ch[NUM_FB_CH];                // i waveform for each channel
        int p_q_wf_ch[NUM_FB_CH];                // q waveform for each channel

        int p_get_iq_wf_ch[NUM_FB_CH];        // get IQWaveform per channel
        int p_get_iq_wf_all;                  // get IQWaveform for all

        struct {
            int p_br_phase[NUM_WINDOW][NUM_FB_CH];
            int p_br_amplitude[NUM_WINDOW][NUM_FB_CH];
            int p_br_pact;
            int p_br_aact;
            int p_br_bvolt;
        } p_br[NUM_TIMESLOT];

        int p_bvolt_conv;
        int p_i_baseband_wf;                       // baseband i waveform
        int p_q_baseband_wf;                       // baseband q waveform

        int p_ampl_coeff[NUM_FB_CH];               // amplitude conversion coefficient, firmware based conversion, per channel
        int p_ampl_norm;                           // normalization factor for amplitude feedback
        int p_var_gain;                            // gain for variance/mean calculation, single pole algorithm in firmware
        int p_rms_phase[NUM_TIMESLOT];             // rms phase, phase jitter
        int p_rms_ampl[NUM_TIMESLOT];              // rms amplitude, amplitude jitter 
        int p_rms_bv[NUM_TIMESLOT];                // rms beam voltage, jitter for beam voltage
        int p_mean_phase[NUM_TIMESLOT];            // mean value for phase
        int p_mean_ampl[NUM_TIMESLOT];             // mean value for amplitude
        int p_mean_bv[NUM_TIMESLOT];               // mean value for beam voltage

        int p_rms_phase_wnd_ch[NUM_WINDOW][NUM_FB_CH];      // rms phase for each channel and each window
        int p_rms_ampl_wnd_ch[NUM_WINDOW][NUM_FB_CH];       // rms amplitude for each channel and each window
        int p_mean_phase_wnd_ch[NUM_WINDOW][NUM_FB_CH];     // mean value of phase for each channel and each window
        int p_mean_ampl_wnd_ch[NUM_WINDOW][NUM_FB_CH];      // mean value of amplitude for each channel and each window


        int p_op_mode;
        int p_p_adaptive_gain;
        int p_a_adaptive_gain;
        int p_p_distb_gain;
        int p_a_distb_gain;
        int p_harmo_cs[NUM_HARMONICS];
        int p_harmo_sn[NUM_HARMONICS];
        int p_p_alpha;
        int p_a_alpha;


#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)      
        int lastLlrfHlsParam;
#define LAST_LLRFHLS_PARAM   lastLlrfHlsParam
#endif /* asyn version check, under 4.32 */

};


#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
#define NUM_LLRFHLS_DET_PARAMS ((int)(&LAST_LLRFHLS_PARAM - &FIRST_LLRFHLS_PARAM-1))
#endif /* asyn version check, under 4.32 */

#define STREAM_ENABLE_STR            "stream_enable"     // stream enable, disable control
#define TIMESLOT_ENABLE_STR          "timeslot_enable"   // timeslot feedback enable, disable control
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
#define IWF_AVG_WINDOW_STR           "iwf_avg_window%d"  // i waveform for complex average window, legnth 4096
#define QWF_AVG_WINDOW_STR           "qwf_avg_window%d"  // q waveform for complex average window, length 4096
#define I_WF_STR                     "i_wf_ch%d"         // i waveform, for each channel, array[10], length = 4096
#define Q_WF_STR                     "q_wf_ch%d"         // q waveform, for each channel, array[10], length = 4096

#define I_BASEBAND_STR               "i_baseband_wf"     // i baseband waveform, length = 4096
#define Q_BASEBAND_STR               "q_baseband_wf"     // q baseband waveform, length = 4096

#define P_BR_WND_CH_STR              "p_br_t%dw%dch%d"      // phase for beam rate PV,     for window and channel
#define A_BR_WND_CH_STR              "a_br_t%dw%dch%d"      // amplitude for beam rate PV, for window and channel
#define P_BSA_WND_CH_STR             "%s:W%dC%d:FAST_PACT"
#define A_BSA_WND_CH_STR             "%s:W%dC%d:FAST_AACT"
#define P_BSA_FB_STR                 "%s:FB:FAST_PACT"
#define A_BSA_FB_STR                 "%s:FB:FAST_AACT"
#define BVOLT_BSA_STR                "%s:BVOLT_PK_FAST"
#define P_BR_STR                     "p_act_br_t%d"              // phase for beam rate PV,     feedback input
#define A_BR_STR                     "a_act_br_t%d"              // amplitude for beam rate PV, feedback input
#define BVOLT_BR_STR                 "bvolt_pk_br_t%d"
#define BVOLT_CONV_STR               "bvolt_conv"

#define GET_IQ_WF_STR                "get_iq_wf_ch%d"    // get iq waveform per channel
#define GET_IQ_WF_ALL_STR            "get_iq_wf_all"     // get iq waveform for all channels

#define AMPL_COEFF_STR               "ampl_coeff_ch%d"   // amplitude conversion coefficient per channel
#define AMPL_NORM_STR                "ampl_norm"         // amplitude normalization factor
#define VAR_GAIN_STR                 "var_gain"          // gain for variance/average calculation
#define PHASE_JITTER_STR             "phase_jitter_ts%d" // phase jitter (RMS) per timeslot
#define AMPL_JITTER_STR              "ampl_jitter_ts%d"  // amplitude jitter (RMS) per timeslot
#define BV_JITTER_STR                "bv_jitter_ts%d"    // beam voltage jitter (RMS) per timeslot
#define PHASE_MEAN_STR               "phase_mean_ts%d"   // mean value for phase per timeslot
#define AMPL_MEAN_STR                "ampl_mean_ts%d"    // mean value for amplitude per timeslot
#define BV_MEAN_STR                  "bv_mean_ts%d"      // mean value for beam peak voltage per timeslot

#define P_JITTER_WND_CH_STR          "p_jitter_w%dch%d"  // phase jitter (RMS) per each channel and each window
#define A_JITTER_WND_CH_STR          "a_jitter_w%dch%d"  // amplitude jitter (RMS) per each channel and each window
#define P_MEAN_WND_CH_STR            "p_mean_w%dch%d"    // mean value of phase per each channel and each window
#define A_MEAN_WND_CH_STR            "a_mean_w%dch%d"    // mean value of amplitude per each channel and each window

#define OP_MODE_STR                  "op_mode"           // enable adaptive mode
#define PHASE_ADAPTIVE_GAIN_STR      "p_adaptive_gain"   // phase gain for adaptive mode
#define AMPL_ADAPTIVE_GAIN_STR       "a_adaptive_gain"   // amplitude gain for adaptive mode
#define PHASE_DISTB_GAIN_STR         "p_distb_gain"      // disturbance gain for phase
#define AMPL_DISTB_GAIN_STR          "a_distp_gain"      // disturbance gain for amplitude
#define HARMO_CS_STR                 "harmo_cs%d"        // CS harmonics 
#define HARMO_SN_STR                 "harmo_sn%d"        // SN harmonics
#define PHASE_ALPHA_STR              "p_alpha"           // alpha values for phase
#define AMPL_ALPHA_STR               "a_alpha"           // alpha values for amplitude


#endif /* _LLRFHLSASYN_H */
