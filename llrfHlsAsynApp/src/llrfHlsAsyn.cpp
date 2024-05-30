#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include <string>
#include <sstream>
#include <fstream>

#include <sys/types.h>
#include <sys/stat.h>

#include <math.h>

#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsExit.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsPrint.h>
#include <ellLib.h>
#include <iocsh.h>

#include <yaml-cpp/yaml.h>
#include <yamlLoader.h>

#include <drvSup.h>
#include <epicsExport.h>
#include <registryFunction.h>

#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>

#include "llrfHlsAsyn.h"
#include "rtmInterface.h"
#include "debugStream.h"

static char *destn_trig_str[] = {"llrf", "rtm", "mod", "ssa"};

static bool          keep_stay_in_loop = true;
static epicsEventId  shutdownEvent;

static ELLLIST *pDrvEllList = NULL;

typedef struct {
    ELLNODE           node;
    char              *named_root;
    char              *port;
    char              *regPath;
    char              *hlsStream;
    char              *bsa_prefix;
    llrfHlsAsynDriver *pLlrfHlsAsyn;
    void              *pRtm;
} pDrvList_t;


static double n_angle(double a)
{

    if(a >= 180.) return (a - 360.);
    if(a < -180.) return (a + 360.);

    return a;

}




static void init_drvList(void)
{
    if(!pDrvEllList) {
        pDrvEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "llrfHlsAsyn driver: init_drvList()");
        ellInit(pDrvEllList);
    }

    return;
}

static pDrvList_t *find_drvByPort(const char *port)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
      if(p->port && strlen(p->port) && !strcmp(p->port, port)) break;
      p = (pDrvList_t *) ellNext(&p->node);
    }

    return p;
}

static pDrvList_t *find_drvByNamedRoot(const char *named_root)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
        if(p->named_root && strlen(p->named_root) && !strcmp(p->named_root, named_root)) break;
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return p;
}


void registerRtm2llrfHlsAsyn(void *pRtm)
{
    init_drvList();
    if(ellCount(pDrvEllList)) {
        pDrvList_t *p = (pDrvList_t *) ellLast(pDrvEllList);
        if(p) p->pRtm = pRtm;
    }

}


llrfHlsAsynDriver::llrfHlsAsynDriver(void *pDrv, const char *portName, const char *pathString, const char *hlsStream, const char *bsa_prefix, const char *named_root)
    : asynPortDriver(portName,
                     1, /* number of elements of this device */
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
                     NUM_LLRFHLS_DET_PARAMS, /* number of asyn params of be cleared for each device */
#endif  /* asyn version check, under 4.32 */
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat64ArrayMask, /* Interface mask */
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynEnumMask    | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat64ArrayMask,  /* Interrupt mask */
                     1, /* asynFlags.  This driver does block and it is not multi-device, so flag is 1 */
                     1, /* Autoconnect */
                     0, /* Default priority */
                     0) /* Default stack size*/
{
    Path p_root;
    Path p_llrfHls;
    port = epicsStrDup(portName);
    path = epicsStrDup(pathString);
    stream = (hlsStream && strlen(hlsStream))?epicsStrDup(hlsStream): NULL;
    this->pDrv = pDrv;

    poll_slowdown     = 0;

    need_to_read      = true;
    stream_read_count = 0;
    stream_read_size  = 0;
    current_bsa       = -1;
    p_buf             = (uint8_t *) &p_bsa_buf[0];
    bsa_name = (bsa_prefix && strlen(bsa_prefix))?epicsStrDup(bsa_prefix):(char *)NULL;

    try {
        p_root = (named_root && strlen(named_root))? cpswGetNamedRoot(named_root): cpswGetRoot();
        p_llrfHls = p_root->findByName(pathString);
        if(stream) hls_stream_ = IStream::create(p_root->findByName(stream));

    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s, file %s, line %d\n", e.getInfo().c_str(), __FILE__, __LINE__);
        throw e;
    }

    llrfHls = IllrfFw::create(p_llrfHls);
    llrfDestn = IllrfDestnTrig::create(p_root->findByName("mmio/AppTop/AppCore"));
    dacSigGen = IdacSigGenFw::create(p_root->findByName("mmio"));


    convBeamPeakVolt = 1.;
    for(int i = 0; i < NUM_TIMESLOT; i++) {
        beam_peak_volt[i].raw = 0.;
        beam_peak_volt[i].val = 0.;

        phase_des_ts[i] = 0.;
        ampl_des_ts[i]  = 0.;
    }

    for(int i = 0; i < NUM_FB_CH; i++) {
        for(int j = 0; j < NUM_DEST; j++) fb_weight_ch[j*NUM_FB_CH + i]  = 0.;
        ampl_coeff_ch[i] = 0.;
    }

    sc_quantization = pow(2., SC_QUANTIZATION);


    ParameterSetup();
    if(bsa_name) bsaSetup();

    getFirmwareInformation();

    callParamCallbacks();

    // Update the baseband readback waveforms
    ReadbacBasebandWaveform();

    // Update average windows readback waveforms
    for (std::size_t w { 0 }; w < NUM_WINDOW; ++w)
        ReadbackIQWaveformAverageWindow(w);
}

llrfHlsAsynDriver::~llrfHlsAsynDriver() {}

asynStatus llrfHlsAsynDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int        function      = pasynUser->reason;
    asynStatus status        = asynSuccess;
    const char *functionName = "writeInt32";

    status = (asynStatus) setIntegerParam(function, value);

    if(function == p_op_mode) {  // adaptive mode enable/disable
        llrfHls->setOpMode(value?true:false);
    } 
    else if(function == p_stream_enable) { // stream enable/disable congtrol
        llrfHls->setStreamEnable(value?true:false);
    }
    else if(function == p_timeslot_enable) {  // timeslot feedback enable/disable control
        llrfHls->setTimeslotFeedback(value?true:false);
    }
    else if(function == p_mode_config) {  // trigger mode contgrol
        llrfHls->setModeConfig((uint32_t) value);
    }
    else if(function == p_ref_subtraction_enable) {  // reference subtraction control
        llrfHls->setReferenceSubtractionEnable(value?true:false);
    }
    else if(function == p_fb_phase_enable) {    // phase loop control
        llrfHls->setPhaseFeedbackEnable(value?true:false);
    }
    else if(function == p_fb_ampl_enable) {    // amplitude loop control
        llrfHls->setAmplFeedbackEnable(value?true:false);
    }
    else if(function == p_get_iq_wf_all & value) {
        getIQWaveform();
    }
    else if(function == p_ampl_norm_od) {    // on-demand command for recalculating amplitude normalization
        recal_norm_flag = value?true:false;  
        if(recal_norm_flag) llrfHls->setRecalNorm(1);    // pass command to frmware
    }
    else
    for(int i = 0; i < NUM_FB_CH; i++) {
        if(function == p_get_iq_wf_ch[i] && value) {
            getIQWaveform(i);
            break;
        }
        if(function == p_permut_idx[i]) {
            llrfHls->setAverageWindowPermutationIndex(value, i);
            break;
        }
    }

    for(int i = 0; i < NUM_DESTNTRIG; i++) {
        if(function == p_destn_trig[i].p_enable) {
            llrfDestn->setEnable(i, value);
            break;
        }
        if(function == p_destn_trig[i].p_polarity) {
            llrfDestn->setPolarity(i, value);
            break;
        }
    }

    return status;
}

asynStatus llrfHlsAsynDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int        function      = pasynUser->reason;
    asynStatus status        = asynSuccess;
    const char *functionName = "writeFloat64";

    status = (asynStatus) setDoubleParam(function, value);

    if(function == p_p_ref_offset) {  // phase offset for reference
        llrfHls->setPhaseReferenceOffset(value);
    }
    else if(function == p_p_fb_offset) {  // phase offset for feedback
        llrfHls->setPhaseFeedbackOffset(value);
    }
    else if(function == p_p_gain) { // gain for phase loop
        llrfHls->setPhaseGain(value);
    }
    else if(function == p_a_gain) { // gain for amplitude loop
        llrfHls->setAmplGain(value);
    }
    else if(function == p_p_corr_upper) {  // phase correction upper limit
        llrfHls->setPhaseCorrectionUpperLimit(value);
    }
    else if(function == p_p_corr_lower) {  // phase correction lower limit
        llrfHls->setPhaseCorrectionLowerLimit(value);
    }
    else if(function == p_a_corr_upper) {  // amplitude correction upper limit
        llrfHls->setAmplCorrectionUpperLimit(value);
    }
    else if(function == p_a_corr_lower) {  // amplitude correction lower limit
        llrfHls->setAmplCorrectionLowerLimit(value);
    }
    else if(function == p_a_drv_upper) {  // amplitude correction upper limit
        llrfHls->setAmplDriveUpperLimit(value);
    }
    else if(function == p_a_drv_lower) {  // amplitude corrrection lower limit
        llrfHls->setAmplDriveLowerLimit(value);
    }
    else if(function == p_a_threshold) { // amplitude threshold
        llrfHls->setAmplThreshold(value);
    }
    else if(function == p_bvolt_conv) {  // beam voltage conversion factor 
        convBeamPeakVolt = value;
    }
    else if(function == p_var_gain) {   // gain for variance / mean calculation, single pole algorithm
        llrfHls->setVarGain(value);
    }
    else if(function == p_var_gain_nt) {  // gain for ariance / mean calculation for non-timeslot aware variables
        llrfHls->setVarNtGain(value);
    }
    else if(function == p_p_adaptive_gain) {    // phase gain for adaptive feedback
        llrfHls->setPhaseAdaptiveGain(value);
        double g;
        getDoubleParam(p_p_distb_gain, &g);
        llrfHls->setPhaseDistbGain(value * g);
    }
    else if(function == p_a_adaptive_gain) {    // amplitude gain for adaptive feedback
        llrfHls->setAmplAdaptiveGain(value);
        double g;
        getDoubleParam(p_a_distb_gain, &g);
        llrfHls->setAmplDistbGain(value * g);
    }
    else if(function == p_p_distb_gain) {       // disturbance gain for phase feedback
        double g;
        getDoubleParam(p_p_adaptive_gain, &g);
        llrfHls->setPhaseDistbGain(value * g);
    }
    else if(function == p_a_distb_gain) {    // disturbance gain for amplitude feedback
        double g;
        getDoubleParam(p_a_adaptive_gain, &g);
        llrfHls->setAmplDistbGain(value * g);
    }

    for(int i = 0; i < NUM_DEST; i++) {
        if(function == p_ampl_norm[i]) {  // normalization factor for amplitude
            if(value != 0.) llrfHls->setAmplNorm(1. / value, i);   // pass the receiprocal of the factor to the firmware
            llrfHls->setRecalNorm(0);   // clear_recal flag for next request
            break;
        }
    }


    for(int i = 0; i < NUM_FB_CH; i++) {   // search for channel number
        if(function == p_ref_weight_ch[i]) {    // channel weight for reference
            llrfHls->setReferenceChannelWeight(value, i);
            break;
        }
        else if(function == p_p_offset_ch[i]) {    // phase offset for channel
            llrfHls->setPhaseOffset(value, i);
            break;
        }
        else if(function == p_ampl_coeff[i]) { // amplitude conversion coefficientfor channel
            ampl_coeff_ch[i] = value;
            llrfHls->setAmplCoeff(value, i);
            recalc_dequantization();
        }
        else if(function == p_power_coeff[i]) { // power conversion coefficientfor channel
            power_coeff_ch[i] = value;
        }

        for(int j = 0; j < NUM_DEST; j++) {
            if(function == p_fb_weight_ch[j*NUM_FB_CH + i]) {  // channel weight for feedback
                fb_weight_ch[j*NUM_FB_CH + i] = value;
                llrfHls->setFeedbackChannelWeight(value, i, j);    // (value, channel, destination index)
                recalc_dequantization();
                break;
            }
        }
    }

    for(int i = 0; i < NUM_TIMESLOT; i++) {    // search for timeslot
         if(function == p_p_des_ts[i]) {  // desired phase for a specific timeslot
             phase_des_ts[i] = value;
             llrfHls->setDesiredPhase(value, i);
             break;
         }
         else if(function == p_a_des_ts[i]) {  // desired amplitude for a specific timeslot
             ampl_des_ts[i] = value;
             llrfHls->setDesiredAmpl(value, i);
             break;
         }
    }
    
    for(int i = 0; i < NUM_DESTNTRIG; i++) {
        if(function == p_destn_trig[i].p_delay) {
            uint32_t tick = value * 1.E-3 * 119. + 0.5;   // using hard-coded clock (LCLS1) for initial implement
            llrfDestn->setDelay(i, tick);
            break;
        }

        if(function == p_destn_trig[i].p_width) {
            uint32_t tick = value * 1.E-3 * 119. +0.5;    // using hard-coded clock (LCLS1) for initial implement
            llrfDestn->setWidth(i, tick);
            break;
        }
    }

    return status;
}

static epicsFloat64* __zero_pad(epicsFloat64 *value, epicsFloat64 *v, size_t nElements, int len)
{
    if(nElements >= MAX_SAMPLES) return value;

    int i;
    for(i = 0; i < nElements;   i++)  *(v + i) = *(value + i);
    for(     ; i < len; i++)  *(v + i) = 0.;

    return v;
}

asynStatus llrfHlsAsynDriver::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn)
{
    int        function      { pasynUser->reason };
    asynStatus status        { asynSuccess };
    const char *functionName = "readFloat64Array";
    bool       functionFound { false };

    for(std::size_t w { 0 }; w < NUM_WINDOW; ++w)
    {
        if(function == p_iwf_avg_window_rbv[w])
        {
            memcpy(value, iwf_avg_window[w], nElements*sizeof(epicsFloat64));
            *nIn = nElements;
            functionFound = true;
        }
        else if(function == p_qwf_avg_window_rbv[w])
        {
            memcpy(value, qwf_avg_window[w], nElements*sizeof(epicsFloat64));
            *nIn = nElements;
            functionFound = true;
        }
        else if(function == p_awf_avg_window_rbv[w])
        {
            memcpy(value, awf_avg_window[w], nElements*sizeof(epicsFloat64));
            *nIn = nElements;
            functionFound = true;
        }
        else if(function == p_pwf_avg_window_rbv[w])
        {
            memcpy(value, pwf_avg_window[w], nElements*sizeof(epicsFloat64));
            *nIn = nElements;
            functionFound = true;
        }
        else if(function == p_i_baseband_wf_rbv)
        {
            memcpy(value, i_baseband_wf, nElements*sizeof(epicsFloat64));
            *nIn = nElements;
            functionFound = true;
        }
        else if(function == p_q_baseband_wf_rbv)
        {
            memcpy(value, q_baseband_wf, nElements*sizeof(epicsFloat64));
            *nIn = nElements;
            functionFound = true;
        }
        else if(function == p_a_baseband_wf_rbv)
        {
            memcpy(value, a_baseband_wf, nElements*sizeof(epicsFloat64));
            *nIn = nElements;
            functionFound = true;
        }
        else if(function == p_p_baseband_wf_rbv)
        {
            memcpy(value, p_baseband_wf, nElements*sizeof(epicsFloat64));
            *nIn = nElements;
            functionFound = true;
        }
    }

    if(!functionFound)
        status = asynPortDriver::readFloat64Array(pasynUser, value, nElements, nIn);

    return status;
}

asynStatus llrfHlsAsynDriver::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements)
{
    int        function      = pasynUser->reason;
    asynStatus status        = asynSuccess;
    const char *functionName = "writeFloat64Array";

    epicsFloat64 v[MAX_SAMPLES];


    if(function == p_i_baseband_wf) {
        memcpy(i_baseband_wf, __zero_pad(value, v, nElements, MAX_SAMPLES), MAX_SAMPLES*sizeof(epicsFloat64));
        UpdatePABasebandWaveform();
        // printf("baseband i waveform size: %d\n", nElements);
    }
    else if(function == p_q_baseband_wf) {
        memcpy(q_baseband_wf, __zero_pad(value, v, nElements, MAX_SAMPLES), MAX_SAMPLES*sizeof(epicsFloat64));
        UpdatePABasebandWaveform();
        // printf("baseband q waveform size: %d\n", nElements);
    }
    if(function == p_a_baseband_wf) {
        memcpy(a_baseband_wf, __zero_pad(value, v, nElements, MAX_SAMPLES), MAX_SAMPLES*sizeof(epicsFloat64));
        UpdateIQBasebandWaveform();
        // printf("baseband ampl waveform size: %d\n", nElements);
    }
    if(function == p_p_baseband_wf) {
        memcpy(p_baseband_wf, __zero_pad(value, v, nElements, MAX_SAMPLES), MAX_SAMPLES*sizeof(epicsFloat64));
        UpdateIQBasebandWaveform();
        // printf("baseband phase waveform size: %d\n", nElements);
    }
    else

    for(int w = 0; w < NUM_WINDOW; w++) {
        if(function == p_avg_window[w]) {    // update average window
            llrfHls->setAverageWindow(__zero_pad(value, v, nElements, MAX_SAMPLES), w);
            // printf("avg window %d size:%d\n", w, nElements);
            break;
        }
        else if(function == p_iwf_avg_window[w]) {  // update complex waveform I
            memcpy(iwf_avg_window[w], __zero_pad(value, v, nElements, MAX_SAMPLES), MAX_SAMPLES*sizeof(epicsFloat64));
            UpdatePAWaveformAverageWindow(w);
            // printf("i window %d size: %d\n", w, nElements);
            break;
        }
        else if(function == p_qwf_avg_window[w]) { // update complex wave098b6587f0ffd1480b58e187b770a4974e560d63form Q
            memcpy(qwf_avg_window[w], __zero_pad(value, v, nElements, MAX_SAMPLES), MAX_SAMPLES*sizeof(epicsFloat64));
            UpdatePAWaveformAverageWindow(w);
            // printf("q window %d size: %d\n", w, nElements);
            break;
        }
        else if(function == p_awf_avg_window[w]) {  // update complex waveform I
            memcpy(awf_avg_window[w], __zero_pad(value, v, nElements, MAX_SAMPLES), MAX_SAMPLES*sizeof(epicsFloat64));
            UpdateIQWaveformAverageWindow(w);
            // printf("ampl window %d size: %d\n", w, nElements);
            break;
        }
        else if(function == p_pwf_avg_window[w]) {  // update complex waveform I
            memcpy(&pwf_avg_window[w][0], value, MAX_SAMPLES*sizeof(epicsFloat64)); //__zero_pad(value, v, nElements, MAX_SAMPLES), MAX_SAMPLES);
            UpdateIQWaveformAverageWindow(w);
            // printf("phase window %d size: %d\n", w, nElements);
            break;
        }
    }

    for(int i = 0; i < NUM_HARMONICS; i++) {
        if(function == p_harmo_cs[i]) {
            llrfHls->setHarmonicsCs(__zero_pad(value, v, nElements, ALPHA_DIM), i + 1);
            break;
        }
        else if(function == p_harmo_sn[i]) {
            llrfHls->setHarmonicsSn(__zero_pad(value, v, nElements, ALPHA_DIM), i + 1);
            break;
        }
    }

    return status;

}

void llrfHlsAsynDriver::report(int interest)
{
    if(!interest) return;

    printf("\tbuffer address     : %p\n", p_buf);
    printf("\tfirmware viersion  : %u\n", version_);
    printf("\tnumber of timeslots: %u\n", num_timeslot_);
    printf("\tnumber of channels : %u\n", num_channel_);
    printf("\tnumber of windows  : %u\n", num_window_);
    printf("\tmax pulse length   : %u\n", max_pulse_len_);
    printf("\tcounter            : %u\n", counter_);
    printf("\tdrop counter       : %u\n", drop_counter_);
    printf("\tstream read count  : %u\n", stream_read_count);
    printf("\tstream read size   : %u\n", stream_read_size);

    if(interest < 5) return;

    printf("\n\tStream data dump (16bit)");
    unsigned short *u = (unsigned short *)p_buf;
    for(int i = 0; i < stream_read_size / sizeof(unsigned short); i++) {
        
        if(!(i % 8)) printf("\n\t%4.4x: ", (unsigned short) (i*sizeof(unsigned short)));
        printf("%4.4x ", *(u +i));

    }

    printf("\n\tStream data dump (32bit)");
    unsigned *u1 = (unsigned *)p_buf;
    for(int i = 0; i < stream_read_size / sizeof(unsigned); i++) {
        
        if(!(i % 4)) printf("\n\t%4.4x: ", (unsigned short) (i*sizeof(unsigned)));
        printf("%8.8x ", *(u1 +i));

    }
    printf("\n");
    bsa_packet_t *p = (bsa_packet_t *) p_buf;
    printf("\tbeam peak voltage: %4.4x\n", p->terminator.u16u16_terminator.u16upper);

    printf("\n");
    char ts_str[80];
    epicsTimeStamp ts = *(epicsTimeStamp *) (u1+1);
    epicsTimeToStrftime(ts_str, sizeof(ts_str), "%Y/%m/%d %H:%M:%S.%09f", &ts);
    printf("\ttimestamp in stream : %s\n", ts_str);

    llrfHls->printFeedbackChannelWeight();
}

void llrfHlsAsynDriver::fast_poll(void)
{
    getIQWaveform();
//    callParamCallbacks();
}

void llrfHlsAsynDriver::poll(void)
{

    fast_poll();

    if(++poll_slowdown >= POLL_RATE) poll_slowdown = 0;
    if(poll_slowdown % POLL_RATE) return;

    llrfHls->getPhaseAllChannels((epicsFloat64 *) phase_wnd_ch);
    llrfHls->getAmplAllChannels((epicsFloat64 *)  ampl_wnd_ch);
    llrfHls->getFeedbackPhaseAllTimeslots(fb_phase_ts);
    llrfHls->getFeedbackAmplAllTimeslots(fb_ampl_ts);
    llrfHls->getReferencePhaseAllTimeslots(ref_phase_ts);
    llrfHls->getReferenceAmplAllTimeslots(ref_ampl_ts);
    llrfHls->getPhaseSetAllTimeslots(phase_set_ts);
    llrfHls->getAmplSetAllTimeslots(ampl_set_ts);
    llrfHls->getCounter(&counter_);                setIntegerParam(p_counter,      counter_);
    llrfHls->getDropCounter(&drop_counter_);       setIntegerParam(p_drop_counter, drop_counter_);

    llrfHls->getPhaseFeedbackStatus(&p_feedback_st_);  setIntegerParam(p_p_feedback_st, p_feedback_st_);
    llrfHls->getAmplFeedbackStatus(&a_feedback_st_);   setIntegerParam(p_a_feedback_st, a_feedback_st_);

    // Do power calculations
    for(int i {0}; i < NUM_WINDOW; ++i)
    {
        for(int j {0}; j < NUM_FB_CH; ++j)
        {
            epicsFloat64 ampl_scale  { ampl_coeff_ch[j]   }; // amplitude coefficient
            epicsFloat64 power_scale { power_coeff_ch[j] }; // power coefficient
            epicsFloat64 scaled_ampl { ampl_wnd_ch[i][j] }; // scaled amplitude

            // scaled_power  = power_scale * (normalized_amplitude)^2
            //               = power_scale * (scaled_amplitude / amplitude_scale)^2
            // We don't check for (ampl_scale == 0), as in that case the PV value will be set to "nan", and
            // its status to "INVALID" automatically.
            power_wnd_ch[i][j] = power_scale * (scaled_ampl * scaled_ampl) / (ampl_scale * ampl_scale);
        }
    }

    {
        double phase_alpha[ALPHA_DIM], ampl_alpha[ALPHA_DIM];

        llrfHls->getPhaseAlpha(phase_alpha);
        llrfHls->getAmplAlpha(ampl_alpha);
        doCallbacksFloat64Array(phase_alpha, ALPHA_DIM, p_p_alpha, 0);
        doCallbacksFloat64Array(ampl_alpha,  ALPHA_DIM, p_a_alpha, 0);
    }

    /*
    for(int i = 0; i < NUM_FB_CH; i++) {
        llrfHls->getIWaveform(i_wf_ch[i], i);
        llrfHls->getQWaveform(q_wf_ch[i], i);
    }
    */

    getPhaseJitterforAllTimeslots();
    getAmplJitterforAllTimeslots();
    getBeamPkVoltforAllTimeslots();
    getPhaseJitterforAllChannels();
    getAmplitudeJitterforAllChannels();

    checkAmplNorm();

    updatePVs();
    callParamCallbacks();
}

void llrfHlsAsynDriver::pollStream(void)
{
    while(stream) {
        if(need_to_read) getFirmwareInformation();

        current_bsa ++;
        current_bsa      = (current_bsa < MAX_BSABUF)? current_bsa: 0;
        p_buf            = (uint8_t *) &p_bsa_buf[current_bsa];
        stream_read_size = hls_stream_->read(p_buf, 4096, CTimeout());
        stream_read_count++;

        if(((pDrvList_t *) pDrv)->pRtm) callRtmProcessing((p_bsa_buf[current_bsa]).time, (p_bsa_buf[current_bsa]).time_slot, ((pDrvList_t *)pDrv)->pRtm);  // asynchronous rtm processing

        setTimeStamp(&((p_bsa_buf[current_bsa]).time));
        beamPeakVoltageProcessing(&p_bsa_buf[current_bsa]);
        if(bsa_name) bsaProcessing(&p_bsa_buf[current_bsa]);
        fastPVProcessing(&p_bsa_buf[current_bsa]);

    }

}

void llrfHlsAsynDriver::updatePVs(void)
{
    updatePhasePVsforAllChannels();
    updateAmplPVsforAllChannels();
    updateFeedbackPhasePVsforAllTimeslots();
    updateFeedbackAmplPVsforAllTimeslots();
    updateReferencePhasePVsforAllTimeslots();
    updateReferenceAmplPVsforAllTimeslots();
    updatePhaseSetPVsforAllTimeslots();
    updateAmplSetPVsforAllTimeslots();

    // flushIQWaveformsforAllChannels();

}


void llrfHlsAsynDriver::getIQWaveform(void)
{
    int channel;

    llrfHls->freezeWaveform(true);

    for(channel = 0; channel < NUM_FB_CH; channel++) {
        llrfHls->getIQWaveform(i_wf_ch[channel], q_wf_ch[channel], channel);

        iq2pa(i_wf_ch[channel], q_wf_ch[channel], p_wf_ch[channel], a_wf_ch[channel], ampl_coeff_ch[channel]);
    }

    llrfHls->freezeWaveform(false);

    flushIQWaveformsforAllChannels();
}


void llrfHlsAsynDriver::getIQWaveform(int channel)
{
    llrfHls->freezeWaveform(true);
    llrfHls->getIQWaveform(i_wf_ch[channel], q_wf_ch[channel], channel);
    llrfHls->freezeWaveform(false);

    for(int k = 0; k < MAX_SAMPLES; k++) {
        double i = i_wf_ch[channel][k];
        double q = q_wf_ch[channel][k];

        p_wf_ch[channel][k] = (i == 0.)? NAN: (atan2(q, i) * 180./M_PI);
        a_wf_ch[channel][k] = sqrt(i*i + q*q) * ampl_coeff_ch[channel];
    }


    doCallbacksFloat64Array(i_wf_ch[channel], MAX_SAMPLES, p_i_wf_ch[channel], 0);
    doCallbacksFloat64Array(q_wf_ch[channel], MAX_SAMPLES, p_q_wf_ch[channel], 0);

    doCallbacksFloat64Array(p_wf_ch[channel], MAX_SAMPLES, p_p_wf_ch[channel], 0);
    doCallbacksFloat64Array(a_wf_ch[channel], MAX_SAMPLES, p_a_wf_ch[channel], 0);
}

void llrfHlsAsynDriver::updatePhasePVsforAllChannels(void)
{
    for(int w = 0; w < NUM_WINDOW; w++) for(int i = 0; i < NUM_FB_CH; i++) {
        setDoubleParam(p_p_wnd_ch[w][i], n_angle(phase_wnd_ch[w][i]));
    }
}

void llrfHlsAsynDriver::updateAmplPVsforAllChannels(void)
{
    for(int w = 0; w < NUM_WINDOW; w++) for(int i = 0; i < NUM_FB_CH; i++) {
        setDoubleParam(p_a_wnd_ch[w][i], ampl_wnd_ch[w][i]);
        setDoubleParam(p_pw_wnd_ch[w][i], power_wnd_ch[w][i]);
    }
}

void llrfHlsAsynDriver::updateFeedbackPhasePVsforAllTimeslots(void)
{
    for(int i = 0; i < NUM_TIMESLOT; i++) {
        setDoubleParam(p_p_fb_ts[i], n_angle(fb_phase_ts[i]));
    }
}


void llrfHlsAsynDriver::updateFeedbackAmplPVsforAllTimeslots(void)
{
    for(int i = 0; i < NUM_TIMESLOT; i++) {
        setDoubleParam(p_a_fb_ts[i], fb_ampl_ts[i]);
    }
}


void llrfHlsAsynDriver::updateReferencePhasePVsforAllTimeslots(void)
{
    for(int i = 0; i < NUM_TIMESLOT; i++) {
        setDoubleParam(p_p_ref_ts[i], n_angle(ref_phase_ts[i]));
    }
}

void llrfHlsAsynDriver::updateReferenceAmplPVsforAllTimeslots(void)
{
    for(int i = 0; i < NUM_TIMESLOT; i++) {
        setDoubleParam(p_a_ref_ts[i], ref_ampl_ts[i]);
    }
}

void llrfHlsAsynDriver::updatePhaseSetPVsforAllTimeslots(void)
{
    for(int i = 0; i< NUM_TIMESLOT; i++) {
        setDoubleParam(p_p_set_ts[i], n_angle(phase_set_ts[i]));
    }
}

void llrfHlsAsynDriver::updateAmplSetPVsforAllTimeslots(void)
{
    for(int i = 0; i < NUM_TIMESLOT; i++) {
        setDoubleParam(p_a_set_ts[i], ampl_set_ts[i]);
    }
}


void llrfHlsAsynDriver::flushIQWaveformsforAllChannels(void)
{
    for(int i = 0; i < NUM_FB_CH; i++) {
        doCallbacksFloat64Array(i_wf_ch[i], MAX_SAMPLES, p_i_wf_ch[i], 0);
        doCallbacksFloat64Array(q_wf_ch[i], MAX_SAMPLES, p_q_wf_ch[i], 0);
        doCallbacksFloat64Array(p_wf_ch[i], MAX_SAMPLES, p_p_wf_ch[i], 0);
        doCallbacksFloat64Array(a_wf_ch[i], MAX_SAMPLES, p_a_wf_ch[i], 0);
    }

}

void llrfHlsAsynDriver::getFirmwareInformation(void)
{
    llrfHls->getVersion(&version_);                   setIntegerParam(p_version,       version_);
    llrfHls->getNumTimeslot(&num_timeslot_);          setIntegerParam(p_num_timeslot,  num_timeslot_);
    llrfHls->getNumChannel(&num_channel_);            setIntegerParam(p_num_channel,   num_channel_);
    llrfHls->getNumWindow(&num_window_);              setIntegerParam(p_num_window,    num_window_);
    llrfHls->getMaxPulseLength(&max_pulse_len_);      setIntegerParam(p_max_pulse_len, max_pulse_len_);

    if(version_ || num_timeslot_ || num_channel_ || num_window_ || max_pulse_len_) need_to_read = false;
}

void llrfHlsAsynDriver::getPhaseJitterforAllTimeslots(void)
{
    double var[NUM_STATISTICS], mean[NUM_STATISTICS];

    llrfHls->getVarPhaseAllTimeslots(var);
    llrfHls->getAvgPhaseAllTimeslots(mean);

    // for timeslot aware
    for(int i = 0; i < NUM_TIMESLOT; i++) {
        setDoubleParam(p_rms_phase[i], sqrt(var[i]) * 180./M_PI);
        setDoubleParam(p_mean_phase[i], n_angle(mean[i] * 180./M_PI));
    }
    // for non-timeslot aware
    setDoubleParam(p_rms_phase_nt, sqrt(var[NT_STATISTICS]) * 180./M_PI);
    setDoubleParam(p_mean_phase_nt, n_angle(mean[NT_STATISTICS] * 180./M_PI));
}

void llrfHlsAsynDriver::getAmplJitterforAllTimeslots(void)
{
    double var[NUM_STATISTICS], mean[NUM_STATISTICS];

    llrfHls->getVarAmplAllTimeslots(var);
    llrfHls->getAvgAmplAllTimeslots(mean);

    // for timeslot aware
    for(int i = 0; i< NUM_TIMESLOT; i++) {
        setDoubleParam(p_rms_ampl[i], sqrt(var[i]));
        setDoubleParam(p_mean_ampl[i], mean[i]);
    }
    // for non-timeslot aware
    setDoubleParam(p_rms_ampl_nt, sqrt(var[NT_STATISTICS]));
    setDoubleParam(p_mean_ampl_nt, mean[NT_STATISTICS]);
}

void llrfHlsAsynDriver::getPhaseJitterforAllChannels(void)
{
    double var[NUM_WINDOW][NUM_FB_CH];
    double mean[NUM_WINDOW][NUM_FB_CH];

    llrfHls->getVarPhaseAllChannels((double*) var);
    llrfHls->getAvgPhaseAllChannels((double*) mean);

    for(int w = 0; w < NUM_WINDOW; w++) {
        for(int c = 0; c < NUM_FB_CH; c++) {
            setDoubleParam(p_rms_phase_wnd_ch[w][c], sqrt(var[w][c]) * 180./M_PI);
            setDoubleParam(p_mean_phase_wnd_ch[w][c], n_angle(mean[w][c] * 180./M_PI));
        }
    }
}

void llrfHlsAsynDriver::getAmplitudeJitterforAllChannels(void)
{
    double var[NUM_WINDOW][NUM_FB_CH];
    double mean[NUM_WINDOW][NUM_FB_CH];

    llrfHls->getVarAmplAllChannels((double*) var);
    llrfHls->getAvgAmplAllChannels((double*) mean);

    for(int w = 0; w < NUM_WINDOW; w++) {
        for(int c = 0; c < NUM_FB_CH; c++) {
            setDoubleParam(p_rms_ampl_wnd_ch[w][c], sqrt(var[w][c]));
            setDoubleParam(p_mean_ampl_wnd_ch[w][c], mean[w][c]);
        }
    }

}

void llrfHlsAsynDriver::getBeamPkVoltforAllTimeslots(void)
{
    double var[NUM_STATISTICS], mean[NUM_STATISTICS];

    llrfHls->getVarBeamVoltageAllTimeslots(var);
    llrfHls->getAvgBeamVoltageAllTimeslots(mean);

    // for timeslot aware
    for(int i = 0; i< NUM_TIMESLOT; i++) {
        setDoubleParam(p_rms_bv[i], sqrt(var[i]) * 32768. * convBeamPeakVolt);
        setDoubleParam(p_mean_bv[i], mean[i] * 32768. * convBeamPeakVolt);
    }
    // for non-timeslot aware
    setDoubleParam(p_rms_bv_nt, sqrt(var[NT_STATISTICS]) * 32768. * convBeamPeakVolt);
    setDoubleParam(p_mean_bv_nt, mean[NT_STATISTICS] * 32768. * convBeamPeakVolt);
}


void llrfHlsAsynDriver::ParameterSetup(void)
{
    char param_name[80];

    for(int i=0; i < NUM_DESTNTRIG; i++) {
        sprintf(param_name, DESTNTRIG_ENABLE_STR,   destn_trig_str[i]); createParam(param_name, asynParamInt32,   &(p_destn_trig[i].p_enable));
        sprintf(param_name, DESTNTRIG_POLARITY_STR, destn_trig_str[i]); createParam(param_name, asynParamInt32,   &(p_destn_trig[i].p_polarity));
        sprintf(param_name, DESTNTRIG_DELAY_STR,    destn_trig_str[i]); createParam(param_name, asynParamFloat64, &(p_destn_trig[i].p_delay));
        sprintf(param_name, DESTNTRIG_WIDTH_STR,    destn_trig_str[i]); createParam(param_name, asynParamFloat64, &(p_destn_trig[i].p_width));
    }

    sprintf(param_name, STREAM_ENABLE_STR);          createParam(param_name, asynParamInt32,   &p_stream_enable);
    sprintf(param_name, TIMESLOT_ENABLE_STR);        createParam(param_name, asynParamInt32,   &p_timeslot_enable);
    sprintf(param_name, MODE_CONFIG_STR);            createParam(param_name, asynParamInt32,   &p_mode_config);
    
    sprintf(param_name, VERSION_STR);                createParam(param_name, asynParamInt32,   &p_version);
    sprintf(param_name, NUM_TIMESLOT_STR);           createParam(param_name, asynParamInt32,   &p_num_timeslot);
    sprintf(param_name, NUM_CHANNEL_STR);            createParam(param_name, asynParamInt32,   &p_num_channel);
    sprintf(param_name, NUM_WINDOW_STR);             createParam(param_name, asynParamInt32,   &p_num_window);
    sprintf(param_name, MAX_PULSE_LEN_STR);          createParam(param_name, asynParamInt32,   &p_max_pulse_len);
    sprintf(param_name, COUNTER_STR);                createParam(param_name, asynParamInt32,   &p_counter);
    sprintf(param_name, DROP_COUNTER_STR);           createParam(param_name, asynParamInt32,   &p_drop_counter);
    sprintf(param_name, P_FEEDBACK_ST_STR);          createParam(param_name, asynParamInt32,   &p_p_feedback_st);
    sprintf(param_name, A_FEEDBACK_ST_STR);          createParam(param_name, asynParamInt32,   &p_a_feedback_st);

    sprintf(param_name, P_REF_OFFSET_STR);           createParam(param_name, asynParamFloat64, &p_p_ref_offset);
    sprintf(param_name, P_FB_OFFSET_STR);            createParam(param_name, asynParamFloat64, &p_p_fb_offset);
    sprintf(param_name, P_GAIN_STR);                 createParam(param_name, asynParamFloat64, &p_p_gain);
    sprintf(param_name, A_GAIN_STR);                 createParam(param_name, asynParamFloat64, &p_a_gain);
    sprintf(param_name, REF_SUBTRACTION_ENABLE_STR); createParam(param_name, asynParamInt32,   &p_ref_subtraction_enable);
    sprintf(param_name, FB_PHASE_ENABLE_STR);        createParam(param_name, asynParamInt32,   &p_fb_phase_enable);
    sprintf(param_name, FB_AMPL_ENABLE_STR);         createParam(param_name, asynParamInt32,   &p_fb_ampl_enable);
    sprintf(param_name, P_CORR_UPPER_STR);           createParam(param_name, asynParamFloat64, &p_p_corr_upper);
    sprintf(param_name, P_CORR_LOWER_STR);           createParam(param_name, asynParamFloat64, &p_p_corr_lower);
    sprintf(param_name, A_CORR_UPPER_STR);           createParam(param_name, asynParamFloat64, &p_a_corr_upper);
    sprintf(param_name, A_CORR_LOWER_STR);           createParam(param_name, asynParamFloat64, &p_a_corr_lower);
    sprintf(param_name, A_DRV_UPPER_STR);            createParam(param_name, asynParamFloat64, &p_a_drv_upper);
    sprintf(param_name, A_DRV_LOWER_STR);            createParam(param_name, asynParamFloat64, &p_a_drv_lower);
    sprintf(param_name, A_THRED_STR);                createParam(param_name, asynParamFloat64, &p_a_threshold);

    for(int i = 0; i < NUM_DEST; i++) {
        sprintf(param_name, AMPL_NORM_STR, i);              createParam(param_name, asynParamFloat64, &p_ampl_norm[i]);
        sprintf(param_name, AMPL_NORM_PB_STR, i);           createParam(param_name, asynParamFloat64, &p_ampl_norm_pb[i]);
    }
    sprintf(param_name, AMPL_NORM_OD_STR);           createParam(param_name, asynParamInt32,   &p_ampl_norm_od);
    sprintf(param_name, VAR_GAIN_STR);               createParam(param_name, asynParamFloat64, &p_var_gain);
    sprintf(param_name, VAR_GAIN_NT_STR);            createParam(param_name, asynParamFloat64, &p_var_gain_nt);

    for(int w = 0; w < NUM_WINDOW; w++) {
        sprintf(param_name, AVG_WINDOW_STR, w);     createParam(param_name, asynParamFloat64Array, &(p_avg_window[w]));
        sprintf(param_name, IWF_AVG_WINDOW_STR, w); createParam(param_name, asynParamFloat64Array, &(p_iwf_avg_window[w]));
        sprintf(param_name, QWF_AVG_WINDOW_STR, w); createParam(param_name, asynParamFloat64Array, &(p_qwf_avg_window[w]));
        sprintf(param_name, AWF_AVG_WINDOW_STR, w); createParam(param_name, asynParamFloat64Array, &(p_awf_avg_window[w]));
        sprintf(param_name, PWF_AVG_WINDOW_STR, w); createParam(param_name, asynParamFloat64Array, &(p_pwf_avg_window[w]));
        sprintf(param_name, IWF_AVG_WINDOW_RBV_STR, w); createParam(param_name, asynParamFloat64Array, &(p_iwf_avg_window_rbv[w]));
        sprintf(param_name, QWF_AVG_WINDOW_RBV_STR, w); createParam(param_name, asynParamFloat64Array, &(p_qwf_avg_window_rbv[w]));
        sprintf(param_name, AWF_AVG_WINDOW_RBV_STR, w); createParam(param_name, asynParamFloat64Array, &(p_awf_avg_window_rbv[w]));
        sprintf(param_name, PWF_AVG_WINDOW_RBV_STR, w); createParam(param_name, asynParamFloat64Array, &(p_pwf_avg_window_rbv[w]));
    }

    for(int i = 0; i < NUM_FB_CH; i++) {    // for loop for number of channels
        sprintf(param_name, REF_WEIGHT_STR, i); createParam(param_name, asynParamFloat64, &(p_ref_weight_ch[i]));
        for(int j=0; j < NUM_DEST; j++) {
            sprintf(param_name, FB_WEIGHT_STR, j, i); createParam(param_name, asynParamFloat64, &(p_fb_weight_ch[j*NUM_FB_CH +i]));
        }
        sprintf(param_name, P_OFFSET_STR,   i); createParam(param_name, asynParamFloat64, &(p_p_offset_ch[i]));
        sprintf(param_name, PERMUT_IDX_STR, i); createParam(param_name, asynParamInt32,   &(p_permut_idx[i]));

        for(int w = 0; w < NUM_WINDOW; w++) {
            sprintf(param_name, P_WND_CH_STR, w, i);  createParam(param_name, asynParamFloat64, &(p_p_wnd_ch[w][i]));
            sprintf(param_name, A_WND_CH_STR, w, i);  createParam(param_name, asynParamFloat64, &(p_a_wnd_ch[w][i]));
            sprintf(param_name, PW_WND_CH_STR, w, i); createParam(param_name, asynParamFloat64, &(p_pw_wnd_ch[w][i]));

            sprintf(param_name, P_JITTER_WND_CH_STR, w, i); createParam(param_name, asynParamFloat64, &(p_rms_phase_wnd_ch[w][i]));
            sprintf(param_name, A_JITTER_WND_CH_STR, w, i); createParam(param_name, asynParamFloat64, &(p_rms_ampl_wnd_ch[w][i]));
            sprintf(param_name, P_MEAN_WND_CH_STR, w, i);   createParam(param_name, asynParamFloat64, &(p_mean_phase_wnd_ch[w][i]));
            sprintf(param_name, A_MEAN_WND_CH_STR, w, i);   createParam(param_name, asynParamFloat64, &(p_mean_ampl_wnd_ch[w][i]));
        }

        sprintf(param_name, I_WF_STR,       i); createParam(param_name, asynParamFloat64Array, &(p_i_wf_ch[i]));
        sprintf(param_name, Q_WF_STR,       i); createParam(param_name, asynParamFloat64Array, &(p_q_wf_ch[i]));
        sprintf(param_name, P_WF_STR,       i); createParam(param_name, asynParamFloat64Array, &(p_p_wf_ch[i]));
        sprintf(param_name, A_WF_STR,       i); createParam(param_name, asynParamFloat64Array, &(p_a_wf_ch[i]));


        sprintf(param_name, GET_IQ_WF_STR,  i); createParam(param_name, asynParamInt32, &(p_get_iq_wf_ch[i]));

        sprintf(param_name, AMPL_COEFF_STR, i);  createParam(param_name, asynParamFloat64, &(p_ampl_coeff[i]));
        sprintf(param_name, POWER_COEFF_STR, i); createParam(param_name, asynParamFloat64, &(p_power_coeff[i]));
       
    }

    sprintf(param_name, GET_IQ_WF_ALL_STR); createParam(param_name, asynParamInt32, &p_get_iq_wf_all);

    for(int i = 0; i < NUM_TIMESLOT; i++) {    // for loop for number of timeslots
        sprintf(param_name, P_DES_STR, i); createParam(param_name, asynParamFloat64, &(p_p_des_ts[i]));
        sprintf(param_name, A_DES_STR, i); createParam(param_name, asynParamFloat64, &(p_a_des_ts[i]));

        sprintf(param_name, P_FB_STR,  i); createParam(param_name, asynParamFloat64, &(p_p_fb_ts[i]));
        sprintf(param_name, A_FB_STR,  i); createParam(param_name, asynParamFloat64, &(p_a_fb_ts[i]));
        sprintf(param_name, P_REF_STR, i); createParam(param_name, asynParamFloat64, &(p_p_ref_ts[i]));
        sprintf(param_name, A_REF_STR, i); createParam(param_name, asynParamFloat64, &(p_a_ref_ts[i]));
        sprintf(param_name, P_SET_STR, i); createParam(param_name, asynParamFloat64, &(p_p_set_ts[i]));
        sprintf(param_name, A_SET_STR, i); createParam(param_name, asynParamFloat64, &(p_a_set_ts[i]));

        sprintf(param_name, PHASE_JITTER_STR, i); createParam(param_name, asynParamFloat64, &(p_rms_phase[i])); 
        sprintf(param_name, AMPL_JITTER_STR,  i); createParam(param_name, asynParamFloat64, &(p_rms_ampl[i]));
        sprintf(param_name, BV_JITTER_STR,    i); createParam(param_name, asynParamFloat64, &(p_rms_bv[i]));
        sprintf(param_name, PHASE_MEAN_STR,   i); createParam(param_name, asynParamFloat64, &(p_mean_phase[i]));
        sprintf(param_name, AMPL_MEAN_STR,    i); createParam(param_name, asynParamFloat64, &(p_mean_ampl[i]));
        sprintf(param_name, BV_MEAN_STR,      i); createParam(param_name, asynParamFloat64, &(p_mean_bv[i]));
    }
    // non-timeslot aware statistics
    sprintf(param_name, PHASE_JITTER_NT_STR); createParam(param_name, asynParamFloat64, &(p_rms_phase_nt)); 
    sprintf(param_name, AMPL_JITTER_NT_STR);  createParam(param_name, asynParamFloat64, &(p_rms_ampl_nt));
    sprintf(param_name, BV_JITTER_NT_STR);    createParam(param_name, asynParamFloat64, &(p_rms_bv_nt));
    sprintf(param_name, PHASE_MEAN_NT_STR);   createParam(param_name, asynParamFloat64, &(p_mean_phase_nt));
    sprintf(param_name, AMPL_MEAN_NT_STR);    createParam(param_name, asynParamFloat64, &(p_mean_ampl_nt));
    sprintf(param_name, BV_MEAN_NT_STR);      createParam(param_name, asynParamFloat64, &(p_mean_bv_nt));

    for(int t = 0; t < NUM_TIMESLOT; t++) {
        for(int w = 0; w < NUM_WINDOW; w++){        // w, window index
            for(int i = 0; i < NUM_FB_CH; i++) {    // i, channel index
                sprintf(param_name, P_BR_WND_CH_STR, t, w, i); createParam(param_name, asynParamFloat64, &(p_br[t].p_br_phase[w][i]));
                sprintf(param_name, A_BR_WND_CH_STR, t, w, i); createParam(param_name, asynParamFloat64, &(p_br[t].p_br_amplitude[w][i]));
            }
        }
        sprintf(param_name, P_BR_STR, t); createParam(param_name, asynParamFloat64, &(p_br[t].p_br_pact));
        sprintf(param_name, A_BR_STR, t); createParam(param_name, asynParamFloat64, &(p_br[t].p_br_aact));

        sprintf(param_name, BVOLT_BR_STR, t); createParam(param_name, asynParamFloat64, &(p_br[t].p_br_bvolt));
    }

    // Beam Rate PVs for No Timeslot Aware
    for(int w = 0; w < NUM_WINDOW; w++) {       // w, window index
        for(int i = 0; i < NUM_FB_CH; i++) {    // i, channel index
            sprintf(param_name, P_BR_NT_WND_CH_STR, w, i); createParam(param_name, asynParamFloat64, &(p_brNT.p_br_phase[w][i]));
            sprintf(param_name, A_BR_NT_WND_CH_STR, w, i); createParam(param_name, asynParamFloat64, &(p_brNT.p_br_amplitude[w][i]));
        }
    }

    sprintf(param_name, P_BR_NT_STR);  createParam(param_name, asynParamFloat64, &(p_brNT.p_br_pact));
    sprintf(param_name, A_BR_NT_STR);  createParam(param_name, asynParamFloat64, &(p_brNT.p_br_aact));
    sprintf(param_name, BVOLT_BR_NT_STR); createParam(param_name, asynParamFloat64, &(p_brNT.p_br_bvolt));


    sprintf(param_name, BVOLT_CONV_STR); createParam(param_name, asynParamFloat64,      &p_bvolt_conv);

    sprintf(param_name, I_BASEBAND_STR); createParam(param_name, asynParamFloat64Array, &(p_i_baseband_wf));
    sprintf(param_name, Q_BASEBAND_STR); createParam(param_name, asynParamFloat64Array, &(p_q_baseband_wf));
    sprintf(param_name, A_BASEBAND_STR); createParam(param_name, asynParamFloat64Array, &(p_a_baseband_wf));
    sprintf(param_name, P_BASEBAND_STR); createParam(param_name, asynParamFloat64Array, &(p_p_baseband_wf));
    sprintf(param_name, I_BASEBAND_RBV_STR); createParam(param_name, asynParamFloat64Array, &(p_i_baseband_wf_rbv));
    sprintf(param_name, Q_BASEBAND_RBV_STR); createParam(param_name, asynParamFloat64Array, &(p_q_baseband_wf_rbv));
    sprintf(param_name, A_BASEBAND_RBV_STR); createParam(param_name, asynParamFloat64Array, &(p_a_baseband_wf_rbv));
    sprintf(param_name, P_BASEBAND_RBV_STR); createParam(param_name, asynParamFloat64Array, &(p_p_baseband_wf_rbv));



    sprintf(param_name, OP_MODE_STR);              createParam(param_name, asynParamInt32,   &p_op_mode);
    sprintf(param_name, PHASE_ADAPTIVE_GAIN_STR);  createParam(param_name, asynParamFloat64, &p_p_adaptive_gain);
    sprintf(param_name, AMPL_ADAPTIVE_GAIN_STR);   createParam(param_name, asynParamFloat64, &p_a_adaptive_gain);
    sprintf(param_name, PHASE_DISTB_GAIN_STR);     createParam(param_name, asynParamFloat64, &p_p_distb_gain);
    sprintf(param_name, AMPL_DISTB_GAIN_STR);      createParam(param_name, asynParamFloat64, &p_a_distb_gain);

    for(int i = 0; i < NUM_HARMONICS; i++) {
        sprintf(param_name, HARMO_CS_STR, i + 1);      createParam(param_name, asynParamFloat64, &p_harmo_cs[i]);
        sprintf(param_name, HARMO_SN_STR, i + 1);      createParam(param_name, asynParamFloat64, &p_harmo_sn[i]);
    }

    sprintf(param_name, PHASE_ALPHA_STR);          createParam(param_name, asynParamFloat64, &p_p_alpha);
    sprintf(param_name, AMPL_ALPHA_STR);           createParam(param_name, asynParamFloat64, &p_a_alpha);


    for(int i = 0; i < NUM_FB_CH; i++) {
        sprintf(param_name, SC_BSA_PCH_SLOPE, i);  createParam(param_name, asynParamFloat64, &(p_scBsa_linconv_ch[i].p_p_slope));
        sprintf(param_name, SC_BSA_PCH_OFFSET, i); createParam(param_name, asynParamFloat64, &(p_scBsa_linconv_ch[i].p_p_offset));
        sprintf(param_name, SC_BSA_ACH_SLOPE, i);  createParam(param_name, asynParamFloat64, &(p_scBsa_linconv_ch[i].p_a_slope));
        sprintf(param_name, SC_BSA_ACH_OFFSET, i); createParam(param_name, asynParamFloat64, &(p_scBsa_linconv_ch[i].p_a_offset));
    }

    sprintf(param_name, SC_BSA_PACT_SLOPE);  createParam(param_name, asynParamFloat64, &(p_scBsa_linconv_fb.p_p_slope));
    sprintf(param_name, SC_BSA_PACT_OFFSET); createParam(param_name, asynParamFloat64, &(p_scBsa_linconv_fb.p_p_offset));
    sprintf(param_name, SC_BSA_AACT_SLOPE);  createParam(param_name, asynParamFloat64, &(p_scBsa_linconv_fb.p_a_slope));
    sprintf(param_name, SC_BSA_AACT_OFFSET); createParam(param_name, asynParamFloat64, &(p_scBsa_linconv_fb.p_a_offset));

}


void llrfHlsAsynDriver::bsaSetup(void)
{
    char param_name[128];


    for(int w = 0; w < 1 /*NUM_WINDOW*/ ; w++) {       // w, window index
        for(int i = 0; i < NUM_FB_CH; i++) {    // i, channel index
            sprintf(param_name, P_BSA_WND_CH_STR, bsa_name, w, i); BsaChn_phase[w][i]     = BSA_CreateChannel(param_name);
            sprintf(param_name, A_BSA_WND_CH_STR, bsa_name, w, i); BsaChn_amplitude[w][i] = BSA_CreateChannel(param_name);
        }
    } 

    sprintf(param_name, PDES_BSA_FB_STR,  bsa_name);  BsaChn_pdes  = BSA_CreateChannel(param_name);
    sprintf(param_name, ADES_BSA_FB_STR,  bsa_name);  BsaChn_ades  = BSA_CreateChannel(param_name);
    sprintf(param_name, PSET_BSA_FB_STR,  bsa_name);  BsaChn_pset  = BSA_CreateChannel(param_name);
    sprintf(param_name, ASET_BSA_FB_STR,  bsa_name);  BsaChn_aset  = BSA_CreateChannel(param_name); 
    sprintf(param_name, PACT_BSA_FB_STR,  bsa_name);  BsaChn_pact  = BSA_CreateChannel(param_name);
    sprintf(param_name, AACT_BSA_FB_STR,  bsa_name);  BsaChn_aact  = BSA_CreateChannel(param_name);
    sprintf(param_name, BVOLT_BSA_STR,    bsa_name);  BsaChn_bvolt = BSA_CreateChannel(param_name);


   BSA_ConfigSetAllPriorites(90);
}


void llrfHlsAsynDriver::beamPeakVoltageProcessing(bsa_packet_t  *p)
{
    int t   = p->time_slot;
    int raw = p->terminator.u16u16_terminator.u16upper;

    beam_peak_volt[t].raw = raw;
    beam_peak_volt[t].val = convBeamPeakVolt * raw;

// do not use TS0 as non-timeslot aware
/*
   if(t) {
       beam_peak_volt[0].raw = beam_peak_volt[t].raw;
       beam_peak_volt[0].val = beam_peak_volt[t].val;
   }
*/
}

void llrfHlsAsynDriver::bsaProcessing(bsa_packet_t *p)
{
    int t = p->time_slot;

    BSA_StoreData(BsaChn_pdes, p->time, n_angle(phase_des_ts[t] * 180./M_PI), 0, 0);
    BSA_StoreData(BsaChn_ades, p->time, ampl_des_ts[t], 0, 0);
    BSA_StoreData(BsaChn_pset, p->time, n_angle(p->phase_set * 180./M_PI), 0, 0);
    BSA_StoreData(BsaChn_aset, p->time, p->ampl_set, 0, 0);
    BSA_StoreData(BsaChn_pact, p->time, n_angle(p->phase_fb * 180./M_PI), 0, 0);
    BSA_StoreData(BsaChn_aact, p->time, p->ampl_fb,  0, 0);
    BSA_StoreData(BsaChn_bvolt, p->time, beam_peak_volt[t].val, 0, 0);

    for(int w = 0; w < 1 /*NUM_WINDOW*/ ; w++) {       // w, windw index
        for(int i = 0; i < NUM_FB_CH; i++) {    // i, channel index
            BSA_StoreData(BsaChn_phase[w][i],     p->time, n_angle(p->ap_wch[w][i].phase * 180./M_PI), 0, 0);
            BSA_StoreData(BsaChn_amplitude[w][i], p->time, p->ap_wch[w][i].ampl,  0, 0);
        }
    }

}

void llrfHlsAsynDriver::fastPVProcessing(bsa_packet_t *p)
{

    // todo, update timestamp for asyn Port driver

    int t = p->time_slot;
    double phase;

    phase = n_angle(p->phase_fb * 180./M_PI);
    setDoubleParam(p_br[t].p_br_pact, phase);                   setDoubleParam(p_brNT.p_br_pact, phase);
    setDoubleParam(p_br[t].p_br_aact, p->ampl_fb);              setDoubleParam(p_brNT.p_br_aact, p->ampl_fb);
    setDoubleParam(p_br[t].p_br_bvolt, beam_peak_volt[t].val);  setDoubleParam(p_brNT.p_br_bvolt, beam_peak_volt[t].val);



    for(int w = 0; w < 1 /*NUM_WINDOW*/ ; w++) {
        for(int i = 0; i < NUM_FB_CH; i++) {
            phase = n_angle(p->ap_wch[w][i].phase * 180./M_PI);
            setDoubleParam(p_br[t].p_br_phase[w][i], phase);                      setDoubleParam(p_brNT.p_br_phase[w][i], phase);
            setDoubleParam(p_br[t].p_br_amplitude[w][i], p->ap_wch[w][i].ampl);   setDoubleParam(p_brNT.p_br_amplitude[w][i], p->ap_wch[w][i].ampl);
        }
    }

    callParamCallbacks();
}


void llrfHlsAsynDriver::recalc_dequantization(void)
{
    double norm = 0.;
    double fb_c = 0.;
    double ph_c = 360. / sc_quantization;
    double ph_offset = -180.;
    double a_offset  = 0.;
    double a_c[NUM_FB_CH];
    

    for(int i = 0; i < NUM_FB_CH; i++) {
        for(int j = 0; j < NUM_DEST; j++) {
            norm += fb_weight_ch[j*NUM_FB_CH + i];
            fb_c += ampl_coeff_ch[i] * fb_weight_ch[j*NUM_FB_CH + i];
        }
        a_c[i] = ampl_coeff_ch[i] / sc_quantization;
    }
    if(norm > 0.1) fb_c /= norm;
    fb_c /= sc_quantization;

    setDoubleParam(p_scBsa_linconv_fb.p_p_slope, ph_c);
    setDoubleParam(p_scBsa_linconv_fb.p_p_offset, ph_offset);
    setDoubleParam(p_scBsa_linconv_fb.p_a_slope, fb_c);
    setDoubleParam(p_scBsa_linconv_fb.p_a_offset, a_offset);

    for(int i = 0; i < NUM_FB_CH; i++) {
        setDoubleParam(p_scBsa_linconv_ch[i].p_p_slope,  ph_c);
        setDoubleParam(p_scBsa_linconv_ch[i].p_p_offset, ph_offset);
        setDoubleParam(p_scBsa_linconv_ch[i].p_a_slope,  a_c[i]);
        setDoubleParam(p_scBsa_linconv_ch[i].p_a_offset, a_offset);
    }
}


void llrfHlsAsynDriver::iq2pa(const epicsFloat64* i, const epicsFloat64* q, epicsFloat64* p, epicsFloat64* a)
{
    for(std::size_t k { 0 }; k < MAX_SAMPLES; ++k)
    {
        *p++ = atan2(*q, *i) * 180./M_PI;
        *a++ = sqrt((*i)*(*i) + (*q)*(*q));
        i++;
        q++;
    }
}

void llrfHlsAsynDriver::iq2pa(const epicsFloat64* i, const epicsFloat64* q, epicsFloat64* p, epicsFloat64* a, epicsFloat64 ampl_scale)
{
    for(std::size_t k { 0 }; k < MAX_SAMPLES; ++k)
    {
        *p++ = atan2(*q, *i) * 180./M_PI;
        *a++ = sqrt((*i)*(*i) + (*q)*(*q)) * ampl_scale;
        i++;
        q++;
    }
}



void llrfHlsAsynDriver::pa2iq(const epicsFloat64* p, const epicsFloat64* a, epicsFloat64* i, epicsFloat64* q)
{
    for(std::size_t k { 0 }; k < MAX_SAMPLES; ++k)
    {
        *i++ = *a * cos( *p * M_PI / 180. );
        *q++ = *a * sin( *p * M_PI / 180. );
        p++;
        a++;
    }
}

void llrfHlsAsynDriver::UpdateIQBasebandWaveform()
{
    // Update the Phase/Amplitude waveforms
    pa2iq(p_baseband_wf, a_baseband_wf, i_baseband_wf, q_baseband_wf);

    // Write I/Q baseband waveforms
    dacSigGen->setIWaveform(i_baseband_wf);
    dacSigGen->setQWaveform(q_baseband_wf);

    // Update readback waveforms
    DoCallbacksReadbackBasebandWaveform();
}

void llrfHlsAsynDriver::UpdatePABasebandWaveform()
{
    // Update the I/Q waveforms
    iq2pa(i_baseband_wf, q_baseband_wf, p_baseband_wf, a_baseband_wf);

    // Write I/Q baseband waveforms
    dacSigGen->setIWaveform(i_baseband_wf);
    dacSigGen->setQWaveform(q_baseband_wf);

    // Update readback waveforms
    DoCallbacksReadbackBasebandWaveform();
}

void llrfHlsAsynDriver::UpdateIQWaveformAverageWindow(int w)
{
    // Update the Phase/Amplitude waveforms
    pa2iq(pwf_avg_window[w], awf_avg_window[w], iwf_avg_window[w], qwf_avg_window[w]);

    // Write the I/Q waveforms to FW
    llrfHls->setIWaveformAverageWindow(iwf_avg_window[w], w);
    llrfHls->setQWaveformAverageWindow(qwf_avg_window[w], w);

    // Update readback waveforms
    DoCallbacksReadbackWaveformAverageWindow(w);
}

void llrfHlsAsynDriver::UpdatePAWaveformAverageWindow(int w)
{
    // Update the Phase/Ampl waveforms
    iq2pa(iwf_avg_window[w], qwf_avg_window[w], pwf_avg_window[w], awf_avg_window[w]);

    // Write the I/Q waveforms to FW
    llrfHls->setIWaveformAverageWindow(iwf_avg_window[w], w);
    llrfHls->setQWaveformAverageWindow(qwf_avg_window[w], w);

    // Update readback waveforms
    DoCallbacksReadbackWaveformAverageWindow(w);
}

void llrfHlsAsynDriver::ReadbacBasebandWaveform()
{
    // Read the I/Q baseband waveforms from FW
    dacSigGen->getIWaveform(i_baseband_wf);
    dacSigGen->getQWaveform(q_baseband_wf);

    // Update the Phase/Ampl waveform
    iq2pa(i_baseband_wf, q_baseband_wf, p_baseband_wf, a_baseband_wf);

    // Update the readback waveforms
    DoCallbacksReadbackBasebandWaveform();
}

void llrfHlsAsynDriver::ReadbackIQWaveformAverageWindow(int w)
{
    // Read the I/Q waveforms from FW
    llrfHls->getIWaveformAverageWindow(iwf_avg_window[w], w);
    llrfHls->getQWaveformAverageWindow(qwf_avg_window[w], w);

    // Update the Phase/Ampl waveforms
    iq2pa(iwf_avg_window[w], qwf_avg_window[w], pwf_avg_window[w], awf_avg_window[w]);

    // Update the readback waveforms
    DoCallbacksReadbackWaveformAverageWindow(w);
}

void llrfHlsAsynDriver::DoCallbacksReadbackBasebandWaveform()
{
    doCallbacksFloat64Array(i_baseband_wf, MAX_SAMPLES, p_i_baseband_wf_rbv, 0);
    doCallbacksFloat64Array(q_baseband_wf, MAX_SAMPLES, p_q_baseband_wf_rbv, 0);
    doCallbacksFloat64Array(a_baseband_wf, MAX_SAMPLES, p_a_baseband_wf_rbv, 0);
    doCallbacksFloat64Array(p_baseband_wf, MAX_SAMPLES, p_p_baseband_wf_rbv, 0);
}

void llrfHlsAsynDriver::DoCallbacksReadbackWaveformAverageWindow(int w)
{
    doCallbacksFloat64Array(iwf_avg_window[w], MAX_SAMPLES, p_iwf_avg_window_rbv[w], 0);
    doCallbacksFloat64Array(qwf_avg_window[w], MAX_SAMPLES, p_qwf_avg_window_rbv[w], 0);
    doCallbacksFloat64Array(awf_avg_window[w], MAX_SAMPLES, p_awf_avg_window_rbv[w], 0);
    doCallbacksFloat64Array(pwf_avg_window[w], MAX_SAMPLES, p_pwf_avg_window_rbv[w], 0);
}

void llrfHlsAsynDriver::checkAmplNorm()
{
    double shadow, readout;
    uint8_t flag;
    int     dest_idx;

    if(!recal_norm_flag) return;   // no requrest from software side

    
    llrfHls->getRecalNorm(&flag);
    if(!(flag &0x1<<2)) return;               // firmware did not proceed the request yet

    dest_idx = flag & 0x3;

    
    recal_norm_flag = false;       // clear flag
    getDoubleParam(p_ampl_norm[dest_idx], &shadow);  // get shadow value which is located/is latched in asyn port driver 
                                           // the shadow value is previous set value from sotware, 
    llrfHls->getAmplNorm(&readout, dest_idx);        // read out normalization factor from firmware, using read-only register

    if(shadow != readout && readout > 1.E-6) {
        setDoubleParam(p_ampl_norm_pb[dest_idx], 0.);            // clean up previous value in the pushback
        setDoubleParam(p_ampl_norm_pb[dest_idx], (1./readout));  // apply new set value for the normalization 
                                                       // remarks PV and firmware value has reciprocal relation
    }

}

extern "C" {

extern int ComplexAvgWindow;
epicsExportAddress(int, ComplexAvgWindow);


// driver configuration, C wrapper 
int llrfHlsAsynDriverConfigure(const char *portName, const char *regPathString, const char *hlsStream, const char *bsa_prefix, const char *named_root)
{
    init_drvList();

    pDrvList_t *p = find_drvByPort(portName);
    if(p) {
        printf("llrfHlsAsynDriver found that port name (%s) has been used.\n", portName);
        return 0;
    }

    p               = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "llrfHlsAsyn driver: llrfHlsAsynDriverConfigure()");
    p->named_root   = (named_root && strlen(named_root))?epicsStrDup(named_root):cpswGetRootName();
    p->port         = epicsStrDup(portName);
    p->regPath      = epicsStrDup(regPathString);
    p->hlsStream    = (hlsStream && strlen(hlsStream))?epicsStrDup(hlsStream): NULL;
    p->bsa_prefix   = (bsa_prefix && strlen(bsa_prefix))?epicsStrDup(bsa_prefix): (char *) "";
    p->pRtm         = (void *) NULL;
    p->pLlrfHlsAsyn = new llrfHlsAsynDriver((void *) p, (const char *) p->port, (const char *) p->regPath, (const char *) p->hlsStream, (const char *) p->bsa_prefix, (const char *) p->named_root);


    ellAdd(pDrvEllList, &p->node);

    return 0;
}



// prepare iocsh commnand
static const iocshArg initArg0 = {"port name",      iocshArgString};
static const iocshArg initArg1 = {"register path",  iocshArgString};
static const iocshArg initArg2 = {"hls stream",     iocshArgString};
static const iocshArg initArg3 = {"bsa prefix",     iocshArgString};
static const iocshArg initArg4 = {"named_root",     iocshArgString};
static const iocshArg * const initArgs[] = { &initArg0,
                                             &initArg1,
                                             &initArg2,
                                             &initArg3,
                                             &initArg4 };
static const iocshFuncDef initFuncDef = {"llrfHlsAsynDriverConfigure", 5, initArgs};
static void  initCallFunc(const iocshArgBuf *args)
{
    llrfHlsAsynDriverConfigure(args[0].sval,     /* port name */
                               args[1].sval,     /* register path */
                               (args[2].sval && strlen(args[2].sval))?args[2].sval: NULL, /* hls stream */
                               (args[3].sval && strlen(args[3].sval))?args[3].sval: NULL, /* bsa prefix*/
                               (args[4].sval && strlen(args[4].sval))?args[4].sval: NULL /* named_root */ );
}


static void llrfHlsAsynDriverRegister(void)
{
    iocshRegister(&initFuncDef,  initCallFunc);
}


epicsExportRegistrar(llrfHlsAsynDriverRegister);


static int llrfHlsAsynDriverPoll(void)
{
    while(keep_stay_in_loop) {
        pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
        while(p) {
            if(p->pLlrfHlsAsyn) p->pLlrfHlsAsyn->poll();
            p = (pDrvList_t *) ellNext(&p->node);
        }
        epicsThreadSleep(1. / POLL_RATE);
    }

    epicsEventSignal(shutdownEvent);

    return 0;
}

static int llrfHlsAsynDriverStreamPoll(void *p)
{
    ((llrfHlsAsynDriver*)p)->pollStream();

     return 0;
}

static void stopPollingThread(void *p)
{
    keep_stay_in_loop = false;
    epicsEventWait(shutdownEvent);
    epicsPrintf("llrfHlsAsynDriver: stop polling thread (%s)\n", (char *)p);
}


// EPICS driver support for llrfHlsAsynDriver

static int llrfHlsAsynDriverReport(int interest);
static int llrfHlsAsynDriverInitialize(void);

static struct drvet llrfHlsAsynDriver = {
    2,
    (DRVSUPFUN) llrfHlsAsynDriverReport,
    (DRVSUPFUN) llrfHlsAsynDriverInitialize
};

epicsExportAddress(drvet, llrfHlsAsynDriver);



static int llrfHlsAsynDriverReport(int interest)
{

   init_drvList();
    printf("Total %d of llrfHlsAsyn driver instance(s) is(are) registered\n", ellCount(pDrvEllList));

    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
    while(p) {
        printf("\tnamed root   : %s\n", p->named_root);
        printf("\tport name    : %s\n", p->port);
        printf("\tregoster path: %s\n", p->regPath);
        printf("\tllrfHlsAsyn  : %p\n", p->pLlrfHlsAsyn);
        printf("\thlsStream    : %p\n", p->hlsStream);
        printf("\tbsa_prefix   : %s\n", p->bsa_prefix);
        printf("\trtm          : %p\n", p->pRtm);
        if(p->pLlrfHlsAsyn) p->pLlrfHlsAsyn->report(interest);
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return 0;
}

static int llrfHlsAsynDriverInitialize(void)
{

    init_drvList();

    if(!pDrvEllList) {
        printf("llrfHlsAsynDriver never been configured\n");
        return 0;
    }

    keep_stay_in_loop = true;
    shutdownEvent     = epicsEventMustCreate(epicsEventEmpty);
    const char *name  = "llrfHlsPoll";

    epicsThreadCreate(name, epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) llrfHlsAsynDriverPoll, 0);

    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
    while(p) {
        if(p->pLlrfHlsAsyn && p->hlsStream) epicsThreadCreate("llrfHlsStreamPoll", epicsThreadPriorityHigh,
                                                               epicsThreadGetStackSize(epicsThreadStackMedium),
                                                               (EPICSTHREADFUNC) llrfHlsAsynDriverStreamPoll, (void *) p->pLlrfHlsAsyn);
         p = (pDrvList_t *) ellNext(&p->node);
        }


    epicsAtExit3((epicsExitFunc) stopPollingThread, (void *) epicsStrDup(name), epicsStrDup(name));

    return 0;
}


} /* end of extern C */
