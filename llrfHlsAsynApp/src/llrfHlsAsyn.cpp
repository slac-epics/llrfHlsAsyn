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
#include "debugStreamInterface.h"

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

    need_to_read      = true;
    stream_read_count = 0;
    stream_read_size  = 0;
    current_bsa       = -1;
    p_buf             = (uint8_t *) &p_bsa_buf[0];
    bsa_macro = (bsa_prefix && strlen(bsa_prefix))? epicsStrDup(bsa_prefix):epicsStrDup("default_bsa");

    try {
        p_root = (named_root && strlen(named_root))? cpswGetNamedRoot(named_root): cpswGetRoot();
        p_llrfHls = p_root->findByName(pathString);
        if(stream) hls_stream_ = IStream::create(p_root->findByName(stream));

    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s, file %s, line %d\n", e.getInfo().c_str(), __FILE__, __LINE__);
        throw e;
    }

    llrfHls = IllrfFw::create(p_llrfHls);
    dacSigGen = IdacSigGenFw::create(p_root->findByName("mmio"));


    ParameterSetup();
    bsaSetup();

    getFirmwareInformation();

    callParamCallbacks();
}

llrfHlsAsynDriver::~llrfHlsAsynDriver() {}

asynStatus llrfHlsAsynDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int        function      = pasynUser->reason;
    asynStatus status        = asynSuccess;
    const char *functionName = "writeInt32";

    status = (asynStatus) setIntegerParam(function, value);

    if(function == p_stream_enable) { // stream enable/disable congtrol
        llrfHls->setStreamEnable(value?true:false);
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
    else
    for(int i = 0; i < NUM_FB_CH; i++) {
        if(function == p_get_iq_wf_ch[i] && value) {
            getIQWaveform(i);
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

    for(int i = 0; i < NUM_FB_CH; i++) {   // search for channel number
        if(function == p_ref_weight_ch[i]) {    // channel weight for reference
            llrfHls->setReferenceChannelWeight(value, i);
            break;
        }
        else if(function == p_fb_weight_ch[i]) {  // channel weight for feedback
            llrfHls->setFeedbackChannelWeight(value, i);
            break;
        }
        else if(function == p_p_offset_ch[i]) {    // phase offset for channel
            llrfHls->setPhaseOffset(value, i);
            break;
        }
    }

    for(int i = 0; i < NUM_TIMESLOT; i++) {    // search for timeslot
         if(function == p_p_des_ts[i]) {  // desired phase for a specific timeslot
             llrfHls->setDesiredPhase(value, i);
             break;
         }
         else if(function == p_a_des_ts[i]) {  // desired amplitude for a specific timeslot
             llrfHls->setDesiredAmpl(value, i);
             break;
         }
    }
    

    return status;
}

static epicsFloat64* __zero_pad(epicsFloat64 *value, epicsFloat64 *v, size_t nElements)
{
    if(nElements >= MAX_SAMPLES) return value;

    int i;
    for(i = 0; i < nElements;   i++)  *(v + i) = *(value + i);
    for(     ; i < MAX_SAMPLES; i++)  *(v + i) = 0.;

    return v;
}

asynStatus llrfHlsAsynDriver::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements)
{
    int        function      = pasynUser->reason;
    asynStatus status        = asynSuccess;
    const char *functionName = "writeFloat64Array";

    epicsFloat64 v[MAX_SAMPLES];


    if(function == i_baseband_wf) {
        dacSigGen->setIWaveform(__zero_pad(value, v, nElements));
        // printf("baseband i waveform size: %d\n", nElements);
    }
    else if(function == q_baseband_wf) {
        dacSigGen->setQWaveform(__zero_pad(value, v, nElements));
        // printf("baseband q waveform size: %d\n", nElements);
    }
    else

    for(int w = 0; w < NUM_WINDOW; w++) {
        if(function == p_avg_window[w]) {    // update average window
            llrfHls->setAverageWindow(__zero_pad(value, v, nElements), w);
            // printf("avg window %d size:%d\n", w, nElements);
            break;
        }
        else if(function == p_iwf_avg_window[w]) {  // update complex waveform I
            llrfHls->setIWaveformAverageWindow(__zero_pad(value, v, nElements), w);
            // printf("i window %d size: %d\n", w, nElements);
            break;
        }
        else if(function == p_qwf_avg_window[w]) { // update complex waveform Q
            llrfHls->setQWaveformAverageWindow(__zero_pad(value, v, nElements), w);
            // printf("q window %d size: %d\n", w, nElements);
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

    char ts_str[80];
    epicsTimeStamp ts = *(epicsTimeStamp *) (u1+1);
    epicsTimeToStrftime(ts_str, sizeof(ts_str), "%Y/%m/%d %H:%M:%S.%09f", &ts);
    printf("\ttimestamp in stream : %s\n", ts_str);

}

void llrfHlsAsynDriver::poll(void)
{
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

    /*
    for(int i = 0; i < NUM_FB_CH; i++) {
        llrfHls->getIWaveform(i_wf_ch[i], i);
        llrfHls->getQWaveform(q_wf_ch[i], i);
    }
    */



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
        bsaProcessing(&p_bsa_buf[current_bsa]);
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
    }

    llrfHls->freezeWaveform(false);

    flushIQWaveformsforAllChannels();
}


void llrfHlsAsynDriver::getIQWaveform(int channel)
{
    llrfHls->freezeWaveform(true);
    llrfHls->getIQWaveform(i_wf_ch[channel], q_wf_ch[channel], channel);
    llrfHls->freezeWaveform(false);


    doCallbacksFloat64Array(i_wf_ch[channel], MAX_SAMPLES, p_i_wf_ch[channel], 0);
    doCallbacksFloat64Array(q_wf_ch[channel], MAX_SAMPLES, p_q_wf_ch[channel], 0);
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

void llrfHlsAsynDriver::ParameterSetup(void)
{
    char param_name[80];

    sprintf(param_name, STREAM_ENABLE_STR);          createParam(param_name, asynParamInt32,   &p_stream_enable);
    sprintf(param_name, MODE_CONFIG_STR);            createParam(param_name, asynParamInt32,   &p_mode_config);
    
    sprintf(param_name, VERSION_STR);                createParam(param_name, asynParamInt32,   &p_version);
    sprintf(param_name, NUM_TIMESLOT_STR);           createParam(param_name, asynParamInt32,   &p_num_timeslot);
    sprintf(param_name, NUM_CHANNEL_STR);            createParam(param_name, asynParamInt32,   &p_num_channel);
    sprintf(param_name, NUM_WINDOW_STR);             createParam(param_name, asynParamInt32,   &p_num_window);
    sprintf(param_name, MAX_PULSE_LEN_STR);          createParam(param_name, asynParamInt32,   &p_max_pulse_len);
    sprintf(param_name, COUNTER_STR);                createParam(param_name, asynParamInt32,   &p_counter);
    sprintf(param_name, DROP_COUNTER_STR);           createParam(param_name, asynParamInt32,   &p_drop_counter);

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

    for(int w = 0; w < NUM_WINDOW; w++) {
        sprintf(param_name, AVG_WINDOW_STR, w);     createParam(param_name, asynParamFloat64Array, &(p_avg_window[w]));
        sprintf(param_name, IWF_AVG_WINDOW_STR, w); createParam(param_name, asynParamFloat64Array, &(p_iwf_avg_window[w]));
        sprintf(param_name, QWF_AVG_WINDOW_STR, w); createParam(param_name, asynParamFloat64Array, &(p_qwf_avg_window[w]));
    }

    for(int i = 0; i < NUM_FB_CH; i++) {    // for loop for number of channels
        sprintf(param_name, REF_WEIGHT_STR, i); createParam(param_name, asynParamFloat64, &(p_ref_weight_ch[i]));
        sprintf(param_name, FB_WEIGHT_STR,  i); createParam(param_name, asynParamFloat64, &(p_fb_weight_ch[i]));
        sprintf(param_name, P_OFFSET_STR,   i); createParam(param_name, asynParamFloat64, &(p_p_offset_ch[i]));

        for(int w = 0; w < NUM_WINDOW; w++) {
            sprintf(param_name, P_WND_CH_STR, w, i); createParam(param_name, asynParamFloat64, &(p_p_wnd_ch[w][i]));
            sprintf(param_name, A_WND_CH_STR, w, i); createParam(param_name, asynParamFloat64, &(p_a_wnd_ch[w][i]));
        }

        sprintf(param_name, I_WF_STR,       i); createParam(param_name, asynParamFloat64Array, &(p_i_wf_ch[i]));
        sprintf(param_name, Q_WF_STR,       i); createParam(param_name, asynParamFloat64Array, &(p_q_wf_ch[i]));


        sprintf(param_name, GET_IQ_WF_STR,  i); createParam(param_name, asynParamInt32, &(p_get_iq_wf_ch[i]));
       
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


    }

    for(int t = 0; t < NUM_TIMESLOT; t++) {
        for(int w = 0; w < NUM_WINDOW; w++){        // w, window index
            for(int i = 0; i < NUM_FB_CH; i++) {    // i, channel index
                sprintf(param_name, P_BR_WND_CH_STR, t, w, i); createParam(param_name, asynParamFloat64, &(p_br[t].p_br_phase[w][i]));
                sprintf(param_name, A_BR_WND_CH_STR, t, w, i); createParam(param_name, asynParamFloat64, &(p_br[t].p_br_amplitude[w][i]));
            }
        }
        sprintf(param_name, P_BR_STR, t); createParam(param_name, asynParamFloat64, &(p_br[t].p_br_pact));
        sprintf(param_name, A_BR_STR, t); createParam(param_name, asynParamFloat64, &(p_br[t].p_br_aact));
    }

    sprintf(param_name, I_BASEBAND_STR); createParam(param_name, asynParamFloat64Array, &(i_baseband_wf));
    sprintf(param_name, Q_BASEBAND_STR); createParam(param_name, asynParamFloat64Array, &(q_baseband_wf));
}


void llrfHlsAsynDriver::bsaSetup(void)
{
    char param_name[128];

    BSA_ConfigSetAllPriorites(90);
    /*
    for(int w = 0; w < NUM_WINDOW ; w++) {       // w, window index
        for(int i = 0; i < NUM_FB_CH; i++) {    // i, channel index
            sprintf(param_name, P_BSA_WND_CH_STR, bsa_macro, w, i); BsaChn_phase[w][i]     = BSA_CreateChannel(param_name);
            sprintf(param_name, A_BSA_WND_CH_STR, bsa_macro, w, i); BsaChn_amplitude[w][i] = BSA_CreateChannel(param_name);
        }
    } 
    */
    sprintf(param_name, P_BSA_FB_STR, bsa_macro);  BsaChn_pact = BSA_CreateChannel(param_name);
    sprintf(param_name, A_BSA_FB_STR, bsa_macro);  BsaChn_aact = BSA_CreateChannel(param_name);
}

void llrfHlsAsynDriver::bsaProcessing(bsa_packet_t *p)
{
    BSA_StoreData(BsaChn_pact, p->time, n_angle(p->phase_fb * 180./M_PI), 0, 0);
    BSA_StoreData(BsaChn_aact, p->time, p->ampl_fb,  0, 0);
    /*
    for(int w = 0; w < NUM_WINDOW ; w++) {       // w, windw index
        for(int i = 0; i < NUM_FB_CH; i++) {    // i, channel index
            BSA_StoreData(BsaChn_phase[w][i],     p->time, p->phase[w][i], 0, 0);
            BSA_StoreData(BsaChn_amplitude[w][i], p->time, p->ampl[w][i],  0, 0);
        }
    }
    */
}

void llrfHlsAsynDriver::fastPVProcessing(bsa_packet_t *p)
{

    // todo, update timestamp for asyn Port driver

    int t = p->time_slot;

    setDoubleParam(p_br[t].p_br_pact, n_angle(p->phase_fb * 180./M_PI));
    setDoubleParam(p_br[t].p_br_aact, p->ampl_fb);

    for(int w = 0; w < NUM_WINDOW ; w++) {
        for(int i = 0; i < NUM_FB_CH; i++) {
            setDoubleParam(p_br[t].p_br_phase[w][i], n_angle(p->ap_wch[w][i].phase * 180./M_PI));
            setDoubleParam(p_br[t].p_br_amplitude[w][i], p->ap_wch[w][i].ampl);
        }
    }

    callParamCallbacks();
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
    p->bsa_prefix   = (bsa_prefix && strlen(bsa_prefix))?epicsStrDup(bsa_prefix): (char *) "Default_BSA";
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
        epicsThreadSleep(1.);
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
