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

static bool          keep_stay_in_loop = true;
static epicsEventId  shutdownEvent;

static ELLLIST *pDrvEllList = NULL;

typedef struct {
    ELLNODE           node;
    char              *named_root;
    char              *port;
    char              *regPath;
    llrfHlsAsynDriver *pLlrfHlsAsyn;
} pDrvList_t;


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



llrfHlsAsynDriver::llrfHlsAsynDriver(const char *portName, const char *pathString, const char *named_root)
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

    try {
        p_root = (named_root && strlen(named_root))? cpswGetNamedRoot(named_root): cpswGetRoot();
        p_llrfHls = p_root->findByName(pathString);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s, file %s, line %d\n", e.getInfo().c_str(), __FILE__, __LINE__);
        throw e;
    }

    llrfHls = IllrfFw::create(p_llrfHls);
    ParameterSetup();

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

asynStatus llrfHlsAsynDriver::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements)
{
    int        function      = pasynUser->reason;
    asynStatus status        = asynSuccess;
    const char *functionName = "writeFloat64Array";

    for(int w = 0; w < NUM_WINDOW; w++) if(function == p_avg_window[w] && nElements == MAX_SAMPLES) {    // update average window
        llrfHls->setAverageWindow(value, w);
        break;
    }

    return status;

}

void llrfHlsAsynDriver::report(int interest)
{
    if(!interest) return;

    printf("\tfirmware viersion  : %u\n", version_);
    printf("\tnumber of timeslots: %u\n", num_timeslot_);
    printf("\tnumber of channels : %u\n", num_channel_);
    printf("\tnumber of windows  : %u\n", num_window_);
    printf("\tmax pulse length   : %u\n", max_pulse_len_);
    printf("\tcounter            : %u\n", counter_);
    printf("\tdrop counter       : %u\n", drop_counter_);


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

    flushIQWaveformsforAllChannels();

}

void llrfHlsAsynDriver::getIQWaveform(int channel)
{
    llrfHls->getIWaveform(i_wf_ch[channel], channel);
    llrfHls->getQWaveform(q_wf_ch[channel], channel);


    doCallbacksFloat64Array(i_wf_ch[channel], MAX_SAMPLES, p_i_wf_ch[channel], 0);
    doCallbacksFloat64Array(q_wf_ch[channel], MAX_SAMPLES, p_q_wf_ch[channel], 0);
}

void llrfHlsAsynDriver::updatePhasePVsforAllChannels(void)
{
    for(int w = 0; w < NUM_WINDOW; w++) for(int i = 0; i < NUM_FB_CH; i++) {
        setDoubleParam(p_p_wnd_ch[w][i], phase_wnd_ch[w][i]);
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
        setDoubleParam(p_p_fb_ts[i], fb_phase_ts[i]);
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
        setDoubleParam(p_p_ref_ts[i], ref_phase_ts[i]);
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
        setDoubleParam(p_p_set_ts[i], phase_set_ts[i]);
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
        sprintf(param_name, AVG_WINDOW_STR, w); createParam(param_name, asynParamFloat64Array, &(p_avg_window[w]));
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
}




extern "C" {

// driver configuration, C wrapper 
int llrfHlsAsynDriverConfigure(const char *portName, const char *regPathString, const char *named_root)
{
    init_drvList();

    pDrvList_t *p = find_drvByPort(portName);
    if(p) {
        printf("llrfHlsAsynDriver found that port name (%s) has been used.\n");
        return 0;
    }

    p               = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "llrfHlsAsyn driver: llrfHlsAsynDriverConfigure()");
    p->named_root   = (named_root && strlen(named_root))?epicsStrDup(named_root):cpswGetRootName();
    p->port         = epicsStrDup(portName);
    p->regPath      = epicsStrDup(regPathString);
    p->pLlrfHlsAsyn = new llrfHlsAsynDriver((const char *) p->port, (const char *) p->regPath, (const char *) p->named_root);

    ellAdd(pDrvEllList, &p->node);

    return 0;
}



// prepare iocsh commnand
static const iocshArg initArg0 = {"port name",      iocshArgString};
static const iocshArg initArg1 = {"register path",  iocshArgString};
static const iocshArg initArg2 = {"named_root",     iocshArgString};
static const iocshArg * const initArgs[] = { &initArg0,
                                             &initArg1,
                                             &initArg2 };
static const iocshFuncDef initFuncDef = {"llrfHlsAsynDriverConfigure", 3, initArgs};
static void  initCallFunc(const iocshArgBuf *args)
{
    llrfHlsAsynDriverConfigure(args[0].sval,     /* port name */
                               args[1].sval,     /* register path */
                               (args[2].sval && strlen(args[2].sval))?args[2].sval: NULL /* named_root */ );
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

    epicsAtExit3((epicsExitFunc) stopPollingThread, (void *) epicsStrDup(name), epicsStrDup(name));

    return 0;
}


} /* end of extern C */
