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

#include "cableCalAsyn.h"

#define  POLL_RATE        10


static bool         keep_stay_in_loop = true;
static epicsEventId shutdownEvent;


static ELLLIST *pDrvEllList = NULL;

typedef struct {
    ELLNODE                node;
    char                   *named_root;
    char                   *port;
    char                   *regPath;
    cableCalAsynDriver     *pCableCalAsyn;
} pDrvList_t;


static void init_drvList(void)
{
    if(!pDrvEllList) {
        pDrvEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "cableCalAsyn driver: init_drvList()");
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



cableCalAsynDriver::cableCalAsynDriver(void *pDrv, const char *portName, const char *pathString, const char *named_root)
    : asynPortDriver(portName,
                     1, /* number of elements of this device */
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
                     NUM_CABLECAL_DET_PARAMS, /* number of asyn params of be cleared for each device */
#endif  /* asyn version check, under 4.32 */
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat64ArrayMask, /* Interface mask */
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynEnumMask    | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat64ArrayMask,  /* Interrupt mask */
                     1, /* asynFlags.  This driver does block and it is not multi-device, so flag is 1 */
                     1, /* Autoconnect */
                     0, /* Default priority */
                     0) /* Default stack size*/
{
    Path       p_root;
    Path       p_calDsp;
    port       = epicsStrDup(portName);
    path       = epicsStrDup(pathString);
    this->pDrv = pDrv;

    try {
        p_root = (named_root && strlen(named_root))? cpswGetNamedRoot(named_root): cpswGetRoot();
        p_calDsp = p_root->findByName(pathString);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s, file %s, line %d\n", e.getInfo().c_str(), __FILE__, __LINE__);
        throw e;
    }

    calDsp = IcalDspFw::create(p_calDsp);

    ParameterSetup();
}

cableCalAsynDriver::~cableCalAsynDriver() {}

asynStatus cableCalAsynDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int        function      = pasynUser->reason;
    asynStatus status        = asynSuccess;
    const char *functionName = "writeInt32";

    status = (asynStatus) setIntegerParam(function, value);

    if(function == p_cal_dac_enable) {         // select DAC output {0: conventional (LLRF), 1: calibration pulse
        calDsp->dacEnable(value?true:false);
    } else
    if(function == p_slow_dac_select) {
        calDsp->setSlowDACSel((unsigned) value, slowDac_string);
        setStringParam(p_slow_dac_string, slowDac_string);
    }

    return  status;
}


asynStatus cableCalAsynDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int        function      = pasynUser->reason;
    asynStatus status        = asynSuccess;
    const char *functionName = "wrtieFloat64";

    status = (asynStatus) setDoubleParam(function, value);

    for(int p = 0; p < NUM_CAL_PULSE; p++) {
        if(function == p_cal_freq_offset[p]) {
            uint32_t raw_freq;
            calDsp->setCalFreqOffset(p, value, &raw_freq);

            setIntegerParam(p_raw_freq[p], raw_freq);
            break;
        }
    }

    for(int c = 0; c < NUM_CAL_ADC; c++) {
        if(function == p_cal_loop_delay[c]) {
            uint16_t delay_tick;
            calDsp->setLoopDelay(c, value, &delay_tick);

            setIntegerParam(p_raw_loop_delay[c], delay_tick);
            break;
        }
    }

    if(function == p_cal_pulse_seq_delay) {
        uint16_t pulse_delay_tick;
        calDsp->setPulseSequenceDelay(value, &pulse_delay_tick);

        setIntegerParam(p_raw_pulse_seq_delay, pulse_delay_tick);
    } else
    if(function == p_cal_pulse_start) {
        double start = value;
        double end;
        uint16_t  start_tick, end_tick;

        getDoubleParam(p_cal_pulse_end, &end);
        calDsp->setCalPulse(start, end, &start_tick, &end_tick);

        setIntegerParam(p_raw_pulse_start, start_tick);
        setIntegerParam(p_raw_pulse_end, end_tick);
    } else
    if(function == p_cal_pulse_end) {
        double start;
        double end   = value;
        uint16_t start_tick, end_tick;

        getDoubleParam(p_cal_pulse_start, &start);
        calDsp->setCalPulse(start, end, &start_tick, &end_tick);

        setIntegerParam(p_raw_pulse_start, start_tick);
        setIntegerParam(p_raw_pulse_end, end_tick);
    } else
    if(function == p_cal_window_start) {
        double start = value;
        double end;
        uint16_t start_tick, end_tick;

        getDoubleParam(p_cal_window_end, &end);
        calDsp->setCalWindow(start, end, &start_tick, &end_tick);

        setIntegerParam(p_raw_window_start, start_tick);
        setIntegerParam(p_raw_window_end,   end_tick);
    } else
    if(function == p_cal_window_end) {
        double start;
        double end   = value;
        uint16_t start_tick, end_tick;

        getDoubleParam(p_cal_window_start, &start);
        calDsp->setCalWindow(start, end, &start_tick, &end_tick);

        setIntegerParam(p_raw_window_start, start_tick);
        setIntegerParam(p_raw_window_end,   end_tick);
    }

    callParamCallbacks();

    return status;
}

void cableCalAsynDriver::poll(void)
{
    for(int p = 0; p < NUM_CAL_PULSE; p++) {
        for(int c = 0; c < NUM_CAL_ADC; c++) {
            double phase, ampl;
	    double cordic_phase, cordic_ampl;

            int32_t  raw_phase, raw_ampl;
	    int32_t  raw_cordic_phase, raw_cordic_ampl;

            calDsp->calPhase(p, c, &phase, &raw_phase);
            calDsp->calAmpl(p, c, &ampl, &raw_ampl);

	    calDsp->cordicPhase(&cordic_phase, &raw_cordic_phase);
	    calDsp->cordicAmpl(&cordic_ampl, &raw_cordic_ampl);

            setDoubleParam(p_cal_phase[p][c], phase);
            setDoubleParam(p_cal_ampl[p][c],  ampl);

            setIntegerParam(p_raw_phase[p][c], raw_phase);
            setIntegerParam(p_raw_ampl[p][c],  raw_ampl);
            
            setDoubleParam(p_cordic_phase, cordic_phase);
            setDoubleParam(p_cordic_ampl,  cordic_ampl);
            
            setIntegerParam(p_raw_cordic_phase,  raw_cordic_phase);
            setIntegerParam(p_raw_cordic_ampl,   raw_cordic_ampl);
        }
    }
    callParamCallbacks();
}

void cableCalAsynDriver::report(int interest)
{
    printf("cableCalAsynDriver: port (%s), register path (%s), driver instance (%p)\n", port, path, pDrv);

    calDsp->report(interest);
}


void cableCalAsynDriver::ParameterSetup(void)
{
    char param_name[80];
    // parameters for engineering values
    sprintf(param_name, CAL_PULSE_START_STR);   createParam(param_name, asynParamFloat64, &p_cal_pulse_start);
    sprintf(param_name, CAL_PULSE_END_STR);     createParam(param_name, asynParamFloat64, &p_cal_pulse_end);
    sprintf(param_name, CAL_WINDOW_START_STR);  createParam(param_name, asynParamFloat64, &p_cal_window_start);
    sprintf(param_name, CAL_WINDOW_END_STR);    createParam(param_name, asynParamFloat64, &p_cal_window_end);
    sprintf(param_name, CAL_DAC_ENABLE_STR);    createParam(param_name, asynParamInt32,   &p_cal_dac_enable);
    sprintf(param_name, CAL_PULSE_SEQ_DELAY_STR); createParam(param_name, asynParamFloat64, &p_cal_pulse_seq_delay);

    sprintf(param_name, SLOWDAC_SEL_STR);       createParam(param_name, asynParamInt32,   &p_slow_dac_select);
    sprintf(param_name, SLOWDAC_STRING_STR);    createParam(param_name, asynParamOctet,   &p_slow_dac_string);

    // parameters for raw values
    sprintf(param_name, RAW_PULSE_START_STR);   createParam(param_name, asynParamInt32,   &p_raw_pulse_start);
    sprintf(param_name, RAW_PULSE_END_STR);     createParam(param_name, asynParamInt32,   &p_raw_pulse_end);
    sprintf(param_name, RAW_WINDOW_START_STR);  createParam(param_name, asynParamInt32,   &p_raw_window_start);
    sprintf(param_name, RAW_WINDOW_END_STR);    createParam(param_name, asynParamInt32,   &p_raw_window_end);
    sprintf(param_name, RAW_PULSE_SEQ_DELAY_STR); createParam(param_name, asynParamInt32,  &p_raw_pulse_seq_delay);
    
    sprintf(param_name, CORDIC_PHASE_STR);      createParam(param_name, asynParamFloat64, &p_cordic_phase);
    sprintf(param_name, CORDIC_AMPL_STR);       createParam(param_name, asynParamFloat64, &p_cordic_ampl);
    sprintf(param_name, RAW_CORDIC_PHASE_STR);  createParam(param_name, asynParamInt32,   &p_raw_cordic_phase);
    sprintf(param_name, RAW_CORDIC_AMPL_STR);   createParam(param_name, asynParamInt32,   &p_raw_cordic_ampl);

    for(int p = 0; p < NUM_CAL_PULSE; p++) {
        for(int c = 0; c < NUM_CAL_ADC; c++) {
            // parameters for engineering values
            sprintf(param_name, CAL_PHASE_STR, p, c); createParam(param_name, asynParamFloat64, &p_cal_phase[p][c]);
            sprintf(param_name, CAL_AMPL_STR,  p, c); createParam(param_name, asynParamFloat64, &p_cal_ampl[p][c]);
            // parameters for raw values
            sprintf(param_name, RAW_PHASE_STR, p, c); createParam(param_name, asynParamInt32,   &p_raw_phase[p][c]);
            sprintf(param_name, RAW_AMPL_STR,  p, c); createParam(param_name, asynParamInt32,   &p_raw_ampl[p][c]);
        }
        // parameters for engineering values
        sprintf(param_name, CAL_FREQ_OFFSET_STR, p);  createParam(param_name, asynParamFloat64, &p_cal_freq_offset[p]);
        // parameters for raw vlaues
        sprintf(param_name, RAW_FREQ_STR,        p);  createParam(param_name, asynParamInt32,   &p_raw_freq[p]);
    }

    for(int c = 0; c < NUM_CAL_ADC; c++) {
        // parameters for engineering values
        sprintf(param_name, CAL_LOOP_DELAY_STR, c);   createParam(param_name, asynParamFloat64, &p_cal_loop_delay[c]);
        // parameters for raw values
        sprintf(param_name, RAW_LOOP_DELAY_STR, c);   createParam(param_name, asynParamInt32,   &p_raw_loop_delay[c]);
    }


}


extern "C" {

// driver configuration, C wrapper
int cableCalAsynDriverConfigure(const char *portName, const char *regPathString, const char *named_root)
{

    init_drvList();

    pDrvList_t *p = find_drvByPort(portName);
    if(p) {
        printf("cableCalAsynDriver found that port name (%s) has been used.\n", portName);
        return 0;
    }

    p  = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "cableCalAsyn driver: cableCalAsynDriverConfigure()");
    p->named_root = (named_root && strlen(named_root))? epicsStrDup(named_root):cpswGetRootName();
    p->port       = epicsStrDup(portName);
    p->regPath    = epicsStrDup(regPathString);
    p->pCableCalAsyn = new cableCalAsynDriver((void *)p, (const char *) p->port, (const char *) p->regPath, (const char *) p->named_root);

    ellAdd(pDrvEllList, &p->node);

    return 0;
}

// prepare iocsh command
static const iocshArg initArg0 = {"port name",         iocshArgString};
static const iocshArg initArg1 = {"register path",     iocshArgString};
static const iocshArg initArg2 = {"named root",        iocshArgString};
static const iocshArg * const initArgs[] = { &initArg0,
                                             &initArg1,
                                             &initArg2 };
static const iocshFuncDef initFuncDef = {"cableCalAsynDriverConfigure", 3, initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    cableCalAsynDriverConfigure(args[0].sval,     /* port name     */
                                args[1].sval,     /* register path */
                                args[2].sval      /* named root    */ );
}

static void cableCalAsynDriverRegister(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(cableCalAsynDriverRegister);

// polling thread function
static int cableCalAsynDriverPoll(void)
{
    while(keep_stay_in_loop) {
        pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
        while(p) {
            if(p->pCableCalAsyn) p->pCableCalAsyn->poll();
            p = (pDrvList_t *) ellNext(&p->node);
        }

        epicsThreadSleep(1./double(POLL_RATE));
    }

    epicsEventSignal(shutdownEvent);

    return 0;
}

// stopping pulling thread for exit hook
static void stopPollingThread(void *p)
{
    keep_stay_in_loop = false;
    epicsEventWait(shutdownEvent);
    epicsPrintf("cableCalAsynDriver: stop polling thread (%s)\n", (char *) p);
}


// EPICS driver support for cableCalAsynDriver

static int cableCalAsynDriverReport(int interest);
static int cableCalAsynDriverInitialize(void);

static struct drvet cableCalAsynDriver = {
    2,
    (DRVSUPFUN) cableCalAsynDriverReport,
    (DRVSUPFUN) cableCalAsynDriverInitialize
};

epicsExportAddress(drvet, cableCalAsynDriver);


static int cableCalAsynDriverReport(int interest)
{

    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
    while(p) {
        if(p->pCableCalAsyn) p->pCableCalAsyn->report(interest);
        p = (pDrvList_t *) ellNext(&p->node);
    }


   return 0;
}

static int cableCalAsynDriverInitialize(void)
{

    init_drvList();

    if(!pDrvEllList) {
        printf("cableCalAsynDriver never been configured\n");
        return 0;
    }

    keep_stay_in_loop = true;
    shutdownEvent     = epicsEventMustCreate(epicsEventEmpty);
    const char *name  = "cableCalAsynPoll";

    epicsThreadCreate(name, epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) cableCalAsynDriverPoll, 0);


    epicsAtExit3((epicsExitFunc) stopPollingThread, (void *) epicsStrDup(name), epicsStrDup(name));

    return 0;
}


}     /* extern C */





