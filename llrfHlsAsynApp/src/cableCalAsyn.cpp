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


static ELLLIST *pDrvEllList = NULL;

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
}

cableCalAsynDriver::~cableCalAsynDriver() {}

void cableCalAsynDriver::ParameterSetup(void)
{
    char param_name[80];

    sprintf(param_name, CAL_PULSE_START_STR);   createParam(param_name, asynParamFloat64, &p_cal_pulse_start);
    sprintf(param_name, CAL_PULSE_END_STR);     createParam(param_name, asynParamFloat64, &p_cal_pulse_end);
    sprintf(param_name, CAL_WINDOW_START_STR);  createParam(param_name, asynParamFloat64, &p_cal_window_start);
    sprintf(param_name, CAL_WINDOW_END_STR);    createParam(param_name, asynParamFloat64, &p_cal_window_end);
    sprintf(param_name, CAL_DAC_ENABLE_STR);    createParam(param_name, asynParamInt32,   &p_cal_dac_enable);

    for(int p = 0; p < NUM_CAL_PULSE; p++) {
        for(int c = 0; c < NUM_CAL_ADC; c++) {
            sprintf(param_name, CAL_PHASE_STR, p, c); createParam(param_name, asynParamFloat64, &p_cal_phase[p][c]);
            sprintf(param_name, CAL_AMPL_STR,  p, c); createParam(param_name, asynParamFloat64, &p_cal_ampl[p][c]);
        }
        sprintf(param_name, CAL_FREQ_OFFSET_STR, p);  createParam(param_name, asynParamFloat64, &p_cal_freq_offset[p]);
    }

    for(int c = 0; c < NUM_CAL_ADC; c++) {
        sprintf(param_name, CAL_LOOP_DELAY_STR, c);   createParam(param_name, asynParamFloat64, &p_cal_loop_delay[c]);
    }


}


extern "C" {

// driver configuration, C wrapper
int cableCalAsynDriverConfigure(const char *portName, const char *regPathString, const char *named_root)
{

    return 0;
}

// prepare iocsh command
static const iocshArg initArg0 = {"port name",         iocshArgString};
static const iocshArg initArg1 = {"register path",     iocshArgString};
static const iocshArg initArg2 = {"named root",        iocshArgString};
static const iocshArg * const initArgs[] = { &initArg0,
                                             &initArg1,
                                             &initArg2 };
static const iocshFuncDef initFuncDef = {"cableCalAsynDriverConfigure", sizeof(initArgs) / sizeof(iocshArg), initArgs};
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


   return 0;
}

static int cableCalAsynDriverInitialize(void)
{



    return 0;
}


}     /* extern C */





