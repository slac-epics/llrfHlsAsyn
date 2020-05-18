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
}

llrfHlsAsynDriver::~llrfHlsAsynDriver() {}



void llrfHlsAsynDriver::ParameterSetup(void)
{
    char param_name[80];

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

    sprintf(param_name, AVG_WINDOW_STR);             createParam(param_name, asynParamFloat64Array, &p_avg_window);

    for(int i = 0; i < NUM_CH; i++) {    // for loop for number of channels
        sprintf(param_name, REF_WEIGHT_STR, i); createParam(param_name, asynParamFloat64, &(p_ref_weight_ch[i]));
        sprintf(param_name, FB_WEIGHT_STR,  i); createParam(param_name, asynParamFloat64, &(p_fb_weight_ch[i]));
        sprintf(param_name, P_OFFSET_STR,   i); createParam(param_name, asynParamFloat64, &(p_p_offset_ch[i]));

        sprintf(param_name, P_CH_STR,       i); createParam(param_name, asynParamFloat64, &(p_p_ch[i]));
        sprintf(param_name, A_CH_STR,       i); createParam(param_name, asynParamFloat64, &(p_a_ch[i]));

        sprintf(param_name, I_WF_STR,       i); createParam(param_name, asynParamFloat64Array, &(p_i_wf_ch[i]));
        sprintf(param_name, Q_WF_STR,       i); createParam(param_name, asynParamFloat64Array, &(p_q_wf_ch[i]));
       
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
