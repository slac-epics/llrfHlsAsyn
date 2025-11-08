#ifndef     _CABLECALASYN_H
#define     _CABLECALASYN_H

#include <asynPortDriver.h>
#include <epicsEvent.h>
#include <epicsTypes.h>
#include <epicsTime.h>

#include <cpsw_api_user.h>
#include <calDspFw.h>
#include <vector>
#include <string>
#include <dlfcn.h>

#include <stdio.h>
#include <sstream>
#include <fstream>




class cableCalAsynDriver
    : asynPortDriver {
    public:
        cableCalAsynDriver(void *pDrv, const char *portName, const char *pathString, const char *named_root = NULL);
        ~cableCalAsynDriver();
    private:


    protected:
#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int firstLlrfHlsParam;
#define FIRST_CABLECAL_PARAM   firstCableCalParam
#endif /* ASYN VERSION CHECK under 4.32 */



#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)      
        int lastLlrfHlsParam;
#define LAST_CABLECAL_PARAM   lastCableCalParam
#endif /* asyn version check, under 4.32 */
};


#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
#define NUM_CABLECAL_DET_PARAMS ((int)(&LAST_CABLECAL_PARAM - &FIRST_CABLECAL_PARAM-1))
#endif /* asyn version check, under 4.32 */


#endif   /* _CABLECALASYN_H    */
