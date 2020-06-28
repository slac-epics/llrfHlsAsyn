#ifndef  _RTM_INTERFACE_H
#define  _RTM_INTERFACE_H

#include <epicsTime.h>

void registerRtm2llrfHlsAsyn(void *pRtm);
void callRtmProcessing(epicsTimeStamp time, void *pRtm);

#endif /* _RTM_INTERFACE_H */
