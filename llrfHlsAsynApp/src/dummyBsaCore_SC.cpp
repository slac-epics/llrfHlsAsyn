#include <BsaApi.h>

// when building application for SC mode only without BsaCore,
// these dummy will make compiler and linker are happy


BsaChannel Bsa_CreateChannel(const char *id)
{
    return NULL;
}

int BSA_StoreData( BsaChannel      bsaChannel,
                   epicsTimeStamp  timeStamp,
                   double          value,
                   BsaStat         status,
                   BsaSevr         severity
) { return 0; }

int BSA_ConfigSetAllPriorities(unsigned val) { return 0; }



