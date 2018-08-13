/*
 * BL22B_FFT_Integration.cpp
 *
 *  Created on: 17 Apr 2014
 *      Author: qvr31998
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsMessageQueue.h>
#include <cantProceed.h>
#include <iocsh.h>
#include "NDArray.h"
#include <epicsExport.h>
#include <epicsTypes.h>
#include <asynStandardInterfaces.h>
#include "NDPluginDriver.h"
#include <algorithm>

class FFT_Integration : public NDPluginDriver {
public:
    FFT_Integration(const char *portName, int queueSize, int blockingCallbacks,
             const char *NDArrayPort, int NDArrayAddr, int maxBuffers,
             size_t maxMemory, int priority, int stackSize);
    /* These methods override the virtual methods in the base class */
    void processCallbacks(NDArray *pArray);
};
#define NUM_FFT_INTEGRATION_PARAMS 0
static const char *driverName="FFT_Integration";

void FFT_Integration::processCallbacks(NDArray *pArray)
{
	// Set name and declare local variables
	const char *functionName = "processCallbacks";
	size_t dims[2];
	NDArrayInfo_t arrayInfo;

	/* Call the base class method */
	NDPluginDriver::beginProcessCallbacks(pArray);

	/* Make sure we have a float64 NDArray */
	if (pArray->dataType != NDFloat64) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: unsupported array structure. Need NDFloat64 array\n",
				driverName, functionName);
		return;
	}

	/* We always keep the last array so read() can use it.
	 * Release previous one. Reserve new one below. */
	if (this->pArrays[0]) {
		this->pArrays[0]->release();
		this->pArrays[0] = NULL;
	}

	/* Allocate a new array for our output data */
	pArray->getInfo(&arrayInfo);
	//dims[0]=arrayInfo.xSize;
	// Hard-code this to 3 for now; may generalise to n-dim if needed
	dims[1]=3;
	dims[0]=arrayInfo.xSize;
	this->pArrays[0] = this->pNDArrayPool->alloc(2, dims, (NDDataType_t)NDFloat64, 0, NULL);
    if (this->pArrays[0] == NULL) {
    	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
    			"%s:%s: Cannot allocate pProcessed array\n",
    			driverName, functionName);
    	return;
    }

	/* Copy all metadata from input to output array */
    this->pNDArrayPool->copy(pArray, this->pArrays[0], 0);

    /* Append any driver-specific attributes to the output array */
    this->getAttributes(this->pArrays[0]->pAttributeList);

    this->unlock();

	/* Perform the actual integration */

    // Loop over yDims to compute the integral for each point
    for (size_t channelNum = 0; channelNum < 3; channelNum++) {
    	double *pXin = (double *)pArray->pData + channelNum*dims[0];
    	double *pXout = (double *)this->pArrays[0]->pData + channelNum*dims[0];
    	*pXout = *pXin;
    	for (size_t i = 1; i < dims[0]; i++) {
    		*(pXout+1) = *pXout + *(pXin+1);
    		pXin += 1;
    		pXout += 1;
    	}
    }

	/* Call param callback for the output array (needed so that other plugins can use this one as
	 * an input source) */
    this->doCallbacksGenericPointer(this->pArrays[0], NDArrayData, 0);

    /* Must return with lock held */
    this->lock();
    this->callParamCallbacks();
}


FFT_Integration::FFT_Integration(const char *portName, int queueSize, int blockingCallbacks,
                   const char *NDArrayPort, int NDArrayAddr, int maxBuffers,
                   size_t maxMemory, int priority, int stackSize)
    /* Invoke the base class constructor */
    : NDPluginDriver(portName, queueSize, blockingCallbacks, NDArrayPort,
                     NDArrayAddr, 1, NUM_FFT_INTEGRATION_PARAMS, maxBuffers, maxMemory,
                     asynGenericPointerMask,
                     asynGenericPointerMask,
                     /* asynFlags is set to 0, because this plugin cannot
                        block and is not multi-device. It does autoconnect */
                     0, 1, priority, stackSize)
{
	/* Add parameters */
    /* Set the plugin type string */
    setStringParam(NDPluginDriverPluginType, "FFT_Integration");
    /* Try to connect to the NDArray port */
    connectToArrayPort();
}

/** Configuration command */
extern "C" int FFT_IntegrationConfigure(const char *portName, int queueSize,
                                 int blockingCallbacks, const char *NDArrayPort,
                                 int NDArrayAddr, int maxBuffers,
                                 size_t maxMemory,int priority, int stackSize)
{
    FFT_Integration *pPlugin = new FFT_Integration(portName, queueSize, blockingCallbacks, NDArrayPort,
                                                   NDArrayAddr, maxBuffers, maxMemory, priority, stackSize);
    pPlugin->start();
    return(asynSuccess);
}


/* EPICS iocsh shell commands */
static const iocshArg initArg0 = {"portName", iocshArgString};
static const iocshArg initArg1 = {"frame queue size", iocshArgInt};
static const iocshArg initArg2 = {"blocking callbacks", iocshArgInt};
static const iocshArg initArg3 = {"NDArrayPort", iocshArgString};
static const iocshArg initArg4 = {"NDArrayAddr", iocshArgInt};
static const iocshArg initArg5 = {"maxBuffers", iocshArgInt};
static const iocshArg initArg6 = {"maxMemory", iocshArgInt};
static const iocshArg initArg7 = {"priority", iocshArgInt};
static const iocshArg initArg8 = {"stackSize", iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0, &initArg1, &initArg2,
                                            &initArg3, &initArg4, &initArg5,
                                            &initArg6, &initArg7, &initArg8};
static const iocshFuncDef initFuncDef = {"FFT_IntegrationConfigure", 9, initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    FFT_IntegrationConfigure(args[0].sval, args[1].ival, args[2].ival,
                      args[3].sval, args[4].ival, args[5].ival,
                      args[6].ival, args[7].ival, args[8].ival);
}

extern "C" void FFT_Integration_register(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
}

extern "C" {
    epicsExportRegistrar(FFT_Integration_register);
}

