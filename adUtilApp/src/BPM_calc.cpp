/*
 * BPM_calc.cpp
 *
 * Asyn driver for callbacks to standard asyn array interfaces for NDArray drivers.
 * This is commonly used for EPICS waveform records.
 *
 * Author: Mark Rivers
 *
 * Created April 25, 2008
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

class BPM_calc : public NDPluginDriver {
public:
    BPM_calc(const char *portName, int queueSize, int blockingCallbacks,
             const char *NDArrayPort, int NDArrayAddr, int maxBuffers,
             size_t maxMemory, int priority, int stackSize);
    /* These methods override the virtual methods in the base class */
    void processCallbacks(NDArray *pArray);
protected:
    int BPM_calcGeometry;
    int BPM_scaleX;
    int BPM_scaleY;
    int BPM_scaleI;
    #define FIRST_BPM_CALC_PARAM BPM_calcGeometry
    #define LAST_BPM_CALC_PARAM BPM_scaleI
};
#define NUM_BPM_CALC_PARAMS ((int)(&LAST_BPM_CALC_PARAM - &FIRST_BPM_CALC_PARAM + 1))
static const char *driverName="BPM_calc";

/** This callback function takes 4 diode inputs waveforms, and calculates
X Y and intensity from these. It expects xdim=4 and ydim=number of samples.
It expects asynFloat64 NDArray input

Inputs (X index) in Geometry=0 (Slit):
<pre>
            |A |
            |__|
        __        __
  X+     D|  _   |B
        __|  /|  |__
            /__
           /|C |
          / |  |
       BEAM
       /
</pre>
* A = index 0 = Y+ Input
* B = index 1 = X- Input
* C = index 2 = Y- Input
* D = index 3 = X+ Input

Inputs (X index) in Geometry=1 (QBPM):
<pre>
            ---
          /0 | 1\
   X+    |---#---|
          \3/| 2/
           /---
          /
       BEAM
       /
</pre>
* A = index 0 = Top Left Input
* B = index 1 = Top Right Input
* C = index 2 = Bottom Right Input
* D = index 3 = Bottom Left Input

Outputs (X index):
* index 0 = X Position Waveform
* index 1 = Y Position Waveform
* index 2 = Intensity Waveform
**/
void BPM_calc::processCallbacks(NDArray *pArray)
{
    /* This function calls back any registered clients on the standard
       asyn array interfaces with the data in our private buffer.
       It is called with the mutex already locked.
    */
	const char *functionName = "processCallbacks";
    unsigned int i;
    int geometry;
    double scaleX;
    double scaleY;
    double scaleI;
    size_t dims[2];
    NDArrayInfo_t arrayInfo;

    /* Call the base class method */
    NDPluginDriver::processCallbacks(pArray);

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

 	/* Allocate 3 x n_elements NDFloat64 NDArray */
    pArray->getInfo(&arrayInfo);
    dims[0] = 3;
    dims[1] = arrayInfo.ySize;
    this->pArrays[0] = this->pNDArrayPool->alloc(2, dims, (NDDataType_t)NDFloat64,
                                           0, NULL);
    if (this->pArrays[0] == NULL) {
    	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
    			"%s:%s: Cannot allocate pProcessed array\n",
    			driverName, functionName);
    	return;
    }
	
	/* Save the dims of this new array */
	NDDimension_t dim0 = this->pArrays[0]->dims[0], dim1 = this->pArrays[0]->dims[1];    
	
    /* Copy everything except the data, e.g. uniqueId and timeStamp, attributes. */
    this->pNDArrayPool->copy(pArray, this->pArrays[0], 0);	
    
    /* That replaced the dimensions in the output array, need to fix. */
    this->pArrays[0]->ndims = 2;
    this->pArrays[0]->dims[0] = dim0;
    this->pArrays[0]->dims[1] = dim1;
    
    /* Get any additional attributes for this new array */
	this->getAttributes(this->pArrays[0]->pAttributeList);

    /* Get the geometry */
    getIntegerParam(BPM_calcGeometry, &geometry);

    /* This function is called with the lock taken, and it must be set
       when we exit. The following code can be exected without the mutex
       because we are not accessing pPvt */
    this->unlock();

    /* Fill in the pointers to each ADC element in the input and output waveforms.
     * Note that the ADC samples are interleaved, so dim0=channel and dim1=sample number */
    double * pAData = (double *)pArray->pData;
    double * pBData = (double *)pArray->pData + 1;
    double * pCData = (double *)pArray->pData + 2;
    double * pDData = (double *)pArray->pData + 3;
    double * pXData = (double *)this->pArrays[0]->pData;
    double * pYData = (double *)this->pArrays[0]->pData + 1;
    double * pIData = (double *)this->pArrays[0]->pData + 2;

    getDoubleParam(BPM_scaleX, &scaleX);
    getDoubleParam(BPM_scaleY, &scaleY);
    getDoubleParam(BPM_scaleI, &scaleI);

    /* Calculate output and put in new NDArray */
    for (i=0; i<arrayInfo.ySize; i++) {
        *pIData = (*pAData + *pBData + *pCData + *pDData) * scaleI;
    	if (geometry == 0) {
    		// slits
    		double div = *pAData + *pCData;
    		if (div < 0.01 && div > -0.01) div = 100; // No signal
    		*pXData = (*pAData - *pCData) * scaleX / div;
    		div = *pDData + *pBData;
    		if (div < 0.01 && div > -0.01) div = 100; // No signal
    		*pYData = (*pDData - *pBData) * scaleY / div;
    	} else {
    		// QBPM
    		double div = *pAData + *pBData + *pCData + *pDData;
    		if (div < 0.01 && div > -0.01) div = 100; // No signal
    		*pXData = ((*pAData + *pDData) - (*pBData + *pCData)) * scaleX / div;
    		*pYData = ((*pAData + *pBData) - (*pCData + *pDData)) * scaleY / div;
    	}
        pXData += 3; pYData += 3; pIData += 3;
        pAData += 4; pBData += 4; pCData += 4; pDData += 4;
    }

    /* Call any clients who have registered for NDArray callbacks */
    this->doCallbacksGenericPointer(this->pArrays[0], NDArrayData, 0);

    /* We must enter the loop and exit with the mutex locked */
    this->lock();
    this->callParamCallbacks();
}

/* Constructor for BPM_calc; all parameters are simply passed to
   NDPluginDriver::NDPluginDriver. This plugin cannot block
   (ASYN_CANBLOCK=0) and is not multi-device (ASYN_MULTIDEVICE=0).
   It allocates a maximum of 2 NDArray buffers for internal use.
   \param[in] portName The name of the asyn port driver to be created.
   \param[in] queueSize The number of NDArrays that the input queue for
                        this plugin can hold when
                        NDPluginDriverBlockingCallbacks=0.  Larger queues
                        can decrease the number of dropped arrays,
                        at the expense of more NDArray buffers being
                        allocated from the underlying driver's NDArrayPool.
   \param[in] blockingCallbacks Initial setting for the
                                NDPluginDriverBlockingCallbacks flag.
                                0=callbacks are queued and executed by the
                                callback thread; 1 callbacks execute in
                                the thread of the driver doing the callbacks.
   \param[in] NDArrayPort Name of asyn port driver for initial source
                          of NDArray callbacks.
   \param[in] NDArrayAddr asyn port driver address for initial
                          source of NDArray callbacks.
   \param[in] maxMemory The maximum amount of memory that the NDArrayPool
                        for this driver is allowed to allocate. Set this
                        to -1 to allow an unlimited amount of memory.
   \param[in] priority The thread priority for the asyn port driver
                       thread if ASYN_CANBLOCK is set in asynFlags.
   \param[in] stackSize The stack size for the asyn port driver
                        thread if ASYN_CANBLOCK is set in asynFlags.
*/

BPM_calc::BPM_calc(const char *portName, int queueSize, int blockingCallbacks,
                   const char *NDArrayPort, int NDArrayAddr, int maxBuffers,
                   size_t maxMemory, int priority, int stackSize)
    /* Invoke the base class constructor */
    : NDPluginDriver(portName, queueSize, blockingCallbacks, NDArrayPort,
                     NDArrayAddr, 1, NUM_BPM_CALC_PARAMS, maxBuffers, maxMemory,
                     asynGenericPointerMask,
                     asynGenericPointerMask,
                     /* asynFlags is set to 0, because this plugin cannot
                        block and is not multi-device. It does autoconnect */
                     0, 1, priority, stackSize)
{
	/* Add parameters */
    createParam("GEOMETRY", asynParamInt32, &BPM_calcGeometry);
    createParam("BPM_SCALE_X", asynParamFloat64, &BPM_scaleX);
    createParam("BPM_SCALE_Y", asynParamFloat64, &BPM_scaleY);
    createParam("BPM_SCALE_I", asynParamFloat64, &BPM_scaleI);
    /* Set the plugin type string */
    setStringParam(NDPluginDriverPluginType, "BPM_calc");
    /* Set default scales */
    setDoubleParam(BPM_scaleX, 1.0);
    setDoubleParam(BPM_scaleY, 1.0);
    setDoubleParam(BPM_scaleI, 1.0);
    /* Try to connect to the NDArray port */
    connectToArrayPort();
}

/** Configuration command */
extern "C" int BPM_calcConfigure(const char *portName, int queueSize,
                                 int blockingCallbacks, const char *NDArrayPort,
                                 int NDArrayAddr, int maxBuffers,
                                 size_t maxMemory,int priority, int stackSize)
{
    new BPM_calc(portName, queueSize, blockingCallbacks, NDArrayPort,
                 NDArrayAddr, maxBuffers, maxMemory, priority, stackSize);
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
static const iocshFuncDef initFuncDef = {"BPM_calcConfigure", 9, initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    BPM_calcConfigure(args[0].sval, args[1].ival, args[2].ival,
                      args[3].sval, args[4].ival, args[5].ival,
                      args[6].ival, args[7].ival, args[8].ival);
}

extern "C" void BPM_calc_register(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
}

extern "C" {
    epicsExportRegistrar(BPM_calc_register);
}
