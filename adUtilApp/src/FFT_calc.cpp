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
#include "window.h"
#include "fftpack.h"
#include <math.h>

/* Converts NDArray callback data into standard asyn arrays
 (asynInt8Array, asynInt16Array, asynInt32Array, asynFloat32Array or
 asynFloat64Array); normally used for putting NDArray data in EPICS
 waveform records. It handles the data type conversion if the NDArray
 data type differs from the data type of the asyn interface. It
 flattens the NDArrays to a single dimension because asyn and EPICS
 do not support multi-dimensional arrays. */
class FFT_calc: public NDPluginDriver {
public:
	FFT_calc(const char *portName, int queueSize, int blockingCallbacks,
			const char *NDArrayPort, int NDArrayAddr, int maxBuffers,
			size_t maxMemory, int priority, int stackSize);
	/* These methods override the virtual methods in the base class */
	void processCallbacks(NDArray *pInput);
protected:
	int FFT_calcData;
#define FIRST_FFT_CALC_PARAM FFT_calcData
#define LAST_FFT_CALC_PARAM FFT_calcData
	int FFT_calcWindow;
	int FFT_calcWindowParam;
private:
	enum windowType {
		RECT, HANN, HAMMING, BLACKMAN, BLACKMANHARRIS, KAISERBESSEL, GAUSSIAN
	};
	int cachedWindow;
	double cachedWindowParam;
	NDArray *pWindow, *pWorkspace;
};
#define NUM_FFT_CALC_PARAMS ((int)(&LAST_FFT_CALC_PARAM - &FIRST_FFT_CALC_PARAM + 3))
static const char *driverName = "FFT_calc";

void FFT_calc::processCallbacks(NDArray *pInput) {
	const char *functionName = "processCallbacks";
	int window;
	unsigned int chan, i;
	double *dInput, *dOutput, *dWindow, *dWorkspace, *dScratch;
	double windowParam;
	size_t dims[2];
	NDArrayInfo_t arrayInfo;

	/* Call the base class processCallbacks */
	NDPluginDriver::processCallbacks(pInput);
	pInput->getInfo(&arrayInfo);

	/* Make sure we have a float64 NDArray */
	if (pInput->dataType != NDFloat64) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: unsupported array structure. Need NDFloat64 array\n", driverName, functionName);
		return;
	}

    /* We always keep the last array so read() can use it.
     * Release previous one. Reserve new one below. */
    if (this->pArrays[0]) {
        this->pArrays[0]->release();
        this->pArrays[0] = NULL;
    }

	/* Create a window waveform if different from cached */
	getIntegerParam(FFT_calcWindow, &window);
	getDoubleParam(FFT_calcWindowParam, &windowParam);
	if (this->pWindow == NULL || this->pWindow->dims[0].size != arrayInfo.ySize
			|| window != this->cachedWindow
			|| windowParam != this->cachedWindowParam) {
		/* Free old window if it exists */
		if (this->pWindow)
			this->pWindow->release();
		this->pWindow = this->pNDArrayPool->alloc(1, &arrayInfo.ySize,
				(NDDataType_t) NDFloat64, 0, NULL);
		if (this->pWindow == NULL) {
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: Cannot allocate pWindow array\n", driverName, functionName);
			return;
		}
		dWindow = (double *) this->pWindow->pData;

		/* fftpack workspace must at least 2n + 15 in length */
		if (this->pWorkspace)
			this->pWorkspace->release();
		size_t worksize = 2 * arrayInfo.ySize + 16;
		this->pWorkspace = this->pNDArrayPool->alloc(1, &worksize,
				(NDDataType_t) NDFloat64, 0, NULL);
		if (this->pWorkspace == NULL) {
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: Cannot allocate pWorkspace array\n", driverName, functionName);
			return;
		}
		dWorkspace = (double *) this->pWorkspace->pData;

		/* Store cached parameters */
		this->cachedWindowParam = windowParam;
		this->cachedWindow = window;

		/* Initialise fftpack workspace */
		rffti(arrayInfo.ySize, dWorkspace);

		/* Create the window */
		switch (window) {
		case RECT:
			WindowRect(dWindow, arrayInfo.ySize);
			break;
		case HANN:
			WindowHann(dWindow, arrayInfo.ySize);
			break;
		case HAMMING:
			WindowHamming(dWindow, arrayInfo.ySize);
			break;
		case BLACKMAN:
			WindowBlackman(dWindow, arrayInfo.ySize,
					windowParam);
			break;
		case BLACKMANHARRIS:
			WindowBlackmanHarris(dWindow,
					arrayInfo.ySize);
			break;
		case KAISERBESSEL:
			WindowKaiserBessel(dWindow, arrayInfo.ySize,
					windowParam);
			break;
		case GAUSSIAN:
			WindowGauss(dWindow, arrayInfo.ySize,
					windowParam);
			break;
		}
	}

	/* Create our output NDArray, same size as our input array, but flipped */
	dims[0] = arrayInfo.ySize / 2; // nsamples / 2
	dims[1] = arrayInfo.xSize; // nchannels
	this->pArrays[0] = this->pNDArrayPool->alloc(2, dims, (NDDataType_t) NDFloat64, 0,
			NULL);
	if (this->pArrays[0] == NULL) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: Cannot allocate pOutput array\n", driverName, functionName);
		return;
	}
	
	/* Save the dims of this new array */
	NDDimension_t dim0 = this->pArrays[0]->dims[0], dim1 = this->pArrays[0]->dims[1];
	
    /* Copy everything except the data, e.g. uniqueId and timeStamp, attributes. */
    this->pNDArrayPool->copy(pInput, this->pArrays[0], 0);	
    
    /* That replaced the dimensions in the output array, need to fix. */
    this->pArrays[0]->ndims = 2;
    this->pArrays[0]->dims[0] = dim0;
    this->pArrays[0]->dims[1] = dim1;
    
    /* Get any additional attributes for this new array */
	this->getAttributes(this->pArrays[0]->pAttributeList);

	/* Give ourselves a bit of memory to created a windowed input */
	NDArray *pScratch = this->pNDArrayPool->alloc(1, &arrayInfo.ySize, (NDDataType_t) NDFloat64, 0,
			NULL);
	if (pScratch == NULL) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: Cannot allocate pScratch array\n", driverName, functionName);
		return;
	}

	/* Unlock for long operations */
	this->unlock();

	/* For each channel in the input array */
	for (chan = 0; chan < arrayInfo.xSize; chan++) {
		/* Step through the input data and multiply it by the window */
		dInput = (double *) pInput->pData;
		/* Pointer to FFT data block and window */
		dWorkspace = (double *) this->pWorkspace->pData;
		dWindow = (double *) this->pWindow->pData;
		dScratch = (double *) pScratch->pData;
		/* Multiply by the window function and scale by the input array size*/
		for (i = 0; i < arrayInfo.ySize; i++) {
			dScratch[i] = dWindow[i] * dInput[i * arrayInfo.xSize + chan] / arrayInfo.ySize;
		}
		/* Calculate the fft of our newly prepared windowed data */
		rfftf(arrayInfo.ySize, dScratch, dWorkspace);
		/* First element is first element of dScratch */
		dOutput = ((double *) this->pArrays[0]->pData) + chan * arrayInfo.ySize / 2;
		dOutput[0] = *dScratch;
		/* Calculate the magnitude for each output element */
		for (i = 1; i < arrayInfo.ySize / 2; i++) {
			const double imag = dScratch[2 * i - 1], real = dScratch[2 * i];
			dOutput[i] = sqrt(imag * imag + real * real);
		}
	}

	/* Output the processed waveform, lock and return */
	pScratch->release();
	this->doCallbacksGenericPointer(this->pArrays[0], NDArrayData, 0);
	this->lock();
	this->callParamCallbacks();
}

/* Constructor for FFT_calc; all parameters are simply passed to
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

FFT_calc::FFT_calc(const char *portName, int queueSize, int blockingCallbacks,
		const char *NDArrayPort, int NDArrayAddr, int maxBuffers,
		size_t maxMemory, int priority, int stackSize)
/* Invoke the base class constructor */
:
		NDPluginDriver(portName, queueSize, blockingCallbacks, NDArrayPort,
				NDArrayAddr, 1, NUM_FFT_CALC_PARAMS, maxBuffers, maxMemory,
				asynGenericPointerMask, asynGenericPointerMask,
				/* asynFlags is set to 0, because this plugin cannot
				 block and is not multi-device. It does autoconnect */
				0, 1, priority, stackSize) {
	/* Initialise local vars */
	pWindow = NULL;
	pWorkspace = NULL;
	cachedWindow = -1;
	cachedWindowParam = -1;
	/* Create parameters */
	createParam("Window", asynParamInt32, &FFT_calcWindow);
	createParam("WindowParam", asynParamFloat64, &FFT_calcWindowParam);
	/* Set the plugin type string */
	setStringParam(NDPluginDriverPluginType, "FFT_calc");
	/* Try to connect to the NDArray port */
	connectToArrayPort();
}

/** Configuration command */
extern "C" int FFT_calcConfigure(const char *portName, int queueSize,
		int blockingCallbacks, const char *NDArrayPort, int NDArrayAddr,
		int maxBuffers, size_t maxMemory, int priority, int stackSize) {
	new FFT_calc(portName, queueSize, blockingCallbacks, NDArrayPort,
			NDArrayAddr, maxBuffers, maxMemory, priority, stackSize);
	return (asynSuccess);
}

/* EPICS iocsh shell commands */
static const iocshArg initArg0 = { "portName", iocshArgString };
static const iocshArg initArg1 = { "frame queue size", iocshArgInt };
static const iocshArg initArg2 = { "blocking callbacks", iocshArgInt };
static const iocshArg initArg3 = { "NDArrayPort", iocshArgString };
static const iocshArg initArg4 = { "NDArrayAddr", iocshArgInt };
static const iocshArg initArg5 = { "maxBuffers", iocshArgInt };
static const iocshArg initArg6 = { "maxMemory", iocshArgInt };
static const iocshArg initArg7 = { "priority", iocshArgInt };
static const iocshArg initArg8 = { "stackSize", iocshArgInt };
static const iocshArg * const initArgs[] = { &initArg0, &initArg1, &initArg2,
		&initArg3, &initArg4, &initArg5, &initArg6, &initArg7, &initArg8 };
static const iocshFuncDef initFuncDef = { "FFT_calcConfigure", 9, initArgs };
static void initCallFunc(const iocshArgBuf *args) {
	FFT_calcConfigure(args[0].sval, args[1].ival, args[2].ival, args[3].sval,
			args[4].ival, args[5].ival, args[6].ival, args[7].ival,
			args[8].ival);
}

extern "C" void FFT_calc_register(void) {
	iocshRegister(&initFuncDef, initCallFunc);
}

extern "C" {
epicsExportRegistrar(FFT_calc_register);
}
