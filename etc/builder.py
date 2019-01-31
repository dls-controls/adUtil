from iocbuilder import Device, AutoSubstitution, SetSimulation, Xml
from iocbuilder.arginfo import *

from iocbuilder.modules.ADCore import ADCore, ADBaseTemplate, NDPluginBaseTemplate, includesTemplates, makeTemplateInstance
from iocbuilder.modules.asyn import AsynIP, AsynPort

class _gdaPlugins(Xml):
    """This plugin instantiates a standard set of plugins for use by GDA:"""
    TemplateFile = 'gdaPlugins.xml'

class gdaPlugins(Device):

    def __init__(
        self, CAM, P, PORTPREFIX, PLUGINQUEUE=16, HIST_SIZE=256, NCHANS=1024, 
        XSIZE=1024, YSIZE=1024, ARRFTVL="UCHAR", NELEMENTS=786432, 
        DISPLAYQUEUE=2, ARRTYPE="Int8", HTTP_PORT=8080, CentroidThreshold=50,
        ComputeProfiles=1, ComputeStatistics=0, ComputeCentroid=0):

        # I wish I could easily unpack dictionaries in Python 2...
        _gdaPlugins(
            CAM=CAM, P=P, PORTPREFIX=PORTPREFIX, PLUGINQUEUE=PLUGINQUEUE,
            HIST_SIZE=HIST_SIZE, NCHANS=NCHANS, XSIZE=XSIZE, YSIZE=YSIZE, 
            ARRFTVL=ARRFTVL, NELEMENTS=NELEMENTS, DISPLAYQUEUE=DISPLAYQUEUE, 
            ARRTYPE=ARRTYPE, HTTP_PORT=HTTP_PORT, 
            CentroidThreshold=CentroidThreshold, ComputeProfiles=ComputeProfiles,
            ComputeStatistics=ComputeStatistics, ComputeCentroid=ComputeCentroid)


    ArgInfo = makeArgInfo(__init__,
        CAM = Ident("Areadetector camera to connect to", ADCore),
        P = Simple("PV prefix for all plugins", str),
        PORTPREFIX = Simple("Asyn port name prefix for each plugin", str),
        PLUGINQUEUE = Simple("Size of queue for each plugin (except NDStdArrays)", int),
        HIST_SIZE = Simple("Size of histogram in NDStats plugins", int),
        NCHANS = Simple("Number of elements in the NDStats waveforms for past statistics", int),
        XSIZE = Simple("Size of NDStats X Histograms", int),
        YSIZE = Simple("Size of NDStats Y Histograms", int),
        ARRFTVL = Choice("Data format of NDStdArrays waveform", ["STRING", "CHAR", "UCHAR", "SHORT", "USHORT", "LONG", "ULONG", "FLOAT", "DOUBLE", "ENUM"]),
        NELEMENTS = Simple("Size of NDStdArrays waveform (Should equal size number of pixels in image)", int),
        DISPLAYQUEUE = Simple("Size of queue for NDStdArrays plugin", int),
        ARRTYPE = Choice("Asyn Array type of NDStdSrrays waveform", ["Int8", "Int16", "Int32", "Float32", "Float64"]),
        HTTP_PORT = Simple("Port used for ffmpegStream plugin", int),
        CentroidThreshold = Simple("Threshold for calculating centroids in NDStats plugins", int),
        ComputeProfiles = Choice("Toggle computation of profiles in NDStats plugins", [0, 1]),
        ComputeStatistics = Choice("Toggle computation of basic statistics in NDStats plugins", [0, 1]),
        ComputeCentroid = Choice("Toggle computation of centroids in NDStats plugins", [0, 1]))

class AdUtil(Device):
    '''Library dependencies for adUtil'''
    Dependencies = (ADCore,)
    # Device attributes
    LibFileList = ['adUtil']
    DbdFileList = ['adUtilSupport']  
    AutoInstantiate = True      

@includesTemplates(NDPluginBaseTemplate)
class _BPM_calc(AutoSubstitution):
    """Template containing the records for an NDROI"""
    TemplateFile = 'BPM_calc.template'

class BPM_calc(AsynPort):
    """This plugin selects a region of interest and optionally scales it to
    fit in a particular data type"""
    # This tells xmlbuilder to use PORT instead of name as the row ID
    UniqueName = "PORT"
    Dependencies = (AdUtil,)    
    _SpecificTemplate = _BPM_calc

    def __init__(self, PORT, NDARRAY_PORT, QUEUE = 2, BLOCK = 0, NDARRAY_ADDR = 0, BUFFERS = 50, MEMORY = 0, **args):
        # Init the superclass (AsynPort)
        self.__super.__init__(PORT)
        # Update the attributes of self from the commandline args
        self.__dict__.update(locals())
        # Make an instance of our template
        makeTemplateInstance(self._SpecificTemplate, locals(), args)

    ArgInfo = _SpecificTemplate.ArgInfo + makeArgInfo(__init__,
        PORT = Simple('Port name for the BPM_calc plugin', str),
        QUEUE = Simple('Input array queue size', int),
        BLOCK = Simple('Blocking callbacks?', int),
        NDARRAY_PORT = Ident('Input array port', AsynPort),
        NDARRAY_ADDR = Simple('Input array port address', int),
        BUFFERS = Simple('Max buffers to allocate', int),
        MEMORY = Simple('Max memory to allocate, should be maxw*maxh*nbuffer for driver and all attached plugins', int))

    def Initialise(self):
        print '# BPM_calcConfigure(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr, maxBuffers, maxMemory)' % self.__dict__
        print 'BPM_calcConfigure("%(PORT)s", %(QUEUE)d, %(BLOCK)d, "%(NDARRAY_PORT)s", %(NDARRAY_ADDR)s, %(BUFFERS)d, %(MEMORY)d)' % self.__dict__

@includesTemplates(NDPluginBaseTemplate)
class _FFT_calc(AutoSubstitution):
    TemplateFile = 'FFT_calc.template'

class FFT_calc(AsynPort):
    # This tells xmlbuilder to use PORT instead of name as the row ID
    UniqueName = "PORT"
    Dependencies = (AdUtil,)    
    _SpecificTemplate = _FFT_calc

    def __init__(self, PORT, NDARRAY_PORT, QUEUE = 2, BLOCK = 0, NDARRAY_ADDR = 0, BUFFERS = 50, MEMORY = 0, **args):
        # Init the superclass (AsynPort)
        self.__super.__init__(PORT)
        # Update the attributes of self from the commandline args
        self.__dict__.update(locals())
        # Make an instance of our template
        makeTemplateInstance(self._SpecificTemplate, locals(), args)

    ArgInfo = _SpecificTemplate.ArgInfo + makeArgInfo(__init__,
        PORT = Simple('Port name for the FFT_calc plugin', str),
        QUEUE = Simple('Input array queue size', int),
        BLOCK = Simple('Blocking callbacks?', int),
        NDARRAY_PORT = Ident('Input array port', AsynPort),
        NDARRAY_ADDR = Simple('Input array port address', int),
        BUFFERS = Simple('Max buffers to allocate', int),
        MEMORY = Simple('Max memory to allocate, should be maxw*maxh*nbuffer for driver and all attached plugins', int))

    def Initialise(self):
        print '# FFT_calcConfigure(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr, maxBuffers, maxMemory)' % self.__dict__
        print 'FFT_calcConfigure("%(PORT)s", %(QUEUE)d, %(BLOCK)d, "%(NDARRAY_PORT)s", %(NDARRAY_ADDR)s, %(BUFFERS)d, %(MEMORY)d)' % self.__dict__

@includesTemplates(NDPluginBaseTemplate)
class _FFTInt_calc(AutoSubstitution):
    TemplateFile = 'FFTInt_calc.template'

class FFTInt_calc(AsynPort):
    # This tells xmlbuilder to use PORT instead of name as the row ID
    UniqueName = "PORT"
    Dependencies = (AdUtil,)    
    _SpecificTemplate = _FFTInt_calc

    def __init__(self, PORT, NDARRAY_PORT, QUEUE = 2, BLOCK = 0, NDARRAY_ADDR = 0, BUFFERS = 50, MEMORY = 0, **args):
        # Init the superclass (AsynPort)
        self.__super.__init__(PORT)
        # Update the attributes of self from the commandline args
        self.__dict__.update(locals())
        # Make an instance of our template
        makeTemplateInstance(self._SpecificTemplate, locals(), args)

    ArgInfo = _SpecificTemplate.ArgInfo + makeArgInfo(__init__,
        PORT = Simple('Port name for the FFTInt_calc plugin', str),
        QUEUE = Simple('Input array queue size', int),
        BLOCK = Simple('Blocking callbacks?', int),
        NDARRAY_PORT = Ident('Input array port', AsynPort),
        NDARRAY_ADDR = Simple('Input array port address', int),
        BUFFERS = Simple('Max buffers to allocate', int),
        MEMORY = Simple('Max memory to allocate, should be maxw*maxh*nbuffer for driver and all attached plugins', int))

    def Initialise(self):
        print '# FFT_IntegrationConfigure(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr, maxBuffers, maxMemory)' % self.__dict__
        print 'FFT_IntegrationConfigure("%(PORT)s", %(QUEUE)d, %(BLOCK)d, "%(NDARRAY_PORT)s", %(NDARRAY_ADDR)s, %(BUFFERS)d, %(MEMORY)d)' % self.__dict__


@includesTemplates(NDPluginBaseTemplate)
class _NDReframe(AutoSubstitution):
    TemplateFile = 'NDReframe.template'

class NDReframe(AsynPort):
    '''This plugin provides a pre and post external trigger frame capture buffer'''
    # This tells xmlbuilder to use PORT instead of name as the row ID
    UniqueName = "PORT"
    Dependencies = (AdUtil,)    
    _SpecificTemplate = _NDReframe
    
    def __init__(self, PORT, NDARRAY_PORT, QUEUE = 50, BLOCK = 0, NDARRAY_ADDR = 0, BUFFERS = 1000, MEMORY = 0, ENABLED = 1, **args):
        #args["Enabled"] = Enabled
        # Init the superclass (AsynPort)
        self.__super.__init__(PORT)
        # Update the attributes of self from the commandline args
        self.__dict__.update(locals())
        # Make an instance of our template
        makeTemplateInstance(self._SpecificTemplate, locals(), args)

    # __init__ arguments
    # NOTE: _NDPluginBase comes 2nd so we overwrite NDARRAY_PORT argInfo
    ArgInfo = _SpecificTemplate.ArgInfo + makeArgInfo(__init__,
        PORT      = Simple('Port name for the FFT_calc plugin', str),
        ENABLED   = Simple('Plugin Enabled at startup?', int),
        QUEUE     = Simple('Input array queue size', int),          
        BLOCK     = Simple('Blocking callbacks?', int),
        NDARRAY_PORT = Ident('Input array port', AsynPort),
        NDARRAY_ADDR = Simple('Input array port address', int),
        BUFFERS   = Simple('Max number of buffers to allocate', int),
        MEMORY    = Simple('Max memory to allocate, should be maxw*maxh*nbuffer '
                           'for driver and all attached plugins', int))                                                                        
    def Initialise(self):
        print '# NDReframeConfigure(portName, queueSize, blockingCallbacks, '\
            'NDArrayPort, NDArrayAddr, maxBuffers, maxMemory)'    
        print 'NDReframeConfigure(' \
            '"%(PORT)s", %(QUEUE)d, %(BLOCK)d, "%(NDARRAY_PORT)s", ' \
            '"%(NDARRAY_ADDR)s", %(BUFFERS)d, %(MEMORY)d)' % self.__dict__
