from iocbuilder import Device, AutoSubstitution, SetSimulation, Xml
from iocbuilder.arginfo import *

from iocbuilder.modules.ADCore import ADCore, ADBaseTemplate, NDPluginBaseTemplate, includesTemplates, makeTemplateInstance
from iocbuilder.modules.asyn import AsynIP, AsynPort

class gdaPlugins(Xml):
    """This plugin instantiates a standard set of plugins for use by GDA:"""
    TemplateFile = 'gdaPlugins.xml'
gdaPlugins.ArgInfo.descriptions["CAM"] = Ident("Areadetecor camera to connect to", ADCore)

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
class _NDCircularBuff(AutoSubstitution):
    TemplateFile = 'NDCircularBuff.template'

class NDCircularBuff(AsynPort):
    '''This plugin provides a pre and post external trigger frame capture buffer'''
    # This tells xmlbuilder to use PORT instead of name as the row ID
    UniqueName = "PORT"
    Dependencies = (AdUtil,)    
    _SpecificTemplate = _NDCircularBuff
        
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
        print '# NDCircularBuffConfigure(portName, queueSize, blockingCallbacks, '\
            'NDArrayPort, NDArrayAddr, maxBuffers, maxMemory)'    
        print 'NDCircularBuffConfigure(' \
            '"%(PORT)s", %(QUEUE)d, %(BLOCK)d, "%(NDARRAY_PORT)s", ' \
            '"%(NDARRAY_ADDR)s", %(BUFFERS)d, %(MEMORY)d)' % self.__dict__


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



