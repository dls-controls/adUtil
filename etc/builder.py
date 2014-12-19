from iocbuilder import Device, AutoSubstitution, SetSimulation, Xml
from iocbuilder.arginfo import *

from iocbuilder.modules.areaDetector import AreaDetector, _ADBase, _ADBaseTemplate, _NDPluginProducerBase, _NDPluginBase
from iocbuilder.modules.asyn import AsynIP

class gdaPlugins(Xml):
    """This plugin instantiates a standard set of plugins for use by GDA:"""
    TemplateFile = 'gdaPlugins.xml'
gdaPlugins.ArgInfo.descriptions["CAM"] = Ident("Areadetecor camera to connect to", _ADBase)

class AdUtil(Device):
    '''Library dependencies for adUtil'''
    Dependencies = (AreaDetector,)
    # Device attributes
    LibFileList = ['adUtil']
    DbdFileList = ['adUtilSupport']  
    AutoInstantiate = True      

class _BPM_calc(AutoSubstitution):
    """Template containing the records for an NDROI"""
    TemplateFile = 'BPM_calc.template'

class BPM_calc(_NDPluginProducerBase):
    """This plugin selects a region of interest and optionally scales it to
    fit in a particular data type"""
    _SpecificTemplate = _BPM_calc
    # NOTE: _NDPluginBase comes 2nd so we overwrite NDARRAY_PORT argInfo
    ArgInfo = _SpecificTemplate.ArgInfo + _NDPluginBase.ArgInfo
    Dependencies = (AdUtil,)

class _FFT_calc(AutoSubstitution):
    TemplateFile = 'FFT_calc.template'

class FFT_calc(_NDPluginProducerBase):
    _SpecificTemplate = _FFT_calc
    ArgInfo = _SpecificTemplate.ArgInfo + _NDPluginBase.ArgInfo
    Dependencies = (AdUtil,)

class _NDCircularBuff(AutoSubstitution):
    TemplateFile = 'NDCircularBuff.template'

class NDCircularBuff(_NDPluginBase):
    '''This plugin provides a pre and post external trigger frame capture buffer'''
    #Dependencies = (FFmpegServer,)    
    _SpecificTemplate = _NDCircularBuff
    
    def __init__(self, QUEUE = 50, BUFFERS = 1000, MEMORY = -1, Enabled = 1, **args):
        # Init the superclass (_NDPluginBase)
        args["Enabled"] = Enabled
        self.__super.__init__(**args)
        # Store the args
        self.__dict__.update(locals())

    # __init__ arguments
    # NOTE: _NDPluginBase comes 2nd so we overwrite NDARRAY_PORT argInfo
    ArgInfo = _SpecificTemplate.ArgInfo + _NDPluginBase.ArgInfo + makeArgInfo(__init__,        
        Enabled   = Simple('Plugin Enabled at startup?', int),
        QUEUE     = Simple('Input array queue size', int),          
        BUFFERS   = Simple('Max number of buffers to allocate', int),
        MEMORY    = Simple('Max memory to allocate, should be maxw*maxh*nbuffer '
            'for driver and all attached plugins', int))
    Dependencies = [AdUtil,]
    #def InitialiseOnce(self):
    #    print "ffmpegServerConfigure(%(HTTP_PORT)d)" % self.__dict__                        
                                                                        
    def Initialise(self):
        print '# NDCircularBuffConfigure(portName, queueSize, blockingCallbacks, '\
            'NDArrayPort, NDArrayAddr, maxBuffers, maxMemory)'    
        print 'NDCircularBuffConfigure(' \
            '"%(PORT)s", %(QUEUE)d, %(BLOCK)d, "%(NDARRAY_PORT)s", ' \
            '"%(NDARRAY_ADDR)s", %(BUFFERS)d, %(MEMORY)d)' % self.__dict__


