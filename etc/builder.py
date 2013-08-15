from iocbuilder import Device, AutoSubstitution, SetSimulation
from iocbuilder.arginfo import *

from iocbuilder.modules.areaDetector import AreaDetector, _ADBase, _ADBaseTemplate, _NDPluginProducerBase, _NDPluginBase
from iocbuilder.modules.asyn import AsynIP

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
