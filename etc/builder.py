from iocbuilder import Device, AutoSubstitution, Architecture, Xml
from iocbuilder.arginfo import *

from iocbuilder.modules.areaDetector import AreaDetector, _NDPluginBase, _NDFile, _NDFileBase, _ADBase
    
class gdaPlugins(Xml):
    """This plugin instantiates a standard set of plugins for use by GDA:"""
    TemplateFile = 'gdaPlugins.xml'  
gdaPlugins.ArgInfo.descriptions["CAM"] = Ident("Areadetecor camera to connect to", _ADBase)  

