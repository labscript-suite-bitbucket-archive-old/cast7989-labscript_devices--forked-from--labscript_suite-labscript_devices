#####################################################################
#                                                                   #
# /NI_PXIe_6535.py                                                  #
#                                                                   #
#####################################################################

from labscript import LabscriptError
from labscript_devices import labscript_device, BLACS_tab, BLACS_worker, runviewer_parser
import labscript_devices.myNIBoard as parent

import numpy as np
import labscript_utils.h5_lock, h5py
import labscript_utils.properties


@labscript_device
class NI_PXIe_6535(parent.myNIBoard):
  description = 'NI-PXIe-6535'
  n_digitals = 32
  clock_limit = 5e6
  digital_dtype = np.uint32

import time

from blacs.tab_base_classes import Worker, define_state
from blacs.tab_base_classes import MODE_MANUAL, MODE_TRANSITION_TO_BUFFERED
from blacs.tab_base_classes import MODE_TRANSITION_TO_MANUAL, MODE_BUFFERED  
from blacs.device_base_class import DeviceTab

@BLACS_tab
class NI_PXIe_6535Tab(DeviceTab):
    def initialise_GUI(self):
        # Capabilities
        self.num_DO = 32
        self.num_ports = 4
        self.num_lines = 8
        
        # Create the DO output objects
        do_prop = {}
        for i in range(self.num_ports):
            for j in range(self.num_lines):
                do_prop['port%d/line%d'%(i,j)] = {}
        
        # now create the digital output objects
        self.create_digital_outputs(do_prop)
        # manually create the digital output widgets so they are grouped separately
        do_widgets = self.create_digital_widgets(do_prop)
        
        def do_sort(channel):
            flagi, flagj = channel.replace('port','').replace('line','').split('/')
            flagi, flagj = int(flagi),int(flagj)
            flagk = self.num_lines*flagi+flagj
            return '%02d'%(flagk)
        
        # and auto place the widgets in the UI
        self.auto_place_widgets(("Digital Outputs",do_widgets,do_sort))
        
        # Store the Measurement and Automation Explorer (MAX) name
        self.MAX_name = str(self.settings['connection_table'].find_by_name(self.device_name).BLACS_connection)
        
        # Create and set the primary worker
        self.create_worker("main_worker",NiPXIe6535Worker,{'MAX_name':self.MAX_name,'num_DO':self.num_DO,'num_ports':self.num_ports,'num_lines':self.num_lines})
        self.primary_worker = "main_worker"

        # Set the capabilities of this device
        self.supports_remote_value_check(False)
        self.supports_smart_programming(False) 
    
@BLACS_worker
class NiPXIe6535Worker(Worker):
    def init(self):
        exec 'from PyDAQmx import Task, DAQmxGetSysNIDAQMajorVersion, DAQmxGetSysNIDAQMinorVersion, DAQmxGetSysNIDAQUpdateVersion' in globals()
        exec 'from PyDAQmx.DAQmxConstants import *' in globals()
        exec 'from PyDAQmx.DAQmxTypes import *' in globals()
        global pylab; import pylab
        global numpy; import numpy
        global h5py; import labscript_utils.h5_lock, h5py
        
        # check version of PyDAQmx
        major = uInt32()
        minor = uInt32()
        patch = uInt32()
        DAQmxGetSysNIDAQMajorVersion(major)
        DAQmxGetSysNIDAQMinorVersion(minor)
        DAQmxGetSysNIDAQUpdateVersion(patch)
        
        if major.value == 14 and minor.value < 2:
            version_exception_message = 'There is a known bug with buffered shots using NI DAQmx v14.0.0. This bug does not exist on v14.2.0. You are currently using v%d.%d.%d. Please ensure you upgrade to v14.2.0 or higher.'%(major.value, minor.value, patch.value)
            raise Exception(version_exception_message)
        
        # Create DO task:
        self.do_task = Task()
        self.do_read = int32()
        self.do_data = numpy.zeros(self.num_DO,dtype=numpy.uint8)
        
        self.setup_static_channels()            
        
        #DAQmx Start Code        
        self.do_task.StartTask()  
        
    def setup_static_channels(self):
        #setup DO ports
        self.do_task.CreateDOChan(self.MAX_name+"/port0/line0:7","",DAQmx_Val_ChanPerLine)
        self.do_task.CreateDOChan(self.MAX_name+"/port1/line0:7","",DAQmx_Val_ChanPerLine)
        self.do_task.CreateDOChan(self.MAX_name+"/port2/line0:7","",DAQmx_Val_ChanPerLine)
        self.do_task.CreateDOChan(self.MAX_name+"/port3/line0:7","",DAQmx_Val_ChanPerLine)
                
    def shutdown(self):
        self.do_task.StopTask()
        self.do_task.ClearTask()
        
    def program_manual(self,front_panel_values):
        k = 0
        for i in range(self.num_ports):
          for j in range(self.num_lines):
            self.do_data[k] = front_panel_values['port%d/line%d'%(i,j)]
            k += 1
			
        self.do_task.WriteDigitalLines(1,True,1,DAQmx_Val_GroupByChannel,self.do_data,byref(self.do_read),None)
     
        # TODO: return coerced/quantised values
        return {}
        
    def transition_to_buffered(self,device_name,h5file,initial_values,fresh):
        # Store the initial values in case we have to abort and restore them:
        self.initial_values = initial_values
            
        with h5py.File(h5file,'r') as hdf5_file:
            group = hdf5_file['devices/'][device_name]
            device_properties = labscript_utils.properties.get(hdf5_file, device_name, 'device_properties')
            connection_table_properties = labscript_utils.properties.get(hdf5_file, device_name, 'connection_table_properties')
            clock_terminal = connection_table_properties['clock_terminal']
            h5_data = group.get('DIGITAL_OUTS')
            if h5_data:
                self.buffered_using_digital = True
                do_channels = device_properties['digital_lines']
                do_bitfield = numpy.array(h5_data,dtype=numpy.uint32)
                
            else:
                self.buffered_using_digital = False
				
        final_values = {} 
        # We must do digital first, so as to make sure the manual mode task is stopped, or reprogrammed, by the time we setup the AO task
        # this is because the clock_terminal PFI must be freed!
        if self.buffered_using_digital:
            # Expand each bitfield int into self.num['DO']
            # (32) individual ones and zeros:
            do_write_data = numpy.zeros((do_bitfield.shape[0],self.num_DO),dtype=numpy.uint8)
            for i in range(self.num_DO):
                do_write_data[:,i] = (do_bitfield & (1 << i)) >> i

            self.do_task.StopTask()
            self.do_task.ClearTask()
            self.do_task = Task()
            self.do_read = int32()
    
            self.do_task.CreateDOChan(do_channels,"",DAQmx_Val_ChanPerLine)
            self.do_task.CfgSampClkTiming(clock_terminal,5000000,DAQmx_Val_Rising,DAQmx_Val_FiniteSamps,do_bitfield.shape[0])
            self.do_task.WriteDigitalLines(do_bitfield.shape[0],False,10.0,DAQmx_Val_GroupByScanNumber,do_write_data,self.do_read,None)
            self.do_task.StartTask()
            
            k = 0
            for i in range(self.num_ports):
              for j in range(self.num_lines):
                final_values['port%d/line%d'%(i,j)] = do_write_data[-1,k]
                k += 1
        else:
            # We still have to stop the task to make the 
            # clock flag available for buffered analog output, or the wait monitor:
            self.do_task.StopTask()
            self.do_task.ClearTask()
			
        return final_values
        
    def transition_to_manual(self,abort=False):
        # if aborting, don't call StopTask since this throws an
        # error if the task hasn't actually finished!
        if self.buffered_using_digital:
            if not abort:
                self.do_task.StopTask()
            self.do_task.ClearTask()
                
        self.do_task = Task()
        self.setup_static_channels()
        self.do_task.StartTask()
        if abort:
            # Reprogram the initial states:
            self.program_manual(self.initial_values)
            
        return True
        
    def abort_transition_to_buffered(self):
        # TODO: untested
        return self.transition_to_manual(True)
        
    def abort_buffered(self):
        # TODO: untested
        return self.transition_to_manual(True)

@runviewer_parser
class RunviewerClass(parent.RunviewerClass):
    num_digitals = 32