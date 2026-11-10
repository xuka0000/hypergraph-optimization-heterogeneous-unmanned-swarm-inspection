"""In-process Windows memory guard, without an observer or background worker."""
import ctypes as c,ctypes.wintypes as w,os
_handles=[]

def memory_guard(limit_bytes):
    if os.name!='nt':raise RuntimeError('This local executor budget guard is Windows-specific')
    class BASIC(c.Structure):
        _fields_=[('PerProcessUserTimeLimit',c.c_int64),('PerJobUserTimeLimit',c.c_int64),('LimitFlags',w.DWORD),('MinimumWorkingSetSize',c.c_size_t),('MaximumWorkingSetSize',c.c_size_t),('ActiveProcessLimit',w.DWORD),('Affinity',c.c_size_t),('PriorityClass',w.DWORD),('SchedulingClass',w.DWORD)]
    class IO(c.Structure):
        _fields_=[(k,c.c_ulonglong) for k in ['ReadOperationCount','WriteOperationCount','OtherOperationCount','ReadTransferCount','WriteTransferCount','OtherTransferCount']]
    class EXT(c.Structure):
        _fields_=[('BasicLimitInformation',BASIC),('IoInfo',IO),('ProcessMemoryLimit',c.c_size_t),('JobMemoryLimit',c.c_size_t),('PeakProcessMemoryUsed',c.c_size_t),('PeakJobMemoryUsed',c.c_size_t)]
    kernel=c.WinDLL('kernel32',use_last_error=True);kernel.CreateJobObjectW.restype=w.HANDLE;kernel.GetCurrentProcess.restype=w.HANDLE
    kernel.AssignProcessToJobObject.argtypes=[w.HANDLE,w.HANDLE];kernel.SetInformationJobObject.argtypes=[w.HANDLE,c.c_int,c.c_void_p,w.DWORD]
    handle=kernel.CreateJobObjectW(None,None);info=EXT();info.BasicLimitInformation.LimitFlags=0x100;info.ProcessMemoryLimit=int(limit_bytes)
    if not kernel.SetInformationJobObject(handle,9,c.byref(info),c.sizeof(info)):raise OSError(c.get_last_error(),'Unable to set process memory ceiling')
    if not kernel.AssignProcessToJobObject(handle,kernel.GetCurrentProcess()):raise OSError(c.get_last_error(),'Unable to bind memory ceiling')
    _handles.append(handle)
    return dict(kind='Windows JobObject process commit limit',limit_bytes=int(limit_bytes),bound_to_pid=os.getpid(),assigned=True)
