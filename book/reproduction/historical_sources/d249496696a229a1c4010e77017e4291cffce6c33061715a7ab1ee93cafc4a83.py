"""Host memory accounting, including Windows commit rather than RAM alone."""
import ctypes,os
import psutil


def memory_state():
    out=dict(physical_available_GiB=psutil.virtual_memory().available/2**30,
             process_private_GiB=getattr(psutil.Process().memory_info(),'private',psutil.Process().memory_info().vms)/2**30)
    if os.name=='nt':
        class Memory(ctypes.Structure):
            _fields_=[('length',ctypes.c_ulong),('load',ctypes.c_ulong)]+[(n,ctypes.c_ulonglong) for n in
                ['total_phys','avail_phys','total_page','avail_page','total_virtual','avail_virtual','extended']]
        x=Memory();x.length=ctypes.sizeof(x)
        if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(x)):raise OSError('Cannot read commit headroom')
        out.update(commit_limit_GiB=x.total_page/2**30,commit_available_GiB=x.avail_page/2**30)
    else:out.update(commit_limit_GiB=None,commit_available_GiB=out['physical_available_GiB'])
    return out


def native_memory_cap(model,state=None):
    state=memory_state() if state is None else state
    # SCIP's stopping counter includes its external LP-memory estimate, not
    # just block memory. Include the already-counted estimate in the baseline;
    # it is NOT additional physical headroom. Retain the same2GiB floor and
    #35% growth admission from fresh RAM AND Windows commit measurements.
    headroom=min(state['commit_available_GiB'],state['physical_available_GiB'])
    return max(256.,(model.getMemTotal()+model.getMemExternEstim())/2**20+.35*max(0.,headroom-2.)*1024)


def native_memory_readmission(model,state=None):
    """Admit bounded growth, including useful increments below256MiB.

    A cap hit is not evidence that the host reserve has been exhausted. The
    previous256MiB increment gate discarded the transformed tree despite a
    freshly admitted177MiB growth allowance. Keep the same2GiB host reserve
    and35% growth fraction, with16MiB hysteresis to avoid no-progress loops.
    """
    state=memory_state() if state is None else state
    previous=float(model.getParam('limits/memory'))
    next_limit=native_memory_cap(model,state)
    headroom=min(state['physical_available_GiB'],state['commit_available_GiB'])
    return dict(host=state,previous_limit_MiB=previous,next_limit_MiB=next_limit,
        host_floor_GiB=2.,minimum_increment_MiB=16.,same_solver_resume=True,
        allowed=bool(headroom>2. and next_limit>previous+16.))
