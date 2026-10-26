"""Loaded by GDB in file-only mode; never creates or runs an inferior."""
import gdb,json
N='eprosima::fastrtps::rtps::'
def T(n):return gdb.lookup_type(n).strip_typedefs()
def fields(t):
    out=[]
    for f in t.strip_typedefs().fields():
        try:b=f.bitpos
        except (AttributeError,RuntimeError):continue
        if b is None:continue
        out.append((f,b//8))
    return out
def member(t,n):
    return next((f,off)for f,off in fields(t)if f.name==n)
def paths(t,wanted,depth=0,offset=0,path=(),ancestors=()):
    t=t.strip_typedefs();name=str(t)
    if depth>12 or name in ancestors:return []
    if t.code not in (gdb.TYPE_CODE_STRUCT,gdb.TYPE_CODE_UNION):return []
    out=[]
    for f,off in fields(t):
        q=path+(dict(field=f.name,type=str(f.type),offset=off,base=bool(f.is_base_class)),)
        if wanted(f):out.append(dict(offset=offset+off,path=q))
        out.extend(paths(f.type,wanted,depth+1,offset+off,q,ancestors+(name,)))
    return out
e=T(N+'Endpoint');p=T(N+'RTPSParticipantImpl');m=T(N+'SendBuffersManager');cv=member(m,'available_cv_')[0].type
u=member(p,'send_buffers_')[0].type;cvptr=member(cv,'_M_mutex')[0].type
up=paths(u,lambda f:f.type.strip_typedefs().code==gdb.TYPE_CODE_PTR and 'SendBuffersManager' in str(f.type))
sp=paths(cvptr,lambda f:f.name=='_M_ptr' and f.type.strip_typedefs().code==gdb.TYPE_CODE_PTR)
a=member(e,'mp_mutex')[0].type;sm=member(p,'m_send_resources_mutex_')[0].type;mm=member(m,'mutex_')[0].type
out=dict(types={n:dict(size=T(n).sizeof,fields=[dict(name=f.name,offset=o,type=str(f.type),base=bool(f.is_base_class))for f,o in fields(T(n)) if f.is_base_class or f.name in ('mp_RTPSParticipant','mp_mutex','m_guid','send_buffers_','m_send_resources_mutex_','mutex_','available_cv_','_M_mutex','_M_cond')])for n in (N+'Endpoint',N+'RTPSReader',N+'StatefulReader',N+'RTPSParticipantImpl',N+'SendBuffersManager','eprosima::fastrtps::TimedConditionVariable')},unique_ptr_raw_pointer_paths=up,cv_shared_ptr_raw_pointer_paths=sp,native_lock_paths=dict(A=paths(a,lambda f:f.name=='__lock'),M=paths(mm,lambda f:f.name=='__lock'),S=paths(sm,lambda f:f.name=='__lock'),I_pointee=paths(T('std::mutex'),lambda f:f.name=='__lock')))
print('PROBE_LAYOUT_JSON='+json.dumps(out))
