import pathlib,json,sys,numpy as np
base=pathlib.Path(sys.argv[1]); rows=[]
calltype=np.dtype([('before','<u8'),('after','<u8'),('offset','<u8'),('requested','<u8'),('returned','<i4'),('copied','<u4')])
for info in sorted(base.glob('*/rx/capture.json')):
 prefix=info.parent/'capture';case=info.parent.parent.name;meta=json.loads(info.read_text())
 iq=np.fromfile(prefix.with_suffix('.iq'),dtype='<i2').reshape(-1,2).astype(np.float32)
 raw=np.fromfile(prefix.with_suffix('.smi'),dtype='<u4')
 metadata=np.fromfile(prefix.with_suffix('.meta'),dtype='u1')
 calls=np.fromfile(prefix.with_suffix('.calls'),dtype=calltype)
 rawcalls=np.fromfile(prefix.with_suffix('.smi_calls'),dtype=calltype)
 invalid=(raw & np.uint32(0xc001c000))!=np.uint32(0x80004000)
 amp=np.sqrt(iq[:,0]**2+iq[:,1]**2)
 phase=np.arctan2(iq[1:,1]*iq[:-1,0]-iq[1:,0]*iq[:-1,1],iq[1:,0]*iq[:-1,0]+iq[1:,1]*iq[:-1,1])
 times=(calls['after']-calls['before'])/1e6
 row={'case':case,'capture':meta,'raw_words':len(raw),'invalid_frame_words':int(invalid.sum()),'invalid_frame_fraction':float(invalid.mean()) if len(raw) else None,
      'iq_min':np.min(iq,axis=0).tolist(),'iq_max':np.max(iq,axis=0).tolist(),'iq_mean':np.mean(iq,axis=0,dtype=np.float64).tolist(),
      'amplitude_quantiles':np.quantile(amp,[0,.01,.5,.99,1]).tolist(),'large_rf_phase_steps_fraction':float((np.abs(phase)>.5).mean()),
      'read_return_counts':{str(int(k)):int(v) for k,v in zip(*np.unique(calls['returned'],return_counts=True))},
      'read_duration_ms_quantiles':np.quantile(times,[0,.5,.9,.99,1]).tolist(),'metadata_counts':{str(int(k)):int(v) for k,v in zip(*np.unique(metadata,return_counts=True))}}
 bad=np.flatnonzero(invalid)
 row['first_invalid_word_indices']=bad[:30].tolist()
 # Decode both 13-bit payloads directly, independently of the application DSP.
 if len(raw)==len(iq):
  decoded=np.column_stack(((raw>>1)&8191,(raw>>17)&8191)).astype(np.int16)
  decoded[decoded>=4096]-=8192
  row['raw_to_library_iq_mismatch_fraction']=float(np.any(decoded!=iq,axis=1).mean())
  del decoded
 rows.append(row);print(json.dumps(row),flush=True)
 del iq,raw,amp,phase,metadata,calls,rawcalls,invalid
(base/'capture-analysis.json').write_text(json.dumps(rows,indent=2)+'\n')
