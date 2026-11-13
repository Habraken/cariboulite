import pathlib,json,numpy as np,subprocess
base=pathlib.Path('build/radio-backend/stutter-diagnosis-20261113/cariboulite-capture-cases-20261113T205437Z')
dt=np.dtype([('before','<u8'),('after','<u8'),('offset','<u8'),('requested','<u8'),('returned','<i4'),('copied','<u4')])
results=[]
for case in ['fresh_1m_a','fresh_1m_b','warm_4_to_1','soft_reset_1m','hard_program_1m']:
 prefix=base/case/'rx/capture';data=prefix.with_suffix('.smi').read_bytes();actual=np.fromfile(prefix.with_suffix('.iq'),dtype='<i2').reshape(-1,2);calls=np.fromfile(prefix.with_suffix('.smi_calls'),dtype=dt)
 scores=[]
 for off in range(4):
  w=np.frombuffer(data,dtype='<u4',count=(len(data)-off)//4,offset=off);scores.append(float(np.mean((w&0xc001c000)==0x80004000)))
 true=int(np.argmax(scores));correct=[];selected_counts={};wrong_calls=0;wrong_samples=0;matched=0;checked=0
 for c in calls:
  offset=int(c['offset']);n=int(c['copied']);chunk=data[offset:offset+n]
  if len(chunk)<=16:continue
  chosen=None
  for off in range(4):
   if len(chunk)<off+16:continue
   words=np.frombuffer(chunk,dtype='<u4',count=4,offset=off)
   if np.all((words&0xc001c000)==0x80004000):chosen=off;break
  if chosen is None:raise RuntimeError('No candidate')
  selected_counts[str(chosen)]=selected_counts.get(str(chosen),0)+1
  true_local=(true-offset)%4
  if chosen!=true_local:wrong_calls+=1;wrong_samples+=n//4
  # Simulate the actual unpacker on every non-interpolated returned sample.
  count=n//4-(1 if chosen else 0)
  words=np.frombuffer(chunk,dtype='<u4',count=count,offset=chosen)
  decoded=np.column_stack(((words>>1)&8191,(words>>17)&8191)).astype(np.int16);decoded[decoded>=4096]-=8192
  start=offset//4;limit=min(count,len(actual)-start)
  matched+=int(np.all(decoded[:limit]==actual[start:start+limit],axis=1).sum());checked+=limit
 # Independently align the complete stream once; no per-read guessed locks,
 # no dropped fragments, no interpolated samples, and no waveform repair.
 words=np.frombuffer(data,dtype='<u4',count=(len(data)-true)//4,offset=true)
 iq=np.column_stack(((words>>1)&8191,(words>>17)&8191)).astype(np.int16);iq[iq>=4096]-=8192
 out=prefix.parent/'aligned.iq';iq.tofile(out)
 row={'case':case,'global_byte_offset':true,'valid_frame_fractions_by_offset':scores,'selected_offset_counts':selected_counts,
      'false_alignment_read_calls':wrong_calls,'samples_in_false_alignment_reads':wrong_samples,
      'simulated_unpacker_samples_checked':checked,'simulated_unpacker_samples_matching_library':matched,
      'aligned_iq_peak':np.abs(iq.astype(np.int32)).max(axis=0).tolist()}
 results.append(row);print(json.dumps(row),flush=True)
 subprocess.run(['build/radio-backend/stutter-diagnosis-20261113/replay_iq',str(out),'1000000',str(prefix.parent/'aligned-replay')],check=True)
(base/'alignment-proof.json').write_text(json.dumps(results,indent=2)+'\n')
