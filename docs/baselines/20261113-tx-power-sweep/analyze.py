import json,csv,sys,pathlib,statistics,html,math
base=pathlib.Path(sys.argv[1]); dest=pathlib.Path(sys.argv[2]);dest.mkdir(parents=True,exist_ok=True)
rows=json.loads((base/'results.json').read_text()); meta=json.loads((base/'metadata.json').read_text())
# Keep repeats and unavailable fractions; a status sample is not an independent
# SINAD window (5 Hz status, 2 Hz estimator). Ranges are observed, not confidence intervals.
raw=[json.loads(l) for l in (base/'points.jsonl').read_text().splitlines()]
def stats(values):
 return {'mean':statistics.mean(values),'min':min(values),'max':max(values),
         'stdev':statistics.stdev(values) if len(values)>1 else 0,'median':statistics.median(values),'n':len(values)} if values else None
for r in rows:
 r.pop('tx_snapshot',None)
 selected=[x for x in raw if x['rx_rate_hz']==r['rx_rate_hz'] and x['tx_power_dbm']==r['tx_power_dbm']]
 # Uniform timestamp-only filter, independent of measurement value.
 end=max(p['rx']['monotonic_s'] for p in selected)
 selected=[p for p in selected if p['rx']['monotonic_s']>=end-5.0]
 r['packets']=len(selected)
 r['analysis_window_s']=5.0
 r['first_rx_monotonic_s']=selected[0]['rx']['monotonic_s']
 r['last_rx_monotonic_s']=selected[-1]['rx']['monotonic_s']
 rss=[p['rx']['hardware']['rf24']['registers'].get('RSSI') for p in selected]
 r['rssi_dbm']=stats([v-256 if v>127 else v for v in rss if isinstance(v,int) and v!=127])
 r['gate_open_fraction']=sum(p['rx']['rx']['gate_open'] is True for p in selected)/len(selected)
 for name,path,key in [('iq_drops','queue','drops'),('audio_drops','audio_queue','drops'),('alsa_underruns','playback','underruns'),('write_errors','playback','write_errors'),('read_errors','reader','errors')]:
  values=[p['rx']['rx'][path][key] for p in selected]
  r['rx_errors'][name]={'start':values[0],'end':values[-1],'delta':values[-1]-values[0]}
 pac=[p['tx']['hardware']['rf24']['registers']['PAC'] for p in selected if p['tx']['hardware']['rf24']['registers']['PAC'] is not None]
 r['tx_pa_code']=None if r['tx_power_dbm'] is None else (pac[-1]&31 if pac else None)
 for key,path in [('sinad_db',('rx','sinad','db')),('noise_rms',('rx','noise','rms'))]:
  values=[]
  for p in selected:
   v=p['rx']
   for k in path:v=v[k]
   if isinstance(v,(int,float)):values.append(v)
  r[key]=stats(values)
 r['sinad_valid_fraction']=r['sinad_db']['n']/len(selected) if r['sinad_db'] else 0
 r['wide_sinad_range']=bool(r['sinad_db'] and r['sinad_db']['max']-r['sinad_db']['min']>2)
 # Add counters for TX defects as well as RX point deltas.
 r['tx_queue_drops_max']=max((p['tx']['tx'].get('queue') or {}).get('drops',0) for p in selected)
 r['tx_writer_errors_max']=max((p['tx']['tx'].get('writer') or {}).get('errors',0) for p in selected)
meta['analysis_window_s']=5.0
meta['analysis_filter']='Keep snapshots in trailing 5 seconds of each point using RX monotonic timestamps; excludes queued stale packets at entry.'
meta['rx_backend']=json.loads((base/'rx-ready.json').read_text())['metadata']
meta['tx_backend']=json.loads((base/'tx-ready.json').read_text())['metadata']
(dest/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
(dest/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
columns=['rx_rate_hz','tx_power_dbm','tx_pa_code','tx_actual_power_dbm','nominal_rx_input_dbm','packets','sinad_valid_fraction','gate_open_fraction','wide_sinad_range']
for metric in ['rssi_dbm','sinad_db','noise_rms']:
 columns += [metric+'_'+key for key in ['mean','min','max','stdev','median','n']]
columns+=['rx_'+name+'_delta' for name in ['iq_drops','audio_drops','alsa_underruns','write_errors','read_errors']]
columns+=['tx_driver_underruns_max','tx_queue_drops_max','tx_writer_errors_max']
with (dest/'results.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=columns,lineterminator="\n");w.writeheader()
 for r in rows:
  flat={k:r.get(k) for k in columns}
  for metric in ['rssi_dbm','sinad_db','noise_rms']:
   for key in ['mean','min','max','stdev','median','n']:flat[metric+'_'+key]=(r[metric] or {}).get(key)
  for name in ['iq_drops','audio_drops','alsa_underruns','write_errors','read_errors']:flat['rx_'+name+'_delta']=r['rx_errors'][name]['delta']
  w.writerow(flat)

def plot(metric,title,ylabel,ymin,ymax,ticks,filename):
 W,H=1080,610; L,R,T,B=95,40,78,110;pw=W-L-R;ph=H-T-B
 def x(v):return L+(v+12)/26*pw
 def y(v):return T+(ymax-v)/(ymax-ymin)*ph
 def esc(v):return html.escape(str(v))
 a=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" role="img" aria-label="{esc(title)}">',
 '<rect width="100%" height="100%" fill="white"/>',
 '<style>text{font-family:Arial,sans-serif;fill:#17212b;font-size:15px}.grid{stroke:#dde3e9;stroke-width:1}.axis{stroke:#334155;stroke-width:1.5}</style>',
 f'<text x="{L}" y="30" font-size="23" font-weight="bold">{esc(title)}</text>',
 f'<text x="{L}" y="55" fill="#475569">430.100 MHz · 90 dB nominal attenuation · TX 4 MS/s · 1 kHz NBFM tone</text>',
 f'<rect x="{x(10)-8}" y="{T}" width="{x(14)-x(10)+8}" height="{ph}" fill="#f1f5f9"/>']
 for v in ticks:
  a += [f'<line class="grid" x1="{L}" y1="{y(v)}" x2="{W-R}" y2="{y(v)}"/>',f'<text x="{L-12}" y="{y(v)+5}" text-anchor="end">{v}</text>']
 for v in range(-12,15,2):
  a += [f'<line class="grid" x1="{x(v)}" y1="{T}" x2="{x(v)}" y2="{T+ph}"/>',f'<text x="{x(v)}" y="{T+ph+27}" text-anchor="middle">{v:+}</text>']
 a += [f'<line class="axis" x1="{L}" y1="{T+ph}" x2="{W-R}" y2="{T+ph}"/>',f'<line class="axis" x1="{L}" y1="{T}" x2="{L}" y2="{T+ph}"/>',
 f'<text x="{L+pw/2}" y="{H-59}" text-anchor="middle">Requested TX power (dBm); shaded steps share PA code 31</text>',
 f'<text transform="translate(24,{T+ph/2}) rotate(-90)" text-anchor="middle">{esc(ylabel)}</text>']
 for i,(rate,color) in enumerate([(1000000,'#1565c0'),(2000000,'#c45d00'),(4000000,'#00845a')]):
  rs=[r for r in rows if r['rx_rate_hz']==rate and r['tx_power_dbm'] is not None and r[metric]]
  pts=' '.join(f"{x(r['tx_power_dbm'])},{y(r[metric]['mean'])}" for r in rs)
  a.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2"/>')
  for r in rs:
   px=x(r['tx_power_dbm']);s=r[metric];py=y(s['mean']);off=(i-1)*4
   a += [f'<path d="M {px+off} {y(s["min"])} V {y(s["max"])} M {px+off-3} {y(s["min"])} h 6 M {px+off-3} {y(s["max"])} h 6" stroke="{color}" opacity="0.65" fill="none"/>',
    f'<circle cx="{px}" cy="{py}" r="3.5" fill="{color}"><title>RX {rate//1000000} MS/s, TX {r["tx_power_dbm"]} dBm: mean {s["mean"]:.3f}, range {s["min"]:.3f}–{s["max"]:.3f}</title></circle>']
  lx=L+i*235
  a += [f'<line x1="{lx}" y1="{H-30}" x2="{lx+25}" y2="{H-30}" stroke="{color}" stroke-width="3"/>',f'<text x="{lx+33}" y="{H-25}">RX {rate//1000000} MS/s</text>']
 a += [f'<text x="{W-R}" y="{H-25}" text-anchor="end" font-size="12">Bars: observed min–max, not uncertainty</text>','</svg>']
 (dest/filename).write_text('\n'.join(a))
plot('sinad_db','SINAD during the wired TX power sweep','Estimated SINAD (dB)',0,30,list(range(0,31,5)),'sinad.svg')
plot('rssi_dbm','RSSI during the wired TX power sweep','Raw signed RSSI register (dBm)',-115,-60,list(range(-110,-59,10)),'rssi.svg')
print('Analyzed',len(rows),'rows; wide SINAD range by RX rate:')
for rate in meta['rx_rates_hz']:
 rs=[r for r in rows if r['rx_rate_hz']==rate and r['tx_power_dbm'] is not None]
 print(rate, sum(r['wide_sinad_range'] for r in rs),'/',len(rs))
