"""Reproducible analysis of the completed, locally archived Pi 4 sweep."""
import collections,csv,gzip,hashlib,json,math,pathlib,statistics,sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import numpy as np
BASE=pathlib.Path(__file__).resolve().parent
ROOT=pathlib.Path(sys.argv[1]) if len(sys.argv)>1 else BASE.parent
DEST=ROOT/'analysis';DEST.mkdir(exist_ok=True)
rows=[json.loads(line) for line in (ROOT/'results.jsonl').read_text().splitlines()]
rs=[r for r in rows if r['phase']=='matrix'];pre=[r for r in rows if r['phase']=='preflight']
RATES=sorted({r['sample_rate_hz'] for r in rs},reverse=True);FREQS=sorted({r['frequency_hz'] for r in rs});lookup={(r['frequency_hz'],r['sample_rate_hz']):r for r in rs}
assert len(rs)==1376 and len(FREQS)==172 and len(RATES)==8 and len(lookup)==len(rs)
assert all((f,s) in lookup for f in FREQS for s in RATES)
assert all(r['tx_power_dbm']==0 for r in rs)
assert json.loads((ROOT/'matrix-verification.json').read_text())['passed']
verification=json.loads((ROOT/'verification.json').read_text());assert verification['passed']
labels={4000000:'4',2000000:'2',1333333:'1.333333',1000000:'1',800000:'0.8',666667:'0.666667',500000:'0.5',400000:'0.4'}
colors=plt.get_cmap('tab10').colors
plt.rcParams.update({'font.size':10,'axes.titlesize':12,'axes.labelsize':11,'figure.facecolor':'white','axes.spines.top':False,'axes.spines.right':False,'svg.fonttype':'none'})
def vals(rate,key):return np.array([lookup[(f,rate)][key]['mean'] if lookup[(f,rate)][key] else np.nan for f in FREQS])
def save(fig,name):
    fig.savefig(DEST/(name+'.png'),dpi=180,bbox_inches='tight');fig.savefig(DEST/(name+'.svg'),bbox_inches='tight');plt.close(fig)
    svg=DEST/(name+'.svg');svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
def decorate(ax,ylabel):ax.set_ylabel(ylabel);ax.grid(alpha=.2);ax.set_axisbelow(True)
# Overview: gaps remain gaps where the estimator reported no number.
fig,axs=plt.subplots(2,1,figsize=(13.5,8),sharex=True,layout='constrained')
for i,rate in enumerate(RATES):
 for ax,key in zip(axs,['rssi_dbm','sinad_db']):ax.plot(np.array(FREQS)/1e9,vals(rate,key),'.-',lw=.85,ms=2.5,color=colors[i],alpha=.85,label=labels[rate])
for ax,key in zip(axs,['rssi_dbm','sinad_db']):
 flagged=[r for r in rs if not r['transport_clean'] and r[key]]
 if flagged:ax.scatter([r['frequency_hz']/1e9 for r in flagged],[r[key]['mean'] for r in flagged],marker='x',s=45,c='black',lw=1.5,zorder=5)
for ax in axs:
 for boundary in [2.385,2.495]:ax.axvline(boundary,color='#777777',ls=':',lw=.8)
 ax.set_xlim(.03,6)
decorate(axs[0],'Mean RSSI (dBm)');decorate(axs[1],'Mean estimated SINAD (dB)')
axs[0].set_title('Pi 4 link — ~2 m separation, added 2 m RG179, nominal 90 dB attenuators')
axs[0].legend(title='Matched TX/RX rate (MS/s)',ncol=4,loc='upper left',fontsize=9,title_fontsize=9)
axs[1].set_xlabel('Requested frequency (GHz)');axs[1].set_xticks(np.arange(0,6.1,.5))
axs[1].set_ylim(bottom=0)
save(fig,'frequency-overview')
# Heatmaps: frequency cells use midpoint edges on the actual nonuniform grid.
f=np.array(FREQS)/1e9;edges=np.r_[f[0],(f[:-1]+f[1:])/2,f[-1]]
fig,axs=plt.subplots(2,1,figsize=(13.5,8),sharex=True,layout='constrained')
for ax,key,cmap_name,title in zip(axs,['rssi_dbm','sinad_db'],['viridis','magma'],['RSSI (dBm)','Estimated SINAD (dB)']):
 data=np.vstack([vals(rate,key) for rate in RATES]);cmap=plt.get_cmap(cmap_name).copy();cmap.set_bad('#c7cbd1')
 im=ax.pcolormesh(edges,np.arange(9),np.ma.masked_invalid(data),cmap=cmap,shading='flat',vmin=np.floor(np.nanmin(data)) if key=='rssi_dbm' else 0,vmax=np.ceil(np.nanmax(data)),rasterized=True)
 ax.set_yticks(np.arange(8)+.5,[labels[r] for r in RATES]);ax.invert_yaxis();ax.set_ylabel('Matched rate (MS/s)');ax.set_title(title);fig.colorbar(im,ax=ax,label=title,pad=.015)
axs[0].set_title('RSSI (dBm) — actual frequency spacing');axs[1].set_title('Estimated SINAD (dB) — gray means unavailable')
axs[1].set_xlabel('Requested frequency (GHz)');axs[1].set_xticks(np.arange(0,6.1,.5));save(fig,'frequency-rate-heatmaps')
# Rate profiles at selected anchors. Fractional rate labels are rounded command values.
anchors=[30000000,430100000,1350000000,2399900000,2494900000,5999999000]
asc=sorted(RATES);fig,axs=plt.subplots(2,1,figsize=(12.5,8),sharex=True,layout='constrained')
for i,hz in enumerate(anchors):
 for ax,key in zip(axs,['rssi_dbm','sinad_db']):
  y=[lookup[(hz,rate)][key]['mean'] if lookup[(hz,rate)][key] else np.nan for rate in asc]
  ax.plot(np.array(asc)/1e6,y,'o-',ms=4,lw=1.2,color=colors[i],label=f'{hz/1e6:.3f}'.rstrip('0').rstrip('.')+' MHz')
for ax in axs:ax.set_xscale('log');ax.grid(alpha=.2)
axs[0].set_title('Sample-rate dependence at selected measured frequencies');axs[0].legend(ncol=3,fontsize=9)
axs[0].set_ylabel('Mean RSSI (dBm)');axs[1].set_ylabel('Mean estimated SINAD (dB)');axs[1].set_ylim(bottom=0)
axs[1].set_xticks(np.array(asc)/1e6,[labels[r] for r in asc]);axs[1].minorticks_off();axs[1].set_xlabel('Matched TX/RX sample rate (MS/s; logarithmic axis)');save(fig,'rate-profiles')
# Native modem path boundaries, where 100 kHz spacing is otherwise invisible.
fig,axs=plt.subplots(2,2,figsize=(13.5,8),layout='constrained')
for col,(lo,hi,bound) in enumerate([(2384.75,2385.25,2385),(2494.75,2495.25,2495)]):
 for i,rate in enumerate(RATES):
  for row,key in enumerate(['rssi_dbm','sinad_db']):
   ff=[h for h in FREQS if lo*1e6<=h<=hi*1e6];yy=[lookup[(h,rate)][key]['mean'] if lookup[(h,rate)][key] else np.nan for h in ff]
   axs[row,col].plot(np.array(ff)/1e6,yy,'o-',lw=1,ms=3,color=colors[i],label=labels[rate])
 for row in range(2):axs[row,col].axvline(bound,color='#666',ls=':');axs[row,col].grid(alpha=.2);axs[row,col].set_xlim(lo,hi)
 axs[0,col].set_title(f'{bound} MHz path boundary');axs[1,col].set_xlabel('Requested frequency (MHz)')
axs[0,0].set_ylabel('Mean RSSI (dBm)');axs[1,0].set_ylabel('Mean estimated SINAD (dB)')
handles,legend_labels=axs[0,0].get_legend_handles_labels()
fig.legend(handles,legend_labels,title='Matched rate (MS/s)',ncol=8,fontsize=9,title_fontsize=9,loc='upper center',bbox_to_anchor=(.5,1.08));save(fig,'path-boundary-zooms')
