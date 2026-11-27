import pathlib,json,sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
reverse=pathlib.Path(sys.argv[1]);forward=pathlib.Path(sys.argv[2])
fig,axs=plt.subplots(2,1,figsize=(12,7),sharex=True,layout='constrained')
for root,label,color in [(forward,'Forward: original TX → repaired #2 RX','#2563eb'),(reverse,'Reverse: repaired #2 TX → original RX','#ea580c')]:
 rows=[r for s in (root/'results.jsonl').read_text().splitlines() if (r:=json.loads(s))['phase']=='matrix' and r['sample_rate_hz']==1000000]
 rows.sort(key=lambda r:r['frequency_hz'])
 for ax,key in zip(axs,['rssi_dbm','sinad_db']):
  ax.plot([r['frequency_hz']/1e9 for r in rows],[r[key]['mean'] if r[key] else np.nan for r in rows],'.-',lw=1,ms=3,label=label,color=color)
for ax in axs:
 ax.grid(alpha=.25);ax.set_xlim(.03,6)
 for hz in [2.385,2.495]:ax.axvline(hz,color='gray',ls=':',lw=.8)
axs[0].set_ylabel('Mean reported RSSI (dBm)');axs[0].legend(loc='lower right',fontsize=9)
axs[1].set_ylabel('Mean estimated SINAD (dB)');axs[1].set_ylim(bottom=0);axs[1].set_xlabel('Frequency (GHz)')
fig.suptitle('Reciprocal link comparison at 1 MS/s — 27 November 2026\n0 dBm requested TX; nominal 90 dB attenuators; added 2 m RG179; ~2 m separation')
for ext in ['png','svg']:fig.savefig(reverse/'analysis'/('reciprocal-1msps.'+ext),dpi=180)
