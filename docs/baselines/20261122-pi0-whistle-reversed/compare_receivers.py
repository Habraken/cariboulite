from pathlib import Path
import json
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
fig,axes=plt.subplots(2,1,figsize=(12,8),layout='constrained')
for label,folder,report in [('Reconstructed #1 RX; original TX','whistle-analysis','20261122-pi0-whistle'),('Original RX; reconstructed #1 TX + timing workaround','whistle-reverse-analysis','20261122-pi0-whistle-reversed')]:
 cap=ROOT/'build'/folder/'data/carrier/rx/capture'
 iq=np.fromfile(cap.with_suffix('.iq'),'<i2').reshape(-1,2);z=iq[:,0].astype(float)+1j*iq[:,1]
 f,p=signal.welch(z[1000000:],1000000,nperseg=262144,return_onesided=False);i=np.argsort(f);f,p=f[i],p[i]
 c=f[np.argmax(p)];power=np.trapezoid(p[np.abs(f-c)<80],f[np.abs(f-c)<80])
 axes[0].plot(f/1000,10*np.log10(np.maximum(p/power,1e-30)),label=label)
 info=json.loads(cap.with_suffix('.json').read_text());audio=np.fromfile(cap.with_suffix('.pcm'),'<i2').reshape(-1,info['channels'])[:,0].astype(float)/32768
 fa,pa=signal.welch(audio[48000:],48000,nperseg=65536)
 axes[1].plot(fa/1000,10*np.log10(np.maximum(pa,1e-30)),label=label)
axes[0].set_xlim(-5,5);axes[0].set_ylim(-95,0);axes[0].set_xlabel('Offset from RX tuning (kHz)');axes[0].set_ylabel('IQ PSD (dBc/Hz)');axes[0].set_title('Both receivers show an unwanted IQ line on a clean-carrier link')
axes[1].set_xlim(0,8);axes[1].set_ylim(-115,-40);axes[1].set_xlabel('Audio frequency (kHz)');axes[1].set_ylabel('Playback PCM PSD (dBFS/Hz)');axes[1].set_title('Both recover an audible whistle near 1.5–1.7 kHz')
for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.savefig(HERE/'analysis/receiver-comparison.png',dpi=160)
