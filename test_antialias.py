"""High-rate synthetic acquisition fixture; does not simulate the motor drive."""
from pathlib import Path
import numpy as np
from scipy import signal,io
HERE=Path(__file__).resolve().parent
fast=1e-6; Ts=1e-4; k=np.tan(np.pi*50*fast); q=1+np.sqrt(2)*k+k*k
b=np.array([1,2,1])*k*k/q
a=np.array([1,2*(k*k-1)/q,(1-np.sqrt(2)*k+k*k)/q])
# Numeric frequency response, independent of the steady-state fixture.
for f in [0,50,200,10000]:
 _,h=signal.freqz(b,a,worN=np.array([2*np.pi*f*fast]))
 print(f'Butterworth {f:5d} Hz: gain={abs(h[0]):.9f} ({20*np.log10(abs(h[0])):.2f} dB)')
 if f==0: assert abs(abs(h[0])-1)<1e-7
 if f==50: assert abs(abs(h[0])-2**-.5)<1e-6
 if f==10000: assert abs(h[0])<3e-5
# At control sampling frequency, a synchronous 10 kHz component aliases to DC.
t=np.arange(1000000)*fast
carrier=np.cos(2*np.pi*10000*t)
raw=carrier[::100]
filtered=signal.lfilter(b,a,carrier)[::100]
alpha=1-np.exp(-2*np.pi*50*Ts)
old=signal.lfilter([alpha],[1,-(1-alpha)],raw)
old_bias=abs(np.mean(old[-2000:])); new_bias=abs(np.mean(filtered[-2000:]))
assert old_bias>.99 and new_bias<3e-5
print(f'PASS synchronous-carrier alias example: post-sample LPF bias={old_bias:.8f}, pre-sample LPF bias={new_bias:.8f}')
# Full acquisition duration including transition state 5. True dq currents are
# held in each triplet. Raw measurement channels include PWM and 6fe harmonics.
R=1.8665; L=.00896; psi=.21; w=220; states=[2,3,4,5,6,7,8]
means=[]
for state in states:
 g=0 if state<=4 else 1
 di=[0,-1][g]; qi=4; d=[.08,-.12][g]
 if state in [3,7]:d+=.07
 if state in [4,8]:d-=.07
 means.append([R*di-w*L*qi-w*psi*np.sin(d),R*qi+w*L*di+w*psi*np.cos(d),di,qi,w])
per=700000; total=per*len(states); time=np.arange(total)*fast
stages=np.repeat(np.arange(len(states)),per)
M=np.asarray(means); sampled=np.empty((total//100,5)); unfiltered=np.empty_like(sampled)
for j,(pwm,harmonic) in enumerate(zip([1.5,.8,.3,.4,0],[.6,.6,.03,.04,.4])):
 x=M[stages,j]+pwm*np.cos(2*np.pi*10000*time+0.1*j)+harmonic*np.sin(6*w*time+0.3*j)
 sampled[:,j]=signal.lfilter(b,a,x)[::100]
 unfiltered[:,j]=x[::100]
slow_states=np.repeat(states,7000)
io.savemat(HERE/'pipeline_fixture.mat',{'samples':sampled,'states':slow_states[:,None]},do_compression=True)
# Measure residual waveform RMS and mean bias only in settled final windows.
for j in range(5):
 errors=[]
 for s in range(7):errors.extend(sampled[s*7000+5000:(s+1)*7000,j]-M[s,j])
 print(f'Filtered {['ud','uq','id','iq','we'][j]} RMS ripple = {np.std(errors):.6g}')
np.savez_compressed(HERE/'antialias_design.npz',b=b,a=a,fast=fast,Ts=Ts)
print('PASS generated high-rate filtered fixture with known R,L,psi and state transitions')
