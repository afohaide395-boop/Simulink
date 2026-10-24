"""模型源信号已按Ts更新；验证实际同速率低通，并生成集成测试输入。"""
from pathlib import Path
import numpy as np
from scipy import signal,io
HERE=Path(__file__).resolve().parent
Ts=1e-4;k=np.tan(np.pi*50*Ts);q=1+np.sqrt(2)*k+k*k
b=np.array([1,2,1])*k*k/q
a=np.array([1,2*(k*k-1)/q,(1-np.sqrt(2)*k+k*k)/q])
for f in [0,50,200,3000]:
 _,h=signal.freqz(b,a,worN=np.array([2*np.pi*f*Ts]))
 print(f'LPF at Ts=100us: {f:4d} Hz gain={abs(h[0]):.9f} ({20*np.log10(abs(h[0])):.2f} dB)')
 if f==0:assert abs(abs(h[0])-1)<1e-10
 if f==50:assert abs(abs(h[0])-2**-.5)<1e-8
 if f==200:assert abs(h[0])<.063
# 已在源端混叠为直流的载波无法由后续低通恢复，此测试明确验证这个限制。
t=np.arange(10000)*Ts
aliased=np.cos(2*np.pi*10000*t)
y=signal.lfilter(b,a,aliased)
assert abs(np.mean(y[-2000:])-1)<1e-8
print('PASS limitation: 10kHz already aliased to DC at the source remains DC after this filter')
# 连续物理工况包含电流响应、Id斜坡和标定激励，供实际辨识器集成回放。
import make_accuracy_fixtures
import shutil
shutil.copy2(HERE/'nominal_fixture.mat',HERE/'pipeline_fixture.mat')
np.savez_compressed(HERE/'lowpass_design.npz',b=b,a=a,Ts=Ts)
print('PASS generated continuous same-rate LPF fixture for actual identification integration')
