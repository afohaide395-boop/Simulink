"""生成连续且满足电压方程的闭环工况数据，仅供函数验证，不替代Simulink。"""
from pathlib import Path
import numpy as np
from scipy.signal import butter,sosfilt
from scipy.io import savemat
HERE=Path(__file__).resolve().parent
Ts=1e-4;t=np.arange(46000)*Ts
stage=np.repeat([2,3,4,5,6,7,8],[6000,6000,6000,10000,6000,6000,6000])
idbase=np.zeros_like(t);ix=t>=1.8;idbase[ix]=-np.minimum((t[ix]-1.8)/.1,1)
cases=[('nominal',1.8665,.00896,.21,209.44,.9,0,0,0,0),
       ('inverter',1.8665,.00896,.21,209.44,.9,1.2,0,0,0),
       ('different_motor',2.2,.0065,.17,180,1.1,.8,0,0,0),
       ('reverse',1.4,.011,.25,-190,-1.1,.7,0,0,0),
       ('noise',1.8665,.00896,.21,209.44,.9,.5,.02,0,0),
       ('speed_ripple',1.8665,.00896,.21,209.44,.9,0,.01,.1,0),
       ('harmonics',1.8665,.00896,.21,209.44,.9,.8,0,0,.12),
       ('positive_angles',.95,.012,.18,180,1.3,.5,0,0,0)]
rng=np.random.default_rng(812)
for name,R,L,psi,w,iqr,drop,noise,ripple,harmonic in cases:
    target=.02+.05*idbase
    if name=='positive_angles':target=.04-.03*idbase
    target[(stage==3)|(stage==7)]+=.01;target[(stage==4)|(stage==8)]-=.01
    delta=np.zeros_like(t);delta[0]=target[0]
    for k in range(1,len(t)):delta[k]=delta[k-1]+Ts/.02*(target[k]-delta[k-1])
    real_speed=w+ripple*np.sin(2*np.pi*.3*t)
    rotor_angle=np.cumsum(real_speed)*Ts
    window=np.zeros_like(t);pulse=(t>=1.95)&(t<=2.15)
    window[pulse]=np.sin(np.pi*(t[pulse]-1.95)/.2)**2
    idref=idbase+.2*window*np.cos(rotor_angle-delta)
    iq=(iqr+idref*np.sin(delta))/np.cos(delta)
    # 谐波电流对应的电压由同一电机方程生成，避免只加电压噪声而不包含电流响应。
    idref+=harmonic*.7*np.sin(6*rotor_angle)
    iq+=harmonic*np.sin(6*rotor_angle+.4)
    estimated_speed=real_speed-np.gradient(delta,Ts)
    hd=np.gradient(idref,Ts)-estimated_speed*iq
    hq=np.gradient(iq,Ts)+estimated_speed*idref
    D=4/np.pi*np.column_stack([idref,iq])/np.hypot(idref,iq)[:,None]
    ud=R*idref+L*hd-real_speed*psi*np.sin(delta)+drop*D[:,0]
    uq=R*iq+L*hq+real_speed*psi*np.cos(delta)+drop*D[:,1]
    raw=np.column_stack([ud,uq,idref,iq,estimated_speed])
    if noise:
        raw+=rng.normal(size=raw.shape)*np.array([noise,noise,noise*.05,noise*.05,noise])
    samples=sosfilt(butter(2,50,fs=1/Ts,output='sos'),raw,axis=0)
    savemat(HERE/(name+'_fixture.mat'),{'samples':samples,'states':stage[:,None],
        'truth':np.array([L,psi,R,drop]),'name':name},do_compression=True)
print('已生成8组连续电机方程记录，包含Id斜坡/标定激励、位置误差瞬态、iq调节及同速率低通。')
