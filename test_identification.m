function test_identification
% Runs the ACTUAL .m implementations. Requires MATLAB or GNU Octave.
% Synthetic steady-state equation tests do not replace switched-model simulation.
base=fileparts(mfilename('fullpath')); addpath(base);
R=1.8665; L=8.96e-3; psi=0.21; rng(42);
names={'ideal','inverter_error','noise','reverse_speed','rank_deficient', ...
       'incomplete','current_drift','Rs_unobservable','nonfinite','voltage_drift','ill_conditioned'};
for c=1:numel(names)
    clear identify_vbemf
    ids=[0 -1]; iqs=[4 4]; deltas=[0.08 -0.12]; w=220;
    dead=0; noise=0;
    if c==2, dead=1.2; end
    if c==3, noise=0.005; end
    if c==4, w=-220; end
    if c==5, ids=[0 0]; deltas=[0.08 0.08]; end
    if c==8, iqs(2)=sqrt(4^2-1); end
    if c==11, ids=[0 -1e-6]; deltas=[0.08 0.08]; end
    identify_vbemf(0,0,0,0,w,0);
    seq=[2 3 4 6 7 8];
    for s=1:6
        if c==6 && s==3, continue; end
        g=1+(s>3); k=mod(s-1,3);
        shift=0; if k==1, shift=0.07; elseif k==2, shift=-0.07; end
        di=ids(g); qi=iqs(g); d=deltas(g)+shift;
        if c==7 && s==2, di=di+0.4; end
        D=(4/pi)*[di qi]/hypot(di,qi);
        ud=R*di-w*L*qi-w*psi*sin(d)+D(1)*dead;
        uq=R*qi+w*L*di+w*psi*cos(d)+D(2)*dead;
        for n=1:7000
            ripple=noise*sin(2*pi*n/17);
            if c==10 && s==1, ripple=ripple+0.001*n; end
            if c==3
                identify_vbemf(ud+ripple+0.01*randn,uq-ripple+0.01*randn, ...
                    di+0.001*randn,qi+0.001*randn,w+0.03*randn,seq(s));
            else
                identify_vbemf(ud+ripple,uq-ripple,di,qi,w,seq(s));
            end
        end
    end
    if c==9, identify_vbemf(NaN,0,0,0,w,8); end
    history=zeros(4401,3);
    for k=1:4401
        [y,e1,e2]=identify_vbemf(0,0,0,0,w,9);
        history(k,:)=[y(3) y(4) y(12)];
    end
    assert(all(isfinite(y)) && numel(e1)==400 && numel(e2)==400);
    if c<=4 || c==8
        assert(y(5)==1, [names{c} ': invalid L/psi']);
        assert(y(12)==4000, 'RLS did not consume 4000 individual observations');
        assert(all(diff(history(402:end,3))==1), 'RLS iteration must increase once per call');
        assert(norm(history(402,1:2)-history(end,1:2))>1e-5, 'Estimate was batch latched');
        assert(abs(y(3)-L)/L<0.002, 'L error');
        assert(abs(y(4)-psi)/psi<0.002, 'psi error');
        assert(max(abs(y(1:2)-deltas(:)))<0.002, 'angle error');
        if c==8
            assert(y(7)==0, 'Unobservable Rs accepted');
        else
            assert(y(7)==1, 'Rs not identified');
            assert(abs(y(6)-R)/R<0.02, 'Rs error');
        end
    else
        assert(y(5)==0 && y(7)==0, [names{c} ': bad data accepted']);
    end
    fprintf('PASS %-20s L=%g psi=%g R=%g flags=%g/%g status=%g iter=%g\n',names{c},y(3),y(4),y(6),y(5),y(7),y(8),y(12));
end
% PLL sequencing test with a synthetic rotating EMF; no mechanical plant.
clear pll_vbemf
Ts=1e-4; W=220; E=46.2; th=0; seen=false(10,1); ticks=zeros(10,1);
% Use the returned speed to align the next synthetic EMF sample.
clear pll_vbemf
th=0; seen=false(10,1); ticks=zeros(10,1);
for n=1:75000
    t=(n-1)*Ts; delta=W*t-th;
    [theta,speed,off,state]=pll_vbemf(-E*sin(delta),E*cos(delta),double(t>=1));
    th=mod(theta+Ts*speed,2*pi);
    seen(double(state)+1)=true; ticks(double(state)+1)=ticks(double(state)+1)+1;
    if state>=5 && state<=8, assert(off==-1); else, assert(off==0); end
end
assert(all(seen([1 3:10])), 'State sequence incomplete');
assert(all(ticks(3:9)==7000), 'Dwell/estimator timing mismatch');
[~,~,off,state]=pll_vbemf(0,E,0); assert(off==0 && state==0);
fprintf('PASS PLL state sequence, seven 7000-tick stages, offset and reset\n');
end
