function test_identification
% Actual MATLAB functions; synthetic equations are NOT switched-model tests.
folder=fileparts(mfilename('fullpath')); addpath(folder); rng(42);
R=1.8665;L=.00896;psi=.21;
names={'ideal','inverter_error','noise','reverse_speed','rank_deficient', ...
    'incomplete','current_mismatch','Rs_unobservable','nonfinite', ...
    'voltage_drift','ill_conditioned','delayed_settling'};
for c=1:numel(names)
    clear identify_vbemf
    ids=[0 -1];iqs=[4 4];deltas=[.08 -.12];w=220;dead=0;
    if c==2,dead=1.2;end
    if c==4,w=-220;end
    if c==5,ids=[0 0];deltas=[.08 .08];end
    if c==8,iqs(2)=sqrt(15);end
    if c==11,ids=[0 -1e-6];deltas=[.08 .08];end
    y=identify_vbemf(0,0,0,0,w,0);
    assert(all(isnan(y([3 4 6]))),'Uncomputed estimates must not be valid zeros');
    states=[2 3 4 6 7 8];failed=false;
    for s=1:6
        if c==6 && s==3,continue;end
        g=1+(s>3);k=mod(s-1,3);shift=0;
        if k==1,shift=.01;elseif k==2,shift=-.01;end
        di=ids(g);qi=iqs(g);d=deltas(g)+shift;
        if c==7 && s==2,di=di+.4;end
        D=(4/pi)*[di qi]/hypot(di,qi);
        ud=R*di-w*L*qi-w*psi*sin(d)+D(1)*dead;
        uq=R*qi+w*L*di+w*psi*cos(d)+D(2)*dead;
        accepted=0;
        for n=1:30000
            u=[ud uq di qi w];
            if c==3,u=u+[.01*randn .01*randn .001*randn .001*randn .03*randn];end
            if c==10 && s==1,u(1)=u(1)+.001*n;end
            if c==12 && s==1 && n>2000 && n<=3000,u(1)=u(1)+1;end
            y=identify_vbemf(u(1),u(2),u(3),u(4),u(5),states(s));
            if y(16)==states(s),accepted=n;break;end
            if y(16)<0,failed=true;break;end
        end
        if failed,break;end
        assert(accepted>=4000,'Collector accepted before settle+window completed');
        if c==12 && s==1,assert(accepted>=5000,'Nonsteady window was accepted instead of retried');end
    end
    if c==9
        y=identify_vbemf(NaN,0,0,0,w,8);failed=true;
    end
    if ~failed
        history=zeros(4401,3);
        for n=1:4401
            [y,e1,e2]=identify_vbemf(0,0,0,0,w,9);
            history(n,:)=[y(3) y(4) y(12)];
        end
        assert(numel(e1)==400 && numel(e2)==400);
    end
    good=c<=4 || c==8 || c==12;
    if good
        assert(y(5)==1,[names{c} ': expected valid L/psi']);
        assert(y(12)==4000 && all(diff(history(402:end,3))==1));
        assert(norm(history(402,1:2)-history(end,1:2))>1e-5,'Batch-latched estimate');
        tol=.002; if c==3,tol=.005;end
        assert(abs(y(3)/L-1)<tol && abs(y(4)/psi-1)<tol);
        assert(max(abs(y(1:2)-deltas(:)))<.002);
        if c==8,assert(y(7)==0 && isnan(y(6)));
        else
            rtol=.02;if c==3,rtol=.05;end
            assert(y(7)==1 && abs(y(6)/R-1)<rtol);
        end
    else
        assert(y(5)==0 && y(7)==0 && y(8)<0);
        assert(isnan(y(3)) && isnan(y(4)),'Rejected data produced estimates');
    end
    fprintf('PASS %-20s L=%g psi=%g R=%g status=%g wait=%g iter=%g\n',names{c},y(3),y(4),y(6),y(8),y(17),y(12));
end
% Actual PLL + collector, linked by the same one-tick ack as in the model.
clear pll_vbemf identify_vbemf
Ts=1e-4;W=220;E=46.2;th=0;ack=0;
seen=false(11,1);ticks=zeros(11,1);final=zeros(17,1);
for n=1:100000
    t=(n-1)*Ts;delta=W*t-th;
    [theta,speed,offset,state]=pll_vbemf(-E*sin(delta),E*cos(delta),double(t>=1),ack);
    th=mod(theta+Ts*speed,2*pi);seen(double(state)+1)=true;ticks(double(state)+1)=ticks(double(state)+1)+1;
    if state>=5 && state<=8,assert(offset==-1);else,assert(offset==0);end
    group=1+(state>=5 && state<=8);di=[0 -1];d0=[.08 -.12];shift=0;
    if state==3 || state==7,shift=.01;elseif state==4 || state==8,shift=-.01;end
    d=d0(group)+shift;dd=di(group);qq=4;
    ud=R*dd-W*L*qq-W*psi*sin(d);uq=R*qq+W*L*dd+W*psi*cos(d);
    final=identify_vbemf(ud,uq,dd,qq,W,double(state));ack=final(16);
end
assert(all(seen([1 3:10])) && ~seen(11),'Handshake phase sequence failed');
assert(all(ticks(3:9)>=4000));assert(final(5)==1 && final(12)==4000);
[~,~,off,state]=pll_vbemf(0,E,0,0);assert(state==0 && off==0);
fprintf('PASS coupled actual PLL/collector handshake, 4000 updates, offset and mode reset\n');
% A timer without a valid-window ack MUST NOT step to positive injection.
clear pll_vbemf
th=0;
for n=1:40000
    t=(n-1)*Ts;delta=W*t-th;
    [theta,speed,~,state]=pll_vbemf(-E*sin(delta),E*cos(delta),double(t>=1),0);
    th=mod(theta+Ts*speed,2*pi);
end
assert(state==2,'State advanced without accepted data');
[~,~,off,state]=pll_vbemf(0,E,1,-2);assert(state==10 && off==0);
fprintf('PASS no ack keeps state2; rejected-window ack aborts to state10 and removes injection\n');
end
