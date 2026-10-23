function [y,err1,err2] = identify_vbemf(ud,uq,id,iq,we,Vstate)
%#codegen
% Inputs pass matched 50 Hz low-pass filters at their ACTUAL Ts=100 us.
% Sources already have rate Ts; this cannot recover any upstream aliasing.
% Columns 1:7 retain v1 meanings. Diagnostics 8:15:
% status, condition, max voltage drift, max dq mismatch, RLS iteration,
% normalized innovation, relative Rs denominator, samples in current stage.
% status: 0 waiting, 1 collecting, 2 angle pass, 3 RLS, 20 valid,
% 10 valid L/psi only; negative values describe rejected data (see README).
persistent buf count oldState ready result aerr berr finished
persistent angleK angDone sumDiff m theta P iteration checked
if isempty(count)
    buf=zeros(2000,5,6); count=0; oldState=-1; ready=false(6,1);
    result=zeros(15,1); aerr=zeros(400,1); berr=zeros(400,1);
    finished=false; angleK=0; angDone=false; sumDiff=zeros(2,2);
    m=zeros(6,5); theta=zeros(2,1); P=10*eye(2);
    iteration=0; checked=false;
end
u=[ud uq id iq we];
if ~all(isfinite(u)) || ~isfinite(Vstate)
    ready(:)=false; result(:)=0; result(8)=-1;
    count=0; oldState=-1; finished=false; checked=false;
    y=result; err1=aerr; err2=berr; return;
end
if Vstate~=oldState
    count=0; oldState=Vstate;
    if Vstate==0 || Vstate==2
        ready(:)=false; result(:)=0; aerr(:)=0; berr(:)=0;
        finished=false; angleK=0; angDone=false; sumDiff(:)=0;
        theta(:)=0; P=10*eye(2); iteration=0; checked=false;
    end
end
slot=0;
if Vstate==2, slot=1;
elseif Vstate==3, slot=2;
elseif Vstate==4, slot=3;
elseif Vstate==6, slot=4;
elseif Vstate==7, slot=5;
elseif Vstate==8, slot=6;
end
% Every state is 0.7 s: allow 0.5 s settling, then store 0.2 s (2000 samples).
if slot>0 && count<7000
    result(8)=1; count=count+1; result(15)=count;
    if count>5000, buf(count-5000,:,slot)=u; end
    if count==7000, ready(slot)=true; end
end
if Vstate==9 && ~finished
    if ~all(ready)
        if result(8)>=0, result(8)=-2; end
        finished=true;
    elseif ~checked
        checked=true; quality=true; reason=0;
        for s=1:6
            for j=1:5, m(s,j)=sum(buf(:,j,s))/2000; end
            % Both voltage and current must be stationary. A fixed wait alone
            % does not establish the steady-state assumption of eq.12.21.
            for j=1:5
                sd=sqrt(sum((buf(:,j,s)-m(s,j)).^2)/2000);
                drift=abs(sum(buf(1:1000,j,s))/1000-sum(buf(1001:2000,j,s))/1000);
                if j<=2
                    st=max(0.10,0.005*abs(m(s,j)));
                    dt=max(0.05,0.002*norm(m(s,1:2)));
                    result(10)=max(result(10),drift); code=-3;
                elseif j<=4
                    st=max(0.01,0.005*norm(m(s,3:4)));
                    dt=max(0.015,0.005*norm(m(s,3:4))); code=-4;
                else
                    st=max(0.3,0.002*abs(m(s,j)));
                    dt=max(0.5,0.003*abs(m(s,j))); code=-5;
                end
                if (sd>st || drift>dt) && quality
                    quality=false; reason=code;
                end
            end
            if abs(m(s,5))<50 && quality, quality=false; reason=-5; end
        end
        for g=0:1
            b=1+3*g;
            for s=b+1:b+2
                mismatch=norm(m(s,3:4)-m(b,3:4));
                result(11)=max(result(11),mismatch);
                if (mismatch>max(0.02,0.005*norm(m(b,3:4))) || ...
                        abs(m(s,5)-m(b,5))>max(1,0.005*abs(m(b,5)))) && quality
                    quality=false; reason=-6;
                end
            end
        end
        if quality, result(8)=2;
        else, result(8)=reason; finished=true;
        end
    elseif ~angDone
        % 400 actual angle updates; each consumes five new stored samples.
        angleK=angleK+1;
        for g=1:2
            pos=2+3*(g-1); neg=pos+1;
            for n=(angleK-1)*5+1:angleK*5
                sumDiff(g,1)=sumDiff(g,1)+buf(n,1,pos)-buf(n,1,neg);
                sumDiff(g,2)=sumDiff(g,2)+buf(n,2,pos)-buf(n,2,neg);
            end
            d=atan2(sumDiff(g,2),sumDiff(g,1));
            if d>pi/2, d=d-pi; elseif d< -pi/2, d=d+pi; end
            result(g)=d;
            if g==1, aerr(angleK)=d; else, berr(angleK)=d; end
        end
        if angleK==400
            angDone=true;
            for g=1:2
                if norm(sumDiff(g,:))/2000<0.2 || abs(result(g))>0.6
                    result(8)=-7; finished=true;
                end
            end
            % Check observability BEFORE replaying, without solving for theta.
            H=zeros(2,2);
            for g=1:2
                b=1+3*(g-1); di=m(b,3); qi=m(b,4); w=m(b,5);
                row=[0.01*w*(di^2+qi^2), ...
                    0.2*w*(di*cos(result(g))+qi*sin(result(g)))];
                H(g,:)=row/max(norm(row),1e-12);
            end
            detH=H(1,1)*H(2,2)-H(1,2)*H(2,1);
            tau=sum(sum(H.^2));
            condH=(tau+sqrt(max(0,tau*tau-4*detH*detH)))/(2*max(abs(detH),1e-12));
            result(9)=condH;
            if condH>100 || abs(detH)<1e-6
                result(8)=-8; finished=true;
            end
        end
    else
        % Revisit stored BASELINE samples, A1,B1,A2,B2,...,A2000,B2000.
        % Each call consumes exactly ONE observation and performs ONE RLS update.
        % Injected samples determine the position errors, not the baseline model.
        iteration=iteration+1;
        g=1+mod(iteration-1,2); n=floor((iteration-1)/2)+1;
        b=1+3*(g-1); di=buf(n,3,b); qi=buf(n,4,b); w=buf(n,5,b);
        raw=[0.01*w*(di^2+qi^2); ...
            0.2*w*(di*cos(result(g))+qi*sin(result(g)))];
        scale=max(norm(raw),1e-12); phi=raw/scale;
        voltage=(buf(n,2,b)*di-buf(n,1,b)*qi)/scale;
        gain=P*phi/(1+phi'*P*phi);
        innovation=voltage-phi'*theta;
        theta=theta+gain*innovation;
        % Joseph covariance update (measurement variance 1, forgetting factor 1).
        % Finite stored data: no artificial exponential forgetting or replay loops.
        A=eye(2)-gain*phi'; P=A*P*A'+gain*gain'; P=0.5*(P+P');
        result(3)=0.01*theta(1); result(4)=0.2*theta(2);
        result(8)=3; result(12)=iteration; result(13)=innovation;
        % Show the Rs value implied by EACH current L/psi iterate as well.
        D=zeros(2,1); z=zeros(2,1);
        for h=1:2
            bb=1+3*(h-1); dd=m(bb,3); qq=m(bb,4); ww=m(bb,5);
            D(h)=(4/pi)*qq/max(hypot(dd,qq),1e-8);
            z(h)=m(bb,2)-ww*result(3)*dd-ww*result(4)*cos(result(h));
        end
        den=D(2)*m(1,4)-D(1)*m(4,4);
        rsScale=abs(D(2)*m(1,4))+abs(D(1)*m(4,4));
        result(14)=abs(den)/max(rsScale,1e-12);
        rsObservable=rsScale>1e-6 && abs(den)>max(1e-3,0.003*rsScale);
        if rsObservable, result(6)=(D(2)*z(1)-D(1)*z(2))/den; end
        if iteration==4000
            finished=true;
            if all(isfinite(theta)) && result(3)>1e-5 && result(3)<0.2 && ...
                    result(4)>1e-3 && result(4)<2
                result(5)=1; result(8)=10;
                if rsObservable && isfinite(result(6)) && result(6)>0 && result(6)<100
                    result(7)=1; result(8)=20;
                end
            else
                result(8)=-9;
            end
        end
    end
end
y=result; err1=aerr; err2=berr;
end
