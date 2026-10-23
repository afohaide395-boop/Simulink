function [y,err1,err2] = identify_vbemf(ud,uq,id,iq,we,Vstate)
%#codegen
% Paper eqs.12.25-12.28: two independent angle errors and sample-wise RLS.
% Inputs are matched 50 Hz low-pass channels, all at Ts=100 us.
% y(1:15) retain previous meanings; y16=accepted state acknowledgement,
% y17=latest waiting/rejection reason. Uncomputed parameters are NaN, not zero.
persistent buf count oldState ready result aerr berr finished
persistent angleK angDone sumDiff m theta P iteration checked window cursor filled
if isempty(count)
    buf=zeros(2000,5,6); count=0; oldState=-1; ready=false(6,1);
    result=zeros(17,1); result([3 4 6])=NaN;
    aerr=zeros(400,1); berr=zeros(400,1);
    finished=false; angleK=0; angDone=false; sumDiff=zeros(2,2);
    m=zeros(6,5); theta=zeros(2,1); P=10*eye(2);
    iteration=0; checked=false; window=zeros(2000,5); cursor=0; filled=0;
end
u=[ud uq id iq we];
if ~all(isfinite(u)) || ~isfinite(Vstate)
    ready(:)=false; result(:)=0; result([3 4 6])=NaN;
    result(8)=-1; result(17)=-1;
    if Vstate>=2 && Vstate<=8, result(16)=-Vstate; end
    finished=true; y=result; err1=aerr; err2=berr; return;
end
if Vstate~=oldState
    count=0; oldState=Vstate; cursor=0; filled=0; result(16)=0;
    if Vstate==0 || Vstate==2
        ready(:)=false; result(:)=0; result([3 4 6])=NaN;
        aerr(:)=0; berr(:)=0;
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
if slot>0 && ~ready(slot) && ~finished
    result(8)=1; count=count+1; result(15)=count;
    % Wait 0.2 s; then keep the latest rolling 0.2 s window. A fixed time
    % deadline no longer advances the injection state before data are valid.
    if count>2000
        cursor=mod(cursor,2000)+1; window(cursor,:)=u;
        filled=min(2000,filled+1);
    end
    if filled==2000 && mod(count,200)==0
        W=zeros(2000,5);
        for n=1:2000, W(n,:)=window(mod(cursor+n-1,2000)+1,:); end
        [meanW,reason,drift]=check_window(W);
        result(10)=drift; result(17)=reason;
        if reason==0 && slot~=1 && slot~=4
            base=1+3*(slot>3);
            mismatch=norm(meanW(3:4)-m(base,3:4));
            result(11)=max(result(11),mismatch);
            if mismatch>max(0.02,0.005*norm(m(base,3:4))) || ...
                    abs(meanW(5)-m(base,5))>max(1,0.005*abs(m(base,5)))
                reason=-6; result(17)=reason;
            end
        end
        % Eq.12.25 subtracts the +/- voltages at the SAME dq current and
        % speed. The negative phase must match the saved positive phase,
        % not just look individually smooth. Retake its rolling window.
        if reason==0 && (slot==3 || slot==6)
            plus=slot-1;
            if norm(meanW(3:4)-m(plus,3:4))>max(0.003,0.0005*norm(m(plus,3:4))) || ...
                    abs(meanW(5)-m(plus,5))>max(0.02,0.0001*abs(m(plus,5)))
                reason=-12; result(17)=reason;
            end
        end
        if reason==0
            buf(:,:,slot)=W; m(slot,:)=meanW; ready(slot)=true;
            result(16)=Vstate; result(17)=0;
        end
    end
    % Explicit abort after 3 s at one operating point. Never fabricate an
    % estimate from rejected data or continue to state 9 with missing samples.
    if count>=30000 && ~ready(slot)
        if result(17)==0, result(17)=-2; end
        result(8)=result(17); result(16)=-Vstate; finished=true;
    end
end
if Vstate==10 && ~finished
    result(8)=-10; result(17)=-10; finished=true;
end
if Vstate==9 && ~finished
    if ~all(ready)
        result(8)=-2; result(17)=-2; finished=true;
    elseif ~checked
        checked=true; result(8)=2;
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
                    result(8)=-7; result(17)=-7; finished=true;
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
                result(8)=-8; result(17)=-8; finished=true;
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
                result(8)=-9; result(17)=-9;
            end
        end
    end
end
y=result; err1=aerr; err2=berr;
end

function [meanW,reason,maxDrift] = check_window(W)
% Engineering steady-state guard for the paper's zero-derivative model.
meanW=sum(W,1)/2000; reason=0; maxDrift=0;
for j=1:5
    sd=sqrt(sum((W(:,j)-meanW(j)).^2)/2000);
    drift=abs(sum(W(1:1000,j))/1000-sum(W(1001:2000,j))/1000);
    if j<=2
        st=max(0.10,0.005*abs(meanW(j)));
        dt=max(0.05,0.002*norm(meanW(1:2)));
        maxDrift=max(maxDrift,drift); code=-3;
    elseif j<=4
        st=max(0.01,0.005*norm(meanW(3:4)));
        dt=max(0.015,0.005*norm(meanW(3:4))); code=-4;
    else
        st=max(0.3,0.002*abs(meanW(j)));
        dt=max(0.5,0.003*abs(meanW(j))); code=-5;
    end
    if (sd>st || drift>dt) && reason==0, reason=code; end
end
if abs(meanW(5))<50 && reason==0, reason=-5; end
end
