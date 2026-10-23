function [y,err1,err2] = identify_vbemf(ud,uq,id,iq,we,Vstate)
%#codegen
% 论文式12.25～12.28：两组位置误差与逐样本递推最小二乘辨识。
% 输入为五路同周期、同系数的50 Hz低通信号，采样周期均为100微秒。
% 在辨识支路内对五路信号使用相同的一电周期滑动平均，抑制周期纹波。
% y的前17列保持原含义；18～27列为电压稳态诊断，未计算参数为NaN。
persistent buf count oldState ready result aerr berr finished
persistent angleK angDone sumDiff m theta P iteration checked window cursor filled
persistent cycleBuf cycleSum cycleHead cycleCount cycleN rawWindow
if isempty(count)
    buf=zeros(2000,5,6); count=0; oldState=-1; ready=false(6,1);
    result=zeros(27,1); result([3 4 6])=NaN;
    aerr=zeros(400,1); berr=zeros(400,1);
    finished=false; angleK=0; angDone=false; sumDiff=zeros(2,2);
    m=zeros(6,5); theta=zeros(2,1); P=10*eye(2);
    iteration=0; checked=false; window=zeros(2000,5); cursor=0; filled=0;
    cycleBuf=zeros(1257,5); cycleSum=zeros(1,5);
    cycleHead=0; cycleCount=0; cycleN=1; rawWindow=zeros(2000,2);
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
    cycleBuf(:)=0; cycleSum(:)=0; cycleHead=0; cycleCount=0; cycleN=1;
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
    result(27)=Vstate;
    % 阶段开始0.1秒后，用滤波后的电角速度确定一电周期长度并锁存。
    % N约等于2π/(|we|*Ts)。只对辨识数据处理，不改变控制环及任务周期。
    % 每次切换阶段均清空平均器，防止混入上一个注入状态的数据。
    if count==1000
        cycleN=min(1257,max(50,round(2*pi/(max(abs(we),50)*1e-4))));
        cycleBuf(:)=0; cycleSum(:)=0; cycleHead=0; cycleCount=0;
    end
    smooth=u;
    if count>=1000
        cycleHead=mod(cycleHead,cycleN)+1;
        cycleSum=cycleSum+u-cycleBuf(cycleHead,:);
        cycleBuf(cycleHead,:)=u; cycleCount=min(cycleN,cycleCount+1);
        smooth=cycleSum/cycleCount;
    end
    result(26)=cycleN;
    % 至少等待0.2秒，并确保周期平均器已填满，再保存最新0.2秒窗口。
    % 不能仅靠固定计时推进状态；窗口必须通过稳态及工作点匹配检查。
    if count>max(2000,999+cycleN)
        cursor=mod(cursor,2000)+1; window(cursor,:)=smooth;
        rawWindow(cursor,:)=u(1:2);
        filled=min(2000,filled+1);
    end
    if filled==2000 && mod(count,200)==0
        W=zeros(2000,5);
        for n=1:2000, W(n,:)=window(mod(cursor+n-1,2000)+1,:); end
        [meanW,reason,drift,diagV]=check_window(W);
        result(10)=drift; result(17)=reason;
        rawMean=sum(rawWindow,1)/2000;
        for j=1:2
            result(17+j)=sqrt(sum((rawWindow(:,j)-rawMean(j)).^2)/2000);
        end
        result(20:25)=diagV;
        if reason==0 && slot~=1 && slot~=4
            base=1+3*(slot>3);
            mismatch=norm(meanW(3:4)-m(base,3:4));
            result(11)=max(result(11),mismatch);
            if mismatch>max(0.02,0.005*norm(m(base,3:4))) || ...
                    abs(meanW(5)-m(base,5))>max(1,0.005*abs(m(base,5)))
                reason=-6; result(17)=reason;
            end
        end
        % 式12.25要求正负注入时的dq电流与转速相同，不能只检查各自平稳。
        % 不匹配则继续滚动取窗，不使用错误的电压差计算位置误差。
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
    % 同一工作点等待超过3秒仍不合格则明确中止，不使用被拒绝的数据。
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
        % 位置误差更新400次，每次消耗五个尚未处理的存储样本。
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
            % 递推之前先检查两组数据的可观测性，此处不预先求解参数。
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
        % 按A1、B1、A2、B2……A2000、B2000顺序读取两组基准数据。
        % 每次调用只使用一个观测量，执行一次实际递推，不反复重放同一批数据。
        % 正负注入数据用于估计位置误差，基准数据用于L和磁链辨识。
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
        % 用Joseph形式更新协方差，测量方差及遗忘因子均为1。
        % 滑动平均后的相邻样本存在相关性，协方差不解释为统计置信区间。
        A=eye(2)-gain*phi'; P=A*P*A'+gain*gain'; P=0.5*(P+P');
        result(3)=0.01*theta(1); result(4)=0.2*theta(2);
        result(8)=3; result(12)=iteration; result(13)=innovation;
        % 每次L和磁链递推后，同步更新当前Rs估计；最终有效性单独判断。
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

function [meanW,reason,maxDrift,diagV] = check_window(W)
% 电压均值漂移与周期平均后的剩余纹波分别判断，避免混用同一个错误码。
% 原始纹波先经相同的电周期平均；慢漂移不会被此单位直流增益滤波消除。
% 诊断六列依次为：ud/uq剩余标准差、ud/uq漂移门限比、ud/uq纹波门限比。
meanW=sum(W,1)/2000; reason=0; maxDrift=0;
diagV=zeros(6,1);
for j=1:5
    sd=sqrt(sum((W(:,j)-meanW(j)).^2)/2000);
    drift=abs(sum(W(1:1000,j))/1000-sum(W(1001:2000,j))/1000);
    if j<=2
        st=max(0.10,0.005*abs(meanW(j)));
        dt=max(0.05,0.002*norm(meanW(1:2)));
        maxDrift=max(maxDrift,drift); code=-13;
        diagV(j)=sd; diagV(2+j)=drift/dt; diagV(4+j)=sd/st;
        % -3只表示均值仍在慢漂移；-13表示周期平均后仍有过大纹波。
        if drift>dt && (reason==0 || reason==-13), reason=-3; end
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
