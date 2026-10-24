function [y,err1,err2] = identify_vbemf_v9(ud,uq,id,iq,we,Vstate)
%#codegen
% 论文式12.25～12.28：两组位置误差与逐样本递推最小二乘辨识。
% 输入为五路同周期、同系数的50 Hz低通信号，采样周期均为100微秒。
% 在辨识支路内对五路信号使用相同的一电周期滑动平均，抑制周期纹波。
% 调试版：窗口填满即保存；稳态、工作点、角度及条件数仅提示，不阻断。
% y(5)、y(7)表示数值计算完成，不代表已经通过精度或物理合理性验收。
% 用采样积分的中心变化标定R和逆变器等效压降，不使用额定参数填充结果。
% y(17)保留首个诊断提示；18～27列保留；28～31列为损耗标定诊断。
persistent buf count oldState ready result aerr berr finished
persistent angleK angDone sumDiff m theta P iteration checked window cursor filled
persistent cycleBuf cycleSum cycleHead cycleCount cycleN rawWindow
persistent integrals previousInstant previousWe thetaRecord hasSample
persistent centerWindow centerBuf centerNormal centerRhs momentDiff lossCalibration
if isempty(count)
    buf=zeros(2000,5,6); count=0; oldState=-1; ready=false(6,1);
    result=zeros(31,1); result([3 4 6 28 29 30 31])=NaN;
    aerr=zeros(400,1); berr=zeros(400,1);
    finished=false; angleK=0; angDone=false; sumDiff=zeros(2,2);
    m=zeros(6,5); theta=zeros(2,1); P=10*eye(2);
    iteration=0; checked=false; window=zeros(2000,5); cursor=0; filled=0;
    cycleBuf=zeros(1257,5); cycleSum=zeros(1,5);
    cycleHead=0; cycleCount=0; cycleN=1; rawWindow=zeros(2000,2);
    integrals=zeros(1,6); previousInstant=zeros(1,6); previousWe=0;
    thetaRecord=0; hasSample=false; centerWindow=zeros(2000,8);
    centerBuf=zeros(2000,8,2); centerNormal=zeros(3,3,2); centerRhs=zeros(3,6,2);
    momentDiff=zeros(2,8); lossCalibration=zeros(2,1);
end
u=[ud uq id iq we];
if ~all(isfinite(u)) || ~isfinite(Vstate)
    ready(:)=false; result(:)=0; result([3 4 6 28 29 30 31])=NaN;
    result(8)=-1; result(17)=-1;
    if Vstate>=2 && Vstate<=8, result(16)=-Vstate; end
    finished=true; y=result; err1=aerr; err2=berr; return;
end
if Vstate~=oldState
    count=0; oldState=Vstate; cursor=0; filled=0; result(16)=0;
    cycleBuf(:)=0; cycleSum(:)=0; cycleHead=0; cycleCount=0; cycleN=1;
    if Vstate==0 || Vstate==2
        ready(:)=false; result(:)=0; result([3 4 6 28 29 30 31])=NaN;
        aerr(:)=0; berr(:)=0;
        finished=false; angleK=0; angDone=false; sumDiff(:)=0;
        theta(:)=0; P=10*eye(2); iteration=0; checked=false;
        integrals(:)=0; previousInstant(:)=0; previousWe=0; thetaRecord=0; hasSample=false;
        centerWindow(:)=0; centerBuf(:)=0; centerNormal(:)=0; centerRhs(:)=0;
        momentDiff(:)=0; lossCalibration(:)=0;
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
% 连续积分跨越所有实验阶段，包括Id切换恢复段。整体旋转初相位不影响标定。
% 原五路低通已经匹配；这里使用低通后的信号，避免重置周期平均器破坏积分连续性。
extra=zeros(1,8);
if Vstate>=2 && Vstate<=8
    if hasSample, thetaRecord=mod(thetaRecord+0.5e-4*(previousWe+we),2*pi); end
    c=cos(thetaRecord); s=sin(thetaRecord);
    stationary=[ud*c-uq*s,ud*s+uq*c,id*c-iq*s,id*s+iq*c];
    magnitude=max(hypot(id,iq),1e-12);
    instant=[stationary,(4/pi)*stationary(3:4)/magnitude];
    if hasSample, integrals=integrals+0.5e-4*(previousInstant+instant); end
    previousInstant=instant; previousWe=we; hasSample=true;
    extra=[integrals,c,s];
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
    % 等待0.4秒后保存最后0.2秒窗口；与原每段6000拍的时序匹配。
    % 调试版不再等待窗口通过验收；窗口填满后即存储，PLL按固定时长推进。
    if count>4000
        cursor=mod(cursor,2000)+1; window(cursor,:)=smooth;
        rawWindow(cursor,:)=u(1:2);
        centerWindow(cursor,:)=extra;
        filled=min(2000,filled+1);
    end
    if filled==2000 && mod(count,200)==0
        W=zeros(2000,5);
        for n=1:2000
            index=mod(cursor+n-1,2000)+1; W(n,:)=window(index,:);
            if slot==1 || slot==4
                centerBuf(n,:,1+(slot==4))=centerWindow(index,:);
            end
        end
        [meanW,reason,drift,diagV]=check_window(W);
        result(10)=drift;
        rawMean=sum(rawWindow,1)/2000;
        for j=1:2
            result(17+j)=sqrt(sum((rawWindow(:,j)-rawMean(j)).^2)/2000);
        end
        result(20:25)=diagV;
        if slot~=1 && slot~=4
            base=1+3*(slot>3);
            mismatch=norm(meanW(3:4)-m(base,3:4));
            result(11)=max(result(11),mismatch);
            if mismatch>max(0.02,0.005*norm(m(base,3:4))) || ...
                    abs(meanW(5)-m(base,5))>max(1,0.005*abs(m(base,5)))
                if reason==0, reason=-6; end
            end
        end
        % 正负窗口差异仅记录提示，按用户要求先输出实际数据的辨识曲线。
        if slot==3 || slot==6
            plus=slot-1;
            if norm(meanW(3:4)-m(plus,3:4))>max(0.003,0.0005*norm(m(plus,3:4))) || ...
                    abs(meanW(5)-m(plus,5))>max(0.02,0.0001*abs(m(plus,5)))
                if reason==0, reason=-12; end
            end
        end
        if result(17)==0 && reason~=0, result(17)=reason; end
        buf(:,:,slot)=W; m(slot,:)=meanW; ready(slot)=true;
        result(16)=Vstate;
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
                a=buf(n,:,pos); b=buf(n,:,neg);
                wa=safe_speed(a(5)); wb=safe_speed(b(5));
                da=(4/pi)*a(3:4)/max(hypot(a(3),a(4)),1e-12);
                db=(4/pi)*b(3:4)/max(hypot(b(3),b(4)),1e-12);
                momentDiff(g,:)=momentDiff(g,:)+[a(1:2)/wa-b(1:2)/wb, ...
                    a(3:4)/wa-b(3:4)/wb,a(3:4)-b(3:4),da/wa-db/wb];
                basis=[1;centerBuf(n,7,g);centerBuf(n,8,g)];
                centerNormal(:,:,g)=centerNormal(:,:,g)+basis*basis';
                centerRhs(:,:,g)=centerRhs(:,:,g)+basis*centerBuf(n,1:6,g);
            end
            d=atan2(sumDiff(g,2),sumDiff(g,1));
            if d>pi/2, d=d-pi; elseif d< -pi/2, d=d+pi; end
            result(g)=d;
            if g==1, aerr(angleK)=d; else, berr(angleK)=d; end
        end
        if angleK==400
            angDone=true;
            % 在两个基准平台拟合积分量的常值、余弦、正弦分量。
            % 磁链及相电流的稳态旋转分量零均值，中心差满足DeltaU=R*DeltaI+V*DeltaD。
            centers=zeros(2,6);
            for h=1:2
                coefficients=(centerNormal(:,:,h)+1e-10*eye(3))\centerRhs(:,:,h);
                centers(h,:)=coefficients(1,:);
            end
            difference=centers(2,:)-centers(1,:);
            matrix=[difference(3:4)',difference(5:6)']; target=difference(1:2)';
            result(31)=sqrt(sum(sum(matrix.^2)));
            normMatrix=max(result(31),1e-12);
            scaled=matrix/normMatrix; target=target/normMatrix;
            % 只用数值正则化处理退化数据，不设置参数范围或中止门限。
            lossCalibration=(scaled'*scaled+1e-10*eye(2))\(scaled'*target);
            result(28:29)=lossCalibration;
            detC=scaled(1,1)*scaled(2,2)-scaled(1,2)*scaled(2,1);tauC=sum(sum(scaled.^2));
            if abs(detC)>0
                result(30)=(tauC+sqrt(max(0,tauC*tauC-4*detC*detC)))/(2*abs(detC));
            else
                result(30)=Inf;
            end
            for h=1:2
                result(h)=corrected_angle(momentDiff(h,:),lossCalibration,0);
            end
            for g=1:2
                if norm(sumDiff(g,:))/2000<0.2 || abs(result(g))>0.6
                    if result(17)==0, result(17)=-7; end
                end
            end
            % 条件数仅用于诊断，不在此处预先求解参数，也不阻止递推。
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
                if result(17)==0, result(17)=-8; end
            end
        end
    else
        % 按A1、B1、A2、B2……A2000、B2000顺序读取两组基准数据。
        % 每次调用只使用一个观测量，执行一次实际递推，不反复重放同一批数据。
        % 正负注入数据用于估计位置误差，基准数据用于L和磁链辨识。
        iteration=iteration+1;
        % 相同的转速、电流才可直接相减；用数据标定的损耗项和当前L递推值补偿。
        for h=1:2
            result(h)=corrected_angle(momentDiff(h,:),lossCalibration,0.01*theta(1));
        end
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
        if mod(iteration,10)==0
            aerr(iteration/10)=result(1); berr(iteration/10)=result(2);
        end
        % 每次L和磁链递推后同步计算Rs，不因物理范围或相对门限隐藏结果。
        D=zeros(2,1); z=zeros(2,1);
        for h=1:2
            bb=1+3*(h-1); dd=m(bb,3); qq=m(bb,4); ww=m(bb,5);
            D(h)=(4/pi)*qq/max(hypot(dd,qq),1e-8);
            z(h)=m(bb,2)-ww*result(3)*dd-ww*result(4)*cos(result(h));
        end
        den=D(2)*m(1,4)-D(1)*m(4,4);
        rsScale=abs(D(2)*m(1,4))+abs(D(1)*m(4,4));
        result(14)=abs(den)/max(rsScale,1e-12);
        % 仅保留浮点除零保护；不可求时保留NaN，不用额定值代替结果。
        canDivide=abs(den)>64*eps(max(rsScale,1));
        if canDivide
            result(6)=(D(2)*z(1)-D(1)*z(2))/den;
        else
            result(6)=NaN;
        end
        if iteration==4000
            finished=true;
            result(5)=double(all(isfinite(theta)));
            result(7)=double(isfinite(result(6)));
            result(8)=30;
            if result(7)==0 && result(17)==0, result(17)=-11; end
        end
    end
end

y=result; err1=aerr; err2=berr;
end

function w=safe_speed(w)
% 仅防止归一化除零，不据此拒绝采样或限制估计值。
if abs(w)<1e-9
    if w<0,w=-1e-9;else,w=1e-9;end
end
end

function angle=corrected_angle(moment,loss,L)
% x=(ud-R*id-V*Dd)/we+L*iq；y=(uq-R*iq-V*Dq)/we-L*id。
% 正负注入的差向量方向给出位置误差；R、V均来自实际采样积分。
x=moment(1)-loss(1)*moment(3)+L*moment(6)-loss(2)*moment(7);
y=moment(2)-loss(1)*moment(4)-L*moment(5)-loss(2)*moment(8);
angle=atan2(y,x);
if angle>pi/2,angle=angle-pi;elseif angle< -pi/2,angle=angle+pi;end
end
function [meanW,reason,maxDrift,diagV] = check_window(W)
% 仅计算诊断提示，不拒绝窗口、不超时中止、不抑制参数输出。
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
