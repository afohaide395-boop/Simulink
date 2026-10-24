function test_pair_matching
% 调试版：正负窗口电流或转速不匹配仅作提示，仍采齐并执行递推。
folder=fileparts(mfilename('fullpath'));addpath(folder);
for mode=1:2
    clear identify_vbemf
    record_identification(0,0,0,0,220,0);
    for state=[2 3 4 6 7 8]
        qi=4;w=220;d=.08;
        di=0;if state>=6,di=-1;d=-.12;end
        if state==3,d=d+.01;elseif state==4,d=d-.01;end
        if state==7,d=d+.01;elseif state==8,d=d-.01;end
        if mode==1 && state==4,w=w+.2;end
        if mode==2 && state==3,qi=qi+.004;end
        if mode==2 && state==4,qi=qi-.004;end
        ud=1.8665*di-w*.00896*qi-w*.21*sin(d);
        uq=1.8665*qi+w*.00896*di+w*.21*cos(d);
        for n=1:6000
            y=record_identification(ud,uq,di,qi,w,state);
            if y(16)==state || y(16)<0,break;end
        end
        assert(y(16)==state && y(8)==1,'工作点提示不应拒绝采样。');
    end
    for n=1:4401,y=record_identification(0,0,0,0,220,9);end
    assert(y(8)==30 && y(17)==-12 && y(12)==4000 && all(isfinite(y([3 4 6]))));
end
fprintf('PASS 正负窗口不匹配仅记录-12，仍完成4000次递推并保留数值\n');
end
