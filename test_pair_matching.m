function test_pair_matching
% Smooth +/- windows alone do not establish the subtraction assumptions.
folder=fileparts(mfilename('fullpath'));addpath(folder);
for mode=1:2
    clear identify_vbemf
    identify_vbemf(0,0,0,0,220,0);
    for state=2:4
        qi=4;w=220;d=.08;
        if state==3,d=d+.01;elseif state==4,d=d-.01;end
        if mode==1 && state==4,w=w+.2;end
        if mode==2 && state==3,qi=qi+.004;end
        if mode==2 && state==4,qi=qi-.004;end
        ud=-w*.00896*qi-w*.21*sin(d);
        uq=1.8665*qi+w*.21*cos(d);
        for n=1:30000
            y=identify_vbemf(ud,uq,0,qi,w,state);
            if y(16)==state || y(16)<0,break;end
        end
        if state<4,assert(y(16)==state);end
    end
    assert(y(8)==-12 && y(17)==-12 && y(16)==-4 && isnan(y(3)));
end
fprintf('PASS +/- windows individually steady but speed/current unmatched: no false angle/RLS; explicit -12 timeout\n');
end
