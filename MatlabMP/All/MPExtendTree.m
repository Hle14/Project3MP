function [] = MPExtendTree(vid, sto)
global mp;
global params;

dstep     = params.distOneStep;
x           = sto(1) - mp.xpts(vid);
y           = sto(2) - mp.ypts(vid);
d           = norm([x, y]);
ux         = dstep * x / d;
uy         = dstep * y / d;
nrSteps = ceil(d / dstep);

for k = 1 : 1 : nrSteps
    params.robot(1) = mp.xpts(vid) + ux;
    params.robot(2) = mp.ypts(vid) + uy;
    if IsValidState() == 0
        return;
    end
    n                              = length(mp.xpts);
    mp.nchildren(vid)      = mp.nchildren(vid) + 1;
    mp.xpts(n + 1)        = params.robot(1);
    mp.ypts(n + 1)        = params.robot(2);
    mp.parents(n + 1)   = vid;
    mp.nchildren(n + 1) = 0;
    
    if HasRobotReachedGoal() == 1
        mp.vidAtGoal = n + 1;
        return;
    end
    vid = n + 1;
end

end

