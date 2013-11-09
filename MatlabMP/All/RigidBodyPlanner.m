function [dx, dy, dtheta] = RigidBodyPlanner()
% This is the function that you should implement.
% This function needs to compute by how much the position (dx, dy) 
% and orientation (dtheta) should change so that the robot makes a small 
% move toward the goal while avoiding obstacles, 
% as guided by the potential field.
%
% You have access to the simulator.
% You can use the methods available in simulator to get all the information
% you need to correctly implement this function
%

    global rigidBodySimulator;

    [x, y, theta] = rigidBodySimulator.GetRobotCurrentConfig();
    [xgoal, ygoal]= rigidBodySimulator.GetGoalCenter();
    currVertices  = rigidBodySimulator.GetRobotCurrentVertices();
    n2            = length(currVertices);
    nrObstacles   = rigidBodySimulator.GetNrObstacles();
    confSpaceGrad = [0 0 0];
    
    for j = 1 : 2 : n2
          xj = currVertices(j);
          yj = currVertices(j + 1);
          jac= RigidBodyJacobian(x, y, theta, xj, yj); 
          for i = 1 : nrObstacles
            [xmin, ymin] = rigidBodySimulator.ClosestPointOnObstacle(i, xj, yj);            
            wgx = xmin - xj;
            wgy = ymin - yj;
            mag = norm([wgx, wgy]);              
            if mag < 0.8
              confSpaceGrad = confSpaceGrad + RigidBodyApplyJacobian(jac, wgx / (mag^2), wgy / (mag^2));
            end
          end
          wgx          = xj - xgoal;
          wgy          = yj - ygoal;
          mag          = norm([wgx, wgy]);
          confSpaceGrad= confSpaceGrad + RigidBodyApplyJacobian(jac, wgx / mag, wgy / mag);
    end
    
    dx = confSpaceGrad(1);
    dy = confSpaceGrad(2);
    dtheta = confSpaceGrad(3);
    if dtheta > 0
        dtheta = -0.01;
    else
        dtheta = 0.01;
    end
    mag = norm([dx, dy]);
    dx = -0.1 * dx / mag;
    dy = -0.1 * dy / mag;
    
end

function jac = RigidBodyJacobian(x, y, theta, xj, yj)
    c = cos(theta);
    s = sin(theta);
    
    jac(1) = 1;  jac(2) = 0;  jac(3) = -xj * s - yj * c;
    jac(4) = 0;  jac(5) = 1;  jac(6) =  xj * c - yj * s;
end

function confSpaceGrad = RigidBodyApplyJacobian(jac, workSpaceGradX, workSpaceGradY)
    confSpaceGrad(1) = jac(1) * workSpaceGradX + jac(4) * workSpaceGradY;
    confSpaceGrad(2) = jac(2) * workSpaceGradX + jac(5) * workSpaceGradY;
    confSpaceGrad(3) = jac(3) * workSpaceGradX + jac(6) * workSpaceGradY;
end
