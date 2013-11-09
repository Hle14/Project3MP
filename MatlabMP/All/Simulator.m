classdef Simulator
% Simulator class
% Methods provide access to all the functionality that you will need from
% the simulator.
% Simulator is written as a class to make it even more similar with the C++
% interface
%
% You do not need to make changes to this file
    
    methods
        function s = SampleState(obj)
            s(1) = obj.m_xmin + (m_xmax - m_xmin) * rand();
            s(2) = obj.m_ymin + (m_ymax - m_ymin) * rand();
        end
        
        function status = IsStateValid(obj, s)
            status = s(1) >= m_xmin && s(1) <= m_xmax && ...
                          s(2)
        end
        
        function status = HasRobotReachedGoal(obj)
            status = obj.GetDistanceFromRobotCenterToGoal() <= obj.GetGoalRadius();
        end
       
        
       % Functions below this line are not needed for the implementation of
       % your planner. These functions are needed only by the graphical
       % interface.
       
        function [] = Draw(obj)
          clf; hold on; grid on;
          set(gca, 'xlim', [-22.5 22.5]); 
          set(gca, 'ylim', [-18.5 18.5]);
  
        
         fill(obj.m_circles(3) * obj.m_xptsStandardCircle + obj.m_circles(1), ...
              obj.m_circles(3) * obj.m_yptsStandardCircle + obj.m_circles(2), [0 1 0]);
              
         n = length(obj.m_circles);
         for i = 4 : 3 : n
           fill(obj.m_circles(i + 2) * obj.m_xptsStandardCircle + obj.m_circles(i), ...
                obj.m_circles(i + 2) * obj.m_yptsStandardCircle + obj.m_circles(i + 1), [0 0 1]);
         end

          n = length(obj.m_robotCurrVertices);
          fill(obj.m_robotCurrVertices(1 : 2 : n), obj.m_robotCurrVertices(2 : 2 : n), [1 0 0]);
    
          drawnow;
        end
               
        
        function poly = ReadPolygon(obj, in)       
            nv   = fscanf(in, '%d', [1 1]);
            poly = zeros(1, 2 * nv);
            for j = 1 : 1 : 2 * nv
                poly(j) = fscanf(in, '%f', [1 1]);
            end    
        end
    
        function obj = ReadRobot(obj, fname)
            in = fopen(fname, 'r');
            obj.m_robotInitVertices = obj.ReadPolygon(in);
            obj.m_robotCurrVertices = obj.m_robotInitVertices;
            obj.m_robotCurrConfig   = [0 0 0];   
            fclose(in);
        end
        
        function obj = RigidBodySimulator(fnameRobot)        
            obj = obj.ReadRobot(fnameRobot);
            obj.m_circles            = [5 5 1];            
            dthetas                  = 0 : 2 * pi / 50 : 2 * pi;
            obj.m_xptsStandardCircle = cos(dthetas);
            obj.m_yptsStandardCircle = sin(dthetas);
        end
    end
    
    
    properties(GetAccess=public, SetAccess=public)
        m_circleObstacles;
        m_circleGoal;
        m_circleRobot;
        m_distOneStep;
        m_xmin;
        m_xmax;
        m_ymin;
        m_ymax;
        m_xptsStandardCircle;
        m_yptsStandardCircle;
    end

end
