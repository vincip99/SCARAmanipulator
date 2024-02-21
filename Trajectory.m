%% Trajectory planning
clear all

%% Trajectory points and Orientation
% Create points and orientation matrix
% path = [.85, .35, .75, 0;
%     .85, .35, .5, 0;
%     .85, .35, .75, 0;
%     .85, .35, .9, 0; % via point
%     .85, .35, 1, 0; % circumference start
%     1.1, .5, 1, 0; % circumference end
%     1.1, .5, .9, 0; % via point
%     1.1, .5, .8, 0; % via point
%     1.1, .6, .75, 0; % via point
%     1.1, .6, .6, 0; % circumference start
%     .8, .6, .6, 0;]; % circumference end

path = [0.3536 -0.8536 0.5 0;
0.75 -0.433 0.7 0;
0.683 0.183 0.8 0;
0.25 0.433 0.8 0;
-0.1464 0.3536 0.6 0;
-0.183 0.183 0.3 0;
-0.5 0 0.2 0;
-0.5 -0.5 0.2 0;
-0.2878 -0.78 0.3 0;
0 -0.866 0.4 0;
0.3536 -0.8536 0.5 0];

% Time vector from ti to tf
% T = [0 2 4 6 8 10 12 14 16 18 20];
T = [0 5 9 12 15 18 21 24 28 32 36];
% Time step precision
precision = 0.1;

t1 = T(1):0.1:T(2); t2 = T(2):0.1:T(3); t3 = T(3):0.1:T(4);
t4 = T(4):0.1:T(5); t5 = T(5):0.1:T(6); t6 = T(6):0.1:T(7);
t7 = T(7):0.1:T(8); t8 = T(8):0.1:T(9); t9 = T(9):0.1:T(10);
t10 = T(10):0.1:T(11);

%% plot the trajectory points
plotPathPlan(path)

%% Arc length calculation

% populate the arc length for each segment
[s1, s1_dot] = trapezioidal(t1,path(1,1:3),path(2,1:3));
[s2, s2_dot] = trapezioidal(t2,path(2,1:3),path(3,1:3));
[s3, s3_dot] = trapezioidal(t3,path(3,1:3),path(4,1:3));
[s4, s4_dot] = trapezioidal(t4,path(4,1:3),path(5,1:3));
[s5, s5_dot] = trapezioidal(t5,path(5,1:3),path(6,1:3));
[s6, s6_dot] = trapezioidal(t6,path(6,1:3),path(7,1:3));
sc7 = trapezioidalTrj(t7,0,pi);
[s8, s8_dot] = trapezioidal(t8,path(8,1:3),path(9,1:3));
[s9, s9_dot] = trapezioidal(t9,path(9,1:3),path(10,1:3));
[s10, s10_dot] = trapezioidal(t10,path(10,1:3),path(11,1:3));
% [s9, s9_dot, vfin9] = trapezioidalVia(t9,path(9,1:3),path(10,1:3),s8_dot(end),0.1);
% [s10, s10_dot] = trapezioidalVia(t10,path(10,1:3),path(11,1:3),vfin9,0.1);

%% 3D path
seg1 = rectilinearPath(path(1,1:3)',path(2,1:3)',s1(1,:),s1(2,:),s1(3,:));
seg2 = rectilinearPath(path(2,1:3)',path(3,1:3)',s2(1,:),s2(2,:),s2(3,:));
seg3 = rectilinearPath(path(3,1:3)',path(4,1:3)',s3(1,:),s3(2,:),s3(3,:));
seg4 = rectilinearPath(path(4,1:3)',path(5,1:3)',s4(1,:),s4(2,:),s4(3,:));
seg5 = rectilinearPath(path(5,1:3)',path(6,1:3)',s5(1,:),s5(2,:),s5(3,:));
seg6 = rectilinearPath(path(6,1:3)',path(7,1:3)',s6(1,:),s6(2,:),s6(3,:));
seg7 = circularPath(path(7,1:3)',path(8,1:3)',sc7);
seg8 = rectilinearPath(path(8,1:3)',path(9,1:3)',s8(1,:),s8(2,:),s8(3,:));
seg9 = rectilinearPath(path(9,1:3)',path(10,1:3)',s9(1,:),s9(2,:),s9(3,:));
seg10 = rectilinearPath(path(10,1:3)',path(11,1:3)',s10(1,:),s10(2,:),s10(3,:));

plotTrj(seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8, seg9, seg10);
 
traj_pos = [seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8, seg9, seg10];
traj_orient = zeros(1,length(traj_pos));
traj = [traj_pos; traj_orient];

traj_dot = zeros(4,length(traj));
traj_dot(1,:) = diff([0 traj(1,:)]); 
traj_dot(2,:) = diff([0 traj(2,:)]); 
traj_dot(3,:) = diff([0 traj(3,:)]);
traj_dot(4,:) = diff([0 traj(4,:)]);

traj_ddot = zeros(4,length(traj));
traj_ddot(1,:) = diff([0 traj_dot(1,:)]); 
traj_ddot(2,:) = diff([0 traj_dot(2,:)]); 
traj_ddot(3,:) = diff([0 traj_dot(3,:)]);
traj_ddot(4,:) = diff([0 traj_dot(4,:)]);

time = [t1, t2, t3, t4, t5, t6, t7, t8, t9, t10];
timesample = timeseries(traj',time);
timesample_dot = timeseries(traj_dot',time);
timesample_ddot = timeseries(traj_ddot',time);

%% Function to plot the trajectory points planned
function plotPathPlan(path)

    % Plot the trajectory points
    figure
    plot3(path(:,1), path(:,2), path(:,3), 'b', 'LineWidth', 2);
    grid on
    hold on
    scatter3(path(1,1), path(1,2), path(1,3), 100, ...
        'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r')
    scatter3(path(2:end-1,1), path(2:end-1,2), path(2:end-1,3), 100, ...
        'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y')
    scatter3(path(end,1), path(end,2), path(end,3), 100, ...
        'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g')
    xlabel('x(t)'); ylabel('y(t)'); zlabel('z(t)')
    title('Trajectory points');
    legend('Trajectory', 'Start Point', 'Intermediate Points', 'End Point');
    hold off
end

%% Trapezioidal velocity profile function motion law 
function [s_i, si_dot] = trapezioidalTrj(t,pin,pfin)
    
    % init s (motion law)
    s_i = zeros(1,length(t));
    
    % Define cruise velocity, cruise time, cruise acceleration
    pc_d = 1.8*(pfin - pin)/(t(end) - t(1));
    tc = (pin - pfin + pc_d*(t(end) - t(1)))/pc_d;
    pc_dd = pc_d^2/(pin - pfin + pc_d*(t(end) - t(1)));
    
    % Composing motion law 
    for i = 1:length(s_i)
        if t(i) >= t(1) && t(i) <= tc + t(1)
            s_i(i) = pin + 0.5 * pc_dd * (t(i) - t(1))^2;
        elseif t(i) > tc + t(1) && t(i) <= t(end) - tc
            s_i(i) = pin + pc_dd * tc *((t(i) - t(1)) - 0.5*tc);
        elseif t(i) > t(end) - tc && t(i) <= t(end)
            s_i(i) = pfin - 0.5 * pc_dd * (t(end) - t(i))^2;
        end
    end

    %% Velocity trapezioidal profile
    si_dot = zeros(1,length(s_i));
    
    sc_dot = pc_d;
    s0_dot = 0;

    for i = 1:length(si_dot)
        if t(i) >= t(1) && t(i) <= tc + t(1)
            si_dot(i) = (sc_dot - s0_dot)/tc * (t(i) - t(1));
        elseif t(i) > tc + t(1) && t(i) <= t(end) - tc
            si_dot(i) = sc_dot;
        elseif t(i) > t(end) - tc && t(i) <= t(end)
            si_dot(i) = (s0_dot - sc_dot)/(tc) * (t(i) - t(end));
        end
    end

end

%% 3D trapezioidal motion law
function [s, s_dot] = trapezioidal(t,pin,pfin)
    
    % x, y, z arc length and velocity
    [sx, sx_dot] = trapezioidalTrj(t,pin(1,1),pfin(1,1));
    [sy, sy_dot] = trapezioidalTrj(t,pin(1,2),pfin(1,2));
    [sz, sz_dot] = trapezioidalTrj(t,pin(1,3),pfin(1,3));

    s = [sx; sy; sz];
    s_dot = [sx_dot; sy_dot; sz_dot];

end

function [s, s_dot, vfin] = trapezioidalVia(t,pin,pfin,vin,dt)
    
    % x, y, z arc length and velocity
    [sx, sx_dot, vfinx] = viaPointTrj(t,pin(1,1),pfin(1,1),vin(1),dt);
    [sy, sy_dot, vfiny] = viaPointTrj(t,pin(1,2),pfin(1,2),vin(2),dt);
    [sz, sz_dot, vfinz] = viaPointTrj(t,pin(1,3),pfin(1,3),vin(3),dt);

    s = [sx; sy; sz];
    s_dot = [sx_dot; sy_dot; sz_dot];
    vfin = [vfinx; vfiny; vfinz]; 

end

%% Trapezioidal velocity profile via points function motion law 
function [s_i, si_dot, vfin] = viaPointTrj(t,pin,pfin,vin,dt)
    
    % init s (motion law)
    s_i = zeros(1,length(t));
    
    % Define cruise velocity, cruise time, cruise acceleration
    pc_d = 1.8*(pfin - pin)/(t(end) - t(1));
    tc = (pin - pfin + pc_d*(t(end) - t(1)))/pc_d;
    pc_dd = pc_d^2/(pin - pfin + pc_d*(t(end) - t(1)));
    
    % Composing motion law 
    for i = 1:length(s_i)
        if t(i) >= t(1) && t(i) <= tc + t(1)
            s_i(i) = pin + 0.5 * pc_dd * (t(i) - t(1))^2;
        elseif t(i) > tc + t(1) && t(i) <= t(end) - tc
            s_i(i) = pin + pc_dd * tc *((t(i) - t(1)) - 0.5*tc);
        elseif t(i) > t(end) - tc && t(i) <= t(end) - dt
            s_i(i) = pfin - 0.5 * pc_dd * (t(end) - t(i))^2;
        end
    end

    %% Velocity trapezioidal profile
    si_dot = zeros(1,length(s_i));
    
    sc_dot = pc_d;
    s0_dot = vin;

    for i = 1:length(si_dot)
        if t(i) >= t(1) && t(i) <= tc + t(1)
            si_dot(i) = (sc_dot - s0_dot)/tc * (t(i) - t(1));
        elseif t(i) > tc + t(1) && t(i) <= t(end) - tc
            si_dot(i) = sc_dot;
        elseif t(i) > t(end) - tc && t(i) <= t(end) - dt
            si_dot(i) = (s0_dot - sc_dot)/(tc) * (t(i) - t(end));
        end
    end


    s_dot = [si_dot(1:i), nonzeros(si_dot(i+1:end))];
    vfin = si_dot(i);

end

%% Cubic profile function motion law 
function s = cubicTrj(t,pin,pfin,vin,vfin)
    
    A = [1 0 0 0;
        0 1 0 0;
        1 t(end) t(end)^2 t(end)^3;
        0 1 2*t(end) 3*t(end)^2;];
    B = [pin; vin; pfin; vfin];

    a = inv(A) * B;
    
    s = a(4)*t.^3 + a(3)*t.^2 + a(2)*t + a(1);

end

%% Rectilinear path primitive function
function rect = rectilinearPath(pin,pfin,sx,sy,sz)
    % s is a 3 x length(t) vector of curvilinear abscissa for each
    % coordinates
    s = [sx; sy; sz];
    % init rectangular path, each row for each coordinates
    rect = zeros(3,length(s));
    
    % for loop in order to populate rectilinear segment for each x,y,z 
    for i = 1:3
        % if there is no motion along the generic axis the norm is 0 and
        % the path should have a constant value
        if (norm(pfin(i) - pin(i)) == 0)
            rect(i,:) = pin(i) .* ones(size(s(i,:)));
        else
           rect(i,:) = pin(i) + (abs(s(i,:) - pin(i))) .* (pfin(i) - pin(i))/(norm(pfin(i) - pin(i),2));  
        end
    end

end

%% Circular path primitive function
function circ = circularPath(pin,pfin,s)
    % center, radius and rotation matrix for circular path
    c = (pin + pfin) / 2;
    rho = norm((pfin - pin), 2) / 2;
    theta = atan2(pin(2) - pfin(2), pin(1) - pfin(1));
    R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    
    s = s * rho;

    % Parametrization
    circ = c + R * [rho*cos((s/rho)); rho*sin((s/rho)); zeros(size(s))];
end

%% Function to plot the trajectory
function plotTrj(seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8, seg9, seg10)
    % Plot segments in 3D space
    plot3(seg1(1,:), seg1(2,:), seg1(3,:), 'LineWidth', 2);
    hold on;
    plot3(seg2(1,:), seg2(2,:), seg2(3,:), 'LineWidth', 2);
    plot3(seg3(1,:), seg3(2,:), seg3(3,:), 'LineWidth', 2);
    plot3(seg4(1,:), seg4(2,:), seg4(3,:), 'LineWidth', 2);
    plot3(seg5(1,:), seg5(2,:), seg5(3,:), 'LineWidth', 2);
    plot3(seg6(1,:), seg6(2,:), seg6(3,:), 'LineWidth', 2);
    plot3(seg7(1,:), seg7(2,:), seg7(3,:), 'LineWidth', 2);
    plot3(seg8(1,:), seg8(2,:), seg8(3,:), 'LineWidth', 2);
    plot3(seg9(1,:), seg9(2,:), seg9(3,:), 'LineWidth', 2);
    plot3(seg10(1,:), seg10(2,:), seg10(3,:), 'LineWidth', 2);
    
    % Plot initial points with star marker
    initialPoints = [seg1(:,1), seg2(:,1), seg3(:,1), seg4(:,1), seg5(:,1),...
                     seg6(:,1), seg7(:,1), seg8(:,1), seg9(:,1), seg10(:,1)];
    plot3(initialPoints(1,:), initialPoints(2,:), initialPoints(3,:),...
          'k*', 'MarkerSize', 10);

    % Beautify plot
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Trajectory Plot');
    
    hold off;
end