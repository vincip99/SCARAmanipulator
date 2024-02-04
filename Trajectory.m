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
-0.2878 -0.78 0.3 0
0 -0.866 0.4 0
0.3536 -0.8536 0.5 0];

% Time vector from ti to tf
% T = [0 1 2 4 4.5 5 6 7.5 7.9 9.1 10]; % Time to be for each point
T = [0 2 4 6 8 10 12 14 16 18 20];

t1 = T(1):0.1:T(2); t2 = T(2):0.1:T(3); t3 = T(3):0.1:T(4);
t4 = T(4):0.1:T(5); t5 = T(5):0.1:T(6); t6 = T(6):0.1:T(7);
t7 = T(7):0.1:T(8); t8 = T(8):0.1:T(9); t9 = T(9):0.1:T(10);
t10 = T(10):0.1:T(11);

%% plot the trajectory points
figure
plot3(path(:,1),path(:,2),path(:,3),'b','LineWidth',2);
grid on
hold on
scatter3(path(1,1),path(1,2),path(1,3),100,...
    'filled','MarkerEdgeColor','k','MarkerFaceColor','r')
scatter3(path(2:end-1,1),path(2:end-1,2),path(2:end-1,3),100,...
    'filled','MarkerEdgeColor','k','MarkerFaceColor','y')
scatter3(path(end,1),path(end,2),path(end,3),100,...
    'filled','MarkerEdgeColor','k','MarkerFaceColor','g')
xlabel('x(t)'); ylabel('y(t)'); zlabel('z(t)')
title('Trajectory points');
hold off

%% Arc length calculation

% populate the arc length for each segment
sx1 = trapezioidalTrj(t1,path(1,1),path(2,1));
sy1 = trapezioidalTrj(t1,path(1,2),path(2,2));
sz1 = trapezioidalTrj(t1,path(1,3),path(2,3));

sx2 = trapezioidalTrj(t2,path(2,1),path(3,1));
sy2 = trapezioidalTrj(t2,path(2,2),path(3,2));
sz2 = trapezioidalTrj(t2,path(2,3),path(3,3));

sx3 = trapezioidalTrj(t3,path(3,1),path(4,1));
sy3 = trapezioidalTrj(t3,path(3,2),path(4,2));
sz3 = trapezioidalTrj(t3,path(3,3),path(4,3));

sx4 = trapezioidalTrj(t4,path(4,1),path(5,1));
sy4 = trapezioidalTrj(t4,path(4,2),path(5,2));
sz4 = trapezioidalTrj(t4,path(4,3),path(5,3));

% dt = 1;
% 
% sx4 = shift(dt,t4,path(4,1),path(5,1));
% sy4 = shift(dt,t4,path(4,2),path(5,2));
% sz4 = shift(dt,t4,path(4,3),path(5,3));

%sc5 = trapezioidalTrj(t5,0,pi);

sx5 = trapezioidalTrj(t5,path(5,1),path(6,1));
sy5 = trapezioidalTrj(t5,path(5,2),path(6,2));
sz5 = trapezioidalTrj(t5,path(5,3),path(6,3));

sx6 = trapezioidalTrj(t6,path(6,1),path(7,1));
sy6 = trapezioidalTrj(t6,path(6,2),path(7,2));
sz6 = trapezioidalTrj(t6,path(6,3),path(7,3));

% sx7 = trapezioidalTrj(t7,path(7,1),path(8,1));
% sy7 = trapezioidalTrj(t7,path(7,2),path(8,2));
% sz7 = trapezioidalTrj(t7,path(7,3),path(8,3));

sc7 = trapezioidalTrj(t7,0,pi);

sx8 = trapezioidalTrj(t8,path(8,1),path(9,1));
sy8 = trapezioidalTrj(t8,path(8,2),path(9,2));
sz8 = trapezioidalTrj(t8,path(8,3),path(9,3));

sx9 = trapezioidalTrj(t9,path(9,1),path(10,1));
sy9 = trapezioidalTrj(t9,path(9,2),path(10,2));
sz9 = trapezioidalTrj(t9,path(9,3),path(10,3));

% sc10 = trapezioidalTrj(t10,0,pi);
sx10 = trapezioidalTrj(t9,path(10,1),path(11,1));
sy10 = trapezioidalTrj(t9,path(10,2),path(11,2));
sz10 = trapezioidalTrj(t9,path(10,3),path(11,3));

% sx = trapezioidalTrj(t,path(1,1),path(2,1));
% sy = trapezioidalTrj(t,path(1,2),path(2,2));
% sz = trapezioidalTrj(t,path(1,3),path(2,3));

% sx = cubicTrj(t,path(1,1),path(2,1),0,0);
% sy = cubicTrj(t,path(1,2),path(2,2),0,0);
% sz = cubicTrj(t,path(1,3),path(2,3),0,0);

% sx1 = trapezioidalTrj(t,path(2,1),path(3,1));
% sy1 = trapezioidalTrj(t,path(2,2),path(3,2));
% sz1 = trapezioidalTrj(t,path(2,3),path(3,3));
% 
% sc = trapezioidalTrj(t,0,pi);

%% 3D path
% rect = pathRect(path(1,1:3)',path(2,1:3)',sx,sy,sz);
% rect1 = pathRect(path(2,1:3)',path(3,1:3)',sx1,sy1,sz1);
% circular = circularPath(path(1,1:3)',path(2,1:3)',sc);

rect1 = pathRect(path(1,1:3)',path(2,1:3)',sx1,sy1,sz1);
rect2 = pathRect(path(2,1:3)',path(3,1:3)',sx2,sy2,sz2);
rect3 = pathRect(path(3,1:3)',path(4,1:3)',sx3,sy3,sz3);
rect4 = pathRect(path(4,1:3)',path(5,1:3)',sx4,sy4,sz4);
% circ5 = circularPath(path(5,1:3)',path(6,1:3)',sc5);
circ5 = pathRect(path(5,1:3)',path(6,1:3)',sx5,sy5,sz5);
rect6 = pathRect(path(6,1:3)',path(7,1:3)',sx6,sy6,sz6);
% rect7 = pathRect(path(7,1:3)',path(8,1:3)',sx7,sy7,sz7);
rect7 = circularPath(path(7,1:3)',path(8,1:3)',sc7);
rect8 = pathRect(path(8,1:3)',path(9,1:3)',sx8,sy8,sz8);
rect9 = pathRect(path(9,1:3)',path(10,1:3)',sx9,sy9,sz9);
% circ10 = circularPath(path(10,1:3)',path(11,1:3)',sc10);
circ10 = pathRect(path(10,1:3)',path(11,1:3)',sx10,sy10,sz10);

figure
plot3(rect1(1,:),rect1(2,:),rect1(3,:),'LineWidth', 2)
hold on 
plot3(rect2(1,:),rect2(2,:),rect2(3,:),'LineWidth', 2)
hold on
plot3(rect3(1,:),rect3(2,:),rect3(3,:),'LineWidth', 2)
hold on 
plot3(rect4(1,:),rect4(2,:),rect4(3,:),'LineWidth', 2)
hold on
plot3(circ5(1,:),circ5(2,:),circ5(3,:),'LineWidth', 2)
hold on
plot3(rect6(1,:),rect6(2,:),rect6(3,:),'LineWidth', 2)
hold on 
plot3(rect7(1,:),rect7(2,:),rect7(3,:),'LineWidth', 2)
hold on
plot3(rect8(1,:),rect8(2,:),rect8(3,:),'LineWidth', 2)
hold on 
plot3(rect9(1,:),rect9(2,:),rect9(3,:),'LineWidth', 2)
hold on
plot3(circ10(1,:),circ10(2,:),circ10(3,:),'LineWidth', 2)
grid on
hold off

traj = [rect1, rect2, rect3, rect4, circ5, rect6, rect7, rect8,...
     rect9, circ10];
time = [t1, t2, t3, t4, t5, t6, t7, t8, t9, t10];
timesample = timeseries(traj',time);

% figure
% plot3(rect(1,:),rect(2,:),rect(3,:),'LineWidth', 2)
% hold on 
% %plot3(rect1(1,:),rect1(2,:),rect1(3,:))
% hold on
% plot3(circular(1,:),circular(2,:),circular(3,:))
% hold on
% scatter3(rect(1,1),rect(2,1),rect(3,1), 100, ...
%     'filled', 'MarkerEdgeColor','k', 'MarkerFaceColor', 'r');
% hold on
% scatter3(rect(1,end),rect(2,end),rect(3,end), 100, ...
%     'filled', 'MarkerEdgeColor','k', 'MarkerFaceColor', 'r');
% xlabel('x(t)'); ylabel('y(t)'); zlabel('z(t)')
% title('Trajectory');
% ax = gca;
% ax.XAxisLocation = 'origin';
% ax.YAxisLocation = 'origin';
% hold off
% grid on

function s = trapezioidalTrj(t,pin,pfin)
    
    % init s (motion law)
    s = zeros(1,length(t));

    pc_d = 2*(pfin - pin)/(t(end) - t(1));
    tc = (pin - pfin + pc_d*(t(end) - t(1)))/pc_d;
    pc_dd = pc_d^2/(pin - pfin + pc_d*(t(end) - t(1)));

    for i = 1:length(s)
        if t(i) >= t(1) && t(i) <= tc + t(1)
            s(i) = pin + 0.5 * pc_dd * (t(i) - t(1))^2;
        elseif t(i) > tc + t(1) && t(i) <= (t(end) - tc)
            s(i) = pin + pc_dd * tc *((t(i) - t(1)) - 0.5*tc);
        elseif t(i) > t(end) - tc && t(i) <= t(end)
            s(i) = pfin - 0.5 * pc_dd * (t(end) - t(i))^2;
        end
    end
end

function s = cubicTrj(t,pin,pfin,vin,vfin)
    
    % init s (motion law)
    s = zeros(1,length(t));
    
    A = [1 0 0 0;
        0 1 0 0;
        1 t(end) t(end)^2 t(end)^3;
        0 1 2*t(end) 3*t(end)^2;];
    B = [pin; vin; pfin; vfin];

    a = inv(A) * B;
    
    s = a(4)*t.^3 + a(3)*t.^2 + a(2)*t + a(1);
end

function rect = pathRect(pin,pfin,sx,sy,sz)
    % s is a 3 x length(t) vector of curvilinear abscissa for each
    % coordinates
    s = [sx; sy; sz];
    % init rectangular path, each row for each coordinates
    rect = zeros(3,length(s));
    
    % for loop in order to populate rect for each x,y,z 
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

%function ts = t_shift(dt)

function s_shift = shift(dt,t,pin,pfin)

    ts = t - dt * ones(1,length(t)); 
    
    s_shift = trapezioidalTrj(ts,pin,pfin);
    
end