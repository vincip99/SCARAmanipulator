%% Direct Kinematics
%% Parameter Initialization
% Init the SCARA manipulator parameters
d0 = 1; a1 = .5; a2 = .5; l1 = .25; l2 = .25; 
ml1 = 20; ml2 = 20; ml3 = 10;
Il1 = 4; Il2 = 4; Il4 = 1;
kr1 = 1; kr2 = 1; kr3 = 50; kr4 = 20;
Im1 = .01; Im2 = .01; Im3 = .005; Im4 = .001;
Fm1 = .00005; Fm2 = .00005; Fm3 = .01; Fm4 = .005;
% numLinks
numLinks = 5;

% Init joint variables
theta1 = pi/6;
theta2 = pi/4;
d3 = .25;
theta4 = pi/3; 

% Init DH matrix
%       | a alpha d theta | length twist offset angle
% link i|                 |
DH = [0, 0, d0, 0;
    a1, 0, 0, theta1;
    a2, pi, 0, theta2;
    0, 0, d3, 0;
    0, 0, 0, theta4;];

% Init T matrix
T = eye(4);
% Init Position vector
p = zeros(3,numLinks);

%% Homogeneus Transformation matrix
figure;
for i = 1:numLinks
    
    % Link Position
    p(:,i) = T(1:3,4);
    
    % Computing the i-th homogeneus matrix
    A = [cos(DH(i,4)), -sin(DH(i,4))*cos(DH(i,2)), ... 
        sin(DH(i,4))*sin(DH(i,2)), DH(i,1)*cos(DH(i,4));
        sin(DH(i,4)), cos(DH(i,4))*cos(DH(i,2)), ...
        -cos(DH(i,4))*sin(DH(i,2)), DH(i,1)*sin(DH(i,4));
        0, sin(DH(i,2)), cos(DH(i,2)), DH(i,3);
        0, 0, 0, 1;];

    % Computing Tb_n
    T = T * A;
    
end

% End effector Position and Orientation
Pe = T(1:3,4);
Re = T(1:3,1:3);

yaw = atan2(Re(2,1),Re(1,1));
pitch = atan2(-Re(3,1),sqrt(Re(3,2)^2 + Re(3,3)^2));
roll = atan2(Re(3,2),Re(3,3)); 

fprintf('\nTransformation matrix from base to end effector\n');
disp(T);

% Direct kinematics function
xe = a1*cos(theta1) + a2*cos(theta1 + theta2);
ye = a1*sin(theta1) + a2*sin(theta1 + theta2);
ze = d0 - d3;
phi = theta1 + theta2 - theta4;

% Plot SCARA manipulator
plotScara(p,xe,ye,ze,phi);

%% Ellipsoid Calculation for different joint variables value

precision = 0.1; % precision of animation 

figure(2)
for i = theta1:precision:theta1 + 2*pi

    %% Geometric Jacobian Matrix
    J = [-a1*sin(theta1) - a2*sin(theta1 + theta2), - a2*sin(theta1 + theta2), 0, 0;
        a1*cos(theta1) + a2*cos(theta1 + theta2), a2*cos(theta1 + theta2), 0, 0;
        0, 0, -1, 0;
        1, 1, 0, -1;];

    J = J(1:2,1:2);

    %% Eigenvalues and Eigenvectors
    [V, D] = eig(J*J');
    ev = diag(D);
    % t1 = atan2(V(2,1), V(1,1));
    xe = sqrt((abs(ev(1))));
    ye = sqrt((abs(ev(2))));
    
    %% Forward Kinematics of 2 link approximation for the SCARA
    x = a1 * cos(theta1) + a2 * cos(theta1 + theta2);
    y = a1 * sin(theta1) + a2 * cos(theta1 + theta2);
    
    plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',8);
    hold on
    plot([0, a1*cos(theta1), x],[0, a1*sin(theta2), y],'r-o',...
        'LineWidth',1.5,'MarkerFaceColor','r','MarkerSize',5);
    
    %% Velocity or Manipulability ellipsoid
    % aa = [cos(t1), -sin(t1); sin(t1), cos(t1)]...
    %    * [xe * cosd(0:360); ye * sind(0:360)];
    aa = V * [xe * cosd(0:360); ye * sind(0:360)]; 
    plot(x + aa(1,:), y + aa(2,:), 'b-');
    hold on
    % plot(x + aa2(1,:), y + aa2(2,:), 'g-');
    % hold on
    
    %% Force ellipsoid
    % Axis are rotated by 90° it's eq to compute (J*J')^-1
    bb = V * [ye * sind(0:360); xe*cosd(0:360)];
    plot(x + bb(1,:), y + bb(2,:), 'm-');
    title('Velocity and Force Ellipsoid');
    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    axis([-2 2 -2 2])
    grid on
    hold off
    pause(0.1);

    theta1 = theta1 + precision;
    theta2 = theta2 + precision;
    theta4 = theta4 + precision;

end

%% Function to Plot the Manipulator Structure
function plotScara(p, xe, ye, ze, phi)

    % Plot Links
    plot3(p(1,:), p(2,:), p(3,:), '-o', 'LineWidth', 2, 'DisplayName',...
        'Links', 'Color','yellow');
    hold on

    % Plot joints
    scatter3(p(1,:), p(2,:), p(3,:), 100, 'filled', 'MarkerEdgeColor', ...
        'k', 'MarkerFaceColor', 'r', 'DisplayName', 'Joints');
    
    arrow = 0.1;
    
    % Plot base frame
    quiver3(0, 0, 0, arrow, 0, 0, 'LineWidth', 1.5,...
        'DisplayName', 'Base Frame X');
    quiver3(0, 0, 0, 0, arrow, 0, 'LineWidth', 1.5,...
        'DisplayName', 'Base Frame Y');
    quiver3(0, 0, 0, 0, 0, arrow, 'LineWidth', 1.5,...
        'DisplayName', 'Orientation Z');

    % End effector Orientation
    Rz = arrow * [cos(phi), -sin(phi), 0;
        sin(phi), cos(phi), 0;
        0, 0, -1];

    % Plot end effector frame
    quiver3(xe, ye, ze, Rz(1,1), Rz(1,2), Rz(1,3), 'LineWidth', 1.5,...
        'DisplayName', 'Orientation X');
    quiver3(xe, ye, ze, Rz(2,1), Rz(2,2), Rz(2,3), 'LineWidth', 1.5,...
        'DisplayName', 'Orientation Y');
    quiver3(xe, ye, ze, Rz(3,1), Rz(3,2), Rz(3,3), 'LineWidth', 1.5,...
        'DisplayName', 'Orientation Z');

    hold off

    title('Manipulator Links and Joints Pose');
    xlabel('x(t)'); ylabel('y(t)'); zlabel('z(t)');
    axis equal
    grid on

    % Add a legend
    legend('Location', 'Best');
end

