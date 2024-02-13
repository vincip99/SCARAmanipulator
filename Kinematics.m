%% Direct Kinematics
clear all

%% Parameter Initialization
% Init the SCARA manipulator kiinimetic parameters
d0 = 1; a1 = .5; a2 = .5;
% numLinks
numLinks = 5;

% Init joint variables
theta1 = pi/2; theta2 = -pi/4; d3 = .25; theta4 = pi/3; 

%% Homogeneus Transformation matrix
p = scaraHomogeneusTransf(a1,a2,d0,numLinks,theta1,theta2,d3,theta4);

%% Direct kinematics 
[xe, ye, ze, phi] = scaraDirectKinematic(a1,a2,d0,theta1,theta2,d3,theta4);

figure
%% Plot SCARA manipulator
subplot(2,1,1);
plotScara(p,xe,ye,ze,phi);
axis equal; %responsive

%% Geometrical Jacobian
J = scaraJacobian(a1,a2,theta1,theta2);

fprintf('\nJacobian matrix\n');
disp(J);
disp(det(J));

%% Velocity and Force Ellipsoid
[velEllips, forceEllips] = scaraEllipsoid(J);

subplot(2,1,2);
plotScaraEllipsoids(velEllips, forceEllips, xe, ye, a1, theta1)
axis equal; %responsive

%% Moving SCARA for some joint values in a straigth line

straightLine(a1,a2,d0,theta4,d3,numLinks)

%% Ellipsoid Calculation for different joint variables value

precision = 0.1; % precision of animation 

figure
for i = theta1:precision:theta1 + 2*pi
    
    % Homogeneus Transformation matrix
    p = scaraHomogeneusTransf(a1,a2,d0,numLinks,theta1,theta2,d3,theta4);
    
    % Direct kinematics 
    [xe, ye, ze, phi] = scaraDirectKinematic(a1,a2,d0,theta1,theta2,d3,theta4);
    
    % Plot SCARA manipulator
    subplot(2,1,1);
    plotScara(p,xe,ye,ze,phi);
    axis equal; %responsive
    
    % Geometrical Jacobian
    J = scaraJacobian(a1,a2,theta1,theta2);
    
    fprintf('\nJacobian matrix\n');
    disp(J);
    
    % Velocity and Force Ellipsoid
    [velEllips, forceEllips] = scaraEllipsoid(J);
    
    subplot(2,1,2);
    plotScaraEllipsoids(velEllips, forceEllips, xe, ye, a1, theta1)
    axis equal; %responsive

    % Iteration
    pause(0.1);

    theta1 = theta1 + precision;
    theta2 = theta2 + precision;
    theta4 = theta4 + precision;

end

clear i precision;

%% Homogeneus Transformation function
function [p, T, Pe, yaw, pitch, roll] = scaraHomogeneusTransf(a1,a2,d0,...
    numLinks,theta1,theta2,d3,theta4)

    %% Init DH matrix
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

end

%% Direct Kinematic function
function [xe, ye, ze, phi] = scaraDirectKinematic(a1,a2,d0,theta1,...
    theta2,d3,theta4)

    xe = a1*cos(theta1) + a2*cos(theta1 + theta2);
    ye = a1*sin(theta1) + a2*sin(theta1 + theta2);
    ze = d0 - d3;
    phi = theta1 + theta2 - theta4;

end

%% Inverse Kinematic function
function [theta1,theta2,d3,theta4] = scaraInverseKinematic(xe, ye, ze, phi)


    % Calculate joint angles using geometric relationships
    theta1 = atan2(ye, xe);
    D = (x^2 + y^2 - a1^2 - a2^2) / (2 * a1 * a2);
    theta2 = atan2(sqrt(1 - D^2), D);
    
    % Calculate joint variables
    theta4 = phi - theta1 - theta2;
    d3 = ze - d0;

end

%% Jacobian function
function [J, determinant] = scaraJacobian(a1,a2,theta1,theta2)

        J = [-a1*sin(theta1) - a2*sin(theta1 + theta2), - a2*sin(theta1 + theta2), 0, 0;
        a1*cos(theta1) + a2*cos(theta1 + theta2), a2*cos(theta1 + theta2), 0, 0;
        0, 0, -1, 0;
        1, 1, 0, -1;];

        determinant = det(J);
end

%% Function to Plot the Manipulator Structure
function plotScara(p, xe, ye, ze, phi)

    % Plot Links
    plot3(p(1,:), p(2,:), p(3,:), '-o', 'LineWidth', 2,...
        'Color',[0.5, 0.5, 0.5]);
    hold on

    % Plot joints
    scatter3(p(1,:), p(2,:), p(3,:), 100, 'filled', 'MarkerEdgeColor', ...
        'k', 'MarkerFaceColor', 'r');
    
    arrow = 0.1;    % Arrow length
    
    % Plot base frame
    quiver3(0, 0, 0, arrow, 0, 0, 'LineWidth', 1.5);
    quiver3(0, 0, 0, 0, arrow, 0, 'LineWidth', 1.5);
    quiver3(0, 0, 0, 0, 0, arrow, 'LineWidth', 1.5);

    % End effector Orientation
    Rz = arrow * [cos(phi), -sin(phi), 0;
        sin(phi), cos(phi), 0;
        0, 0, -1];

    % Plot end effector frame
    quiver3(xe, ye, ze, Rz(1,1), Rz(1,2), Rz(1,3), 'LineWidth', 1.5);
    quiver3(xe, ye, ze, Rz(2,1), Rz(2,2), Rz(2,3), 'LineWidth', 1.5);
    quiver3(xe, ye, ze, Rz(3,1), Rz(3,2), Rz(3,3), 'LineWidth', 1.5);

    hold off
    
    % Add a title
    title('Manipulator Links and Joints Pose');
    xlabel('x(t), [m]'); ylabel('y(t), [m]'); zlabel('z(t), [m]');
    axis equal
    grid on

    % Add a legend
    % legend('Location', 'BestOutside');
    legend('Links','Joints','Base Frame X','Base Frame Y','Base Frame Z',...
        'Orientation X','Orientation Y','Orientation Z','Location',...
        'BestOutside');
end

%% Function to use for velocity and force ellipsoid computation
function [velEllips, forceEllips, manipulability] = scaraEllipsoid(J)
    
    %% Forward Kinematics of 2 link approximation for the SCARA Ellipsoid
    J = J(1:2,1:2);

    %% Eigenvalues and Eigenvectors
    [V, D] = eig(J*J');
    ev = diag(D);
    ax = sqrt((abs(ev(1))));
    ay = sqrt((abs(ev(2))));
    
    %% Velocity or Manipulability ellipsoid
    velEllips = V * [ax * cosd(0:360); ay * sind(0:360)]; 
    
    %% Force ellipsoid
    % Axis are rotated by 90° it's eq to compute (J*J')^-1
    forceEllips = V * [ay * sind(0:360); ax * cosd(0:360)];
    
    %% Manipulability measure
    manipulability = sqrt(det(J*J'));

end

%% Function to plot ellipsoid
function plotScaraEllipsoids(velEllips, forceEllips, xe, ye, a1, theta1)

    % Plot 2D SCARA approximation
    plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',8);
    hold on;
    plot([0, a1*cos(theta1), xe],[0, a1*sin(theta1), ye],'r-o',...
        'LineWidth',1.5,'MarkerFaceColor','r','MarkerSize',5);

    
    % Plot Velocity or Manipulability ellipsoid
    plot(xe + velEllips(1,:), ye + velEllips(2,:), 'b-');
    hold on

    % Plot Force ellipsoid
    plot(xe + forceEllips(1,:), ye + forceEllips(2,:), 'm-');
    hold off

    % add title and legend
    title('Velocity and Force Ellipsoid');
    xlabel('xe, [m]'); ylabel('ye, [m]');
    legend('Base', 'Arm', 'Velocity Ellipsoid', 'Force Ellipsoid',...
        'Location','BestOutside');

    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    axis equal;
    grid on;
    
end

%% Function to plot ellipsoid
function plotScaraEllipsoid(Ellips, xe, ye, a1, theta1)

    % Plot 2D SCARA approximation
    plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',8);
    hold on;
    plot([0, a1*cos(theta1), xe],[0, a1*sin(theta1), ye],'r-o',...
        'LineWidth',1.5,'MarkerFaceColor','r','MarkerSize',5);

    
    % Plot Manipulability ellipsoid
    plot(xe + Ellips(1,:), ye + Ellips(2,:), 'g-');
    hold on

    % add title and legend
    title('Ellipsoid');
    xlabel('xe, [m]'); ylabel('ye, [m]');
    legend('Base', 'Arm', 'Ellipsoid','Location','BestOutside');

    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    axis equal;
    grid on;
    
end

%% Routine functions
function straightLine(a1,a2,d0,theta4,d3,numLinks)
%% Moving SCARA for some joint values in a straigth line
    % joint variables of interest
    theta1 = pi/2; theta2 = -pi;
    
    figure
    while theta1 >= pi/32
        
        % computing DK Jacobian and ellipsoid
        p = scaraHomogeneusTransf(a1,a2,d0,numLinks,theta1,theta2,d3,theta4);
        [xe, ye, ze, phi] = scaraDirectKinematic(a1,a2,d0,theta1,theta2,d3,theta4);
        J = scaraJacobian(a1,a2,theta1,theta2);
        [velEllips1, forceEllips1, manipulability] = scaraEllipsoid(J);
        disp('manipulability')
        disp(manipulability)
    
        % Plot
        subplot(2,2,1);
        plotScara(p, xe, ye, ze, phi);
        axis equal;
        title('Robot Manipulator Pose');
        hold on
        
        subplot(2,2,2);
        plotScaraEllipsoids(velEllips1, forceEllips1, xe, ye, a1, theta1);
        axis equal;
        title('Manipulability Ellipsoids');
        hold on
        subplot(2,2,3);
        plotScaraEllipsoid(velEllips1, xe, ye, a1, theta1);
        axis equal;
        title('Velocity Ellipsoids');
        hold on
        subplot(2,2,4);
        plotScaraEllipsoid(forceEllips1, xe, ye, a1, theta1);
        axis equal;
        title('Force Ellipsoids');
        hold on
        
        % Update joint variables
        theta1 = theta1/2;
        theta2 = theta2/2;
        
        % Pause to visualize each step
        pause(0.1);
    
    end
    hold off

end
