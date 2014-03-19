function hout = plot_puma_kuchenbe(x, y, z, phi, theta, psi, theta1, theta2, theta3, theta4, theta5, theta6, redValue, greenValue, blueValue, hin)
%
% This Matlab function is part of the starter code for the inverse
% kinematics part of Project 2 in MEAM 520 at the University of Pennsylvania.
% It plots many things, as explained below.
% 
% A gray coordinate frame at the position and orientation specified by the
% first six inputs.  x, y, and z specify the desired position of the origin
% of the sixth frame with respect to frame zero.  phi, theta, and psi are
% ZYZ Euler angles that specify the desired orientation of the sixth frame
% with respect to frame zero.
%
% The PUMA robot in the configuration specified by the six joint
% angle inputs, which are the seventh through twelfth arguments, named
% theta1 through theta 6.
%
% A point of light in the specified RGB color at the end-effector tip.  
% Each color value should range from 0 to 1.  Set the color to all zeros
% (black) to plot no point of light for this pose.
%
% This function returns a vector of handles to the items it has plotted so
% that this function can be used to update a plot rather than replace it.
%
% The final input argument, which is optional, is a vector of handles from
% a previous call to this function.  When provided, this function updates
% the plot rather than replacing it.


%% CHECK THE INPUTS

% Put joint angles into a column vector for simplicity.
thetas = [theta1 theta2 theta3 theta4 theta5 theta6]';

% We must check all of the thetas for NaN (not-a-number).
if (sum(isnan(thetas)) > 0)
    
    % At least one joint angle is NaN, so we cannot plot the robot as
    % requested.  Display a warning.
    %warning('NaN passed in for one or more joint angles.');
    
    % Put the robot in the home configuration and turn it red.
    theta1 = 0;
    theta2 = 0;
    theta3 = 0;
    theta4 = 0;
    theta5 = -pi/2;
    theta6 = 0;
    pcolor = [1 0 0];
    
else
    
    % All joint angles are real numbers.  Set the robot's color to be
    % normal, and don't modify any of the joint angles.
    pcolor = .8*[1 .88 .75];
    
end


%% PUMA FORWARD KINEMATICS

% Run the forward kinematics for the PUMA for these joint angles.
[points_to_plot, x06, y06, z06] = puma_fk_kuchenbe(theta1, theta2, theta3, theta4, theta5, theta6);

% Save the origin of frame 6 in its own variable for convenience.
o6 = points_to_plot(:,end);


%% DESIRED END-EFFECTOR COORDINATE FRAME

% Calculate sine and coside of the three Euler angles.
sph = sin(phi);
cph = cos(phi);
sth = sin(theta);
cth = cos(theta);
sps = sin(psi);
cps = cos(psi);
            
% Calculate desired rotation matrix from Euler angles.
Rdes = [cph -sph 0 ; sph cph 0 ; 0 0 1] * ...
       [cth 0 sth  ; 0 1 0     ; -sth 0 cth ] * ...
       [cps -sps 0 ; sps cps 0 ; 0 0 1];

% Put desired position in a column vector.
odes = [x ; y ; z];

% Assemble into homogeneous transformation.
Hdes = [Rdes odes; 0 0 0 1];

% Change odes to homogeneous representation.
odes = [odes ; 1];

% Length of coordinate frame vectors, in inches.
vlen = 8;

% Calculate the coordinates of the x, y, and z unit vectors of the desired
% location for frame 6 in frame 0.  We calculate the location of the end by
% multiplying Hdes into a scaled unit vector in the correct direction.
xdes = [odes (Hdes * [vlen 0 0 1]')];
ydes = [odes (Hdes * [0 vlen 0 1]')];
zdes = [odes (Hdes * [0 0 vlen 1]')];


%% PLOT ROBOT

if (nargin == 16)

    % The user has passed in an array of handles so the plot can be updated
    % instead of replaced.  Split them out. 
    hrobot = hin(1);
    hx06 = hin(2);
    hy06 = hin(3);
    hz06 = hin(4);
    hxdes = hin(5);
    hydes = hin(6);
    hzdes = hin(7);

    % Update the robot's position.
    set(hrobot, 'xdata', points_to_plot(1,:), 'ydata', points_to_plot(2,:), ...
        'zdata', points_to_plot(3,:),'color',pcolor);

    % Update the position of the axes of frame 6.
    set(hx06, 'xdata', x06(1,:), 'ydata', x06(2,:), 'zdata', x06(3,:));
    set(hy06, 'xdata', y06(1,:), 'ydata', y06(2,:), 'zdata', y06(3,:));
    set(hz06, 'xdata', z06(1,:), 'ydata', z06(2,:), 'zdata', z06(3,:));
    
    % If the color is not black, add this tip position to the plot.
    if (((redValue ~= 0) && (greenValue ~= 0)) && blueValue ~= 0)
        hold on
        plot3(o6(1),o6(2),o6(3),'.','color',[redValue greenValue blueValue],'markersize',10);
        hold off
    end
    
    % Update the position of the desired pose for frame 6.
    set(hxdes, 'xdata', xdes(1,:), 'ydata', xdes(2,:), 'zdata', xdes(3,:));
    set(hydes, 'xdata', ydes(1,:), 'ydata', ydes(2,:), 'zdata', ydes(3,:));
    set(hzdes, 'xdata', zdes(1,:), 'ydata', zdes(2,:), 'zdata', zdes(3,:));

    % Copy the inputted array of handles to the output.
    hout = hin;
else
    
    % This is the first time this function is being called.
    
    % Plot the robot in 3D using big dots at the points and thick lines
    % to connect neighboring points.  Keep a handle to the plot.
    hrobot = plot3(points_to_plot(1,:), points_to_plot(2,:), points_to_plot(3,:), ...
        '.-','linewidth',7,'markersize',35,'color',pcolor);
    
    % Call hold on so we can plot more things on this axis.
    hold on
    
    % Plot the x, y, and z axes of frame 6, keeping handles.
    hx06 = plot3(x06(1,:), x06(2,:), x06(3,:), 'w:', 'linewidth',2);
    hy06 = plot3(y06(1,:), y06(2,:), y06(3,:), 'w--', 'linewidth',2);
    hz06 = plot3(z06(1,:), z06(2,:), z06(3,:), 'w-', 'linewidth',2);
    
    % If the color is not black, plot the tip position.
    if (((redValue ~= 0) && (greenValue ~= 0)) && blueValue ~= 0)
        plot3(o6(1),o6(2),o6(3),'.','color',[redValue greenValue blueValue],'markersize',10);
    end
        
    % Plot the x, y, and z axes of the desired pose for frame 6, keeping handles.
    hxdes = plot3(xdes(1,:), xdes(2,:), xdes(3,:), ':', 'linewidth',2,'color',[.5 .5 .5]);
    hydes = plot3(ydes(1,:), ydes(2,:), ydes(3,:), '--', 'linewidth',2,'color',[.5 .5 .5]);
    hzdes = plot3(zdes(1,:), zdes(2,:), zdes(3,:), '-', 'linewidth',2,'color',[.5 .5 .5]);
    
    % Call hold off because we are done plotting things.
    hold off
    
    % Label the axes.
    xlabel('X (in.)')
    ylabel('Y (in.)')
    zlabel('Z (in.)')
    
    % Set the background color to be almost black.
    set(gca,'color',[.1 .1 .1])
    
    % Set the axis properties to make one unit the same in every
    % direction and enable 3D rotation.
    axis equal vis3d
    
    % Set the axis limits.
    axis([-20 20 -20 20 0 40])
    
    % Set the viewing angle.
    view(45,30)

    % Assemble handles into a vector to return.
    hout = [hrobot; hx06; hy06; hz06; hxdes; hydes; hzdes];

end
