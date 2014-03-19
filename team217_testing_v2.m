%% team217_testing.m
%
% This Matlab script is part of the starter code for the inverse
% kinematics part of Project 2 in MEAM 520 at the University of Pennsylvania.
% It tests the team's inverse kinematics code for various configurations.

%% SETUP

% Clear all variables from the workspace.
clear all

% Clear the console, so you can more easily find any errors that may occur.
clc

% Set whether to animate the robot's movement and how much to slow it down.
pause on;  % Set this to off if you don't want to watch the animation.
GraphingTimeDelay = 0.05; % The length of time that Matlab should pause between positions when graphing, if at all, in seconds.


%% CHOOSE INPUT PARAMETERS

% Set the test type.
testType = 5;

switch(testType)

    case 1

        % One position and orientation that you can manually modify.

        % Define the configuration.
        ox_history = 10.5; % inches
        oy_history = 5; % inches
        oz_history = 21; % inches
        phi_history = 0; % radians
        theta_history = 0; % radians
        psi_history = 0; % radians
        
    case 2
        
        % Linear interpolation between two points and two sets of Euler angles.
        
        % Create a time vector.
        t = (0:0.2:2*pi)';

        % Define starting configuration.
        ox_start = 10.5; % inches
        oy_start = -5; % inches
        oz_start = 21; % inches
        phi_start = 0; % radians
        theta_start = -pi/2; % radians
        psi_start = 0; % radians

        % Define ending configuration.
        ox_end = 5; % inches
        oy_end = 15; % inches
        oz_end = 10; % inches
        phi_end = pi; % radians
        theta_end = -pi/2; % radians
        psi_end = 0; % radians

        % Do the interpolation.
        ox_history = ox_start * ones(length(t),1) + (ox_end - ox_start)*(t - t(1))./(t(end) - t(1));
        oy_history = oy_start * ones(length(t),1) + (oy_end - oy_start)*(t - t(1))./(t(end) - t(1));
        oz_history = oz_start * ones(length(t),1) + (oz_end - oz_start)*(t - t(1))./(t(end) - t(1));
        phi_history = phi_start * ones(length(t),1) + (phi_end - phi_start)*(t - t(1))./(t(end) - t(1));
        theta_history = theta_start * ones(length(t),1) + (theta_end - theta_start)*(t - t(1))./(t(end) - t(1));
        psi_history = psi_start * ones(length(t),1) + (psi_end - psi_start)*(t - t(1))./(t(end) - t(1));
        
    case 3

        % A vertical circle parallel to the x-z plane.

        % Define the radius of the circle.
        radius = 4; % inches
        
        % Define the y-value for the plane that contains the circle.
        y_offset = 7; % inches
        
        % Define the x and z coordinates for the center of the circle.
        x_center = 4; % inches
        z_center = 15; % inches
        
        % Create a time vector.
        t = (0:0.2:2*pi)';
        
        % Set the desired x, y, and z positions over time given the circle parameters.
        ox_history = x_center + radius * sin(t);
        oy_history = y_offset * ones(size(t));
        oz_history = z_center + radius * cos(t);
        
        % Set the desired Euler Angles to be constant for the duration of
        % the test.
        phi_history = 0*ones(length(t),1); % radians
        theta_history = 0*ones(length(t),1); % radians
        psi_history = pi/4*ones(length(t),1); % radians
        
    case 4
        
        % A set number of random positions and random orientations.  
        
        % Note that these are not all guaranteed to be reachable, nor
        % is this function guaranteed to cover the full reachable
        % workspace.  But it's a decent start.
        
        % Set the number of random poses to generate.
        nPoses = 10;
        
        % Pick a random radius approximately matched to the size of the robot.
        rr = 5 + rand(nPoses,1) * (10); % inches
        
        % Pick random values for two angles of spherical coordinates.
        thr = rand(nPoses,1) * 2 * pi; % radians
        phr = -pi/3 + rand(nPoses,1) * 2*pi/3; % radians
        
        % Set random tip position.
        ox_history = rr .* cos(phr) .* cos(thr);
        oy_history = rr .* cos(phr) .* sin(thr);
        oz_history = 13 + rr .* sin(phr);
        
        % Set random Euler angles.
        phi_history = rand(nPoses,1) * 2 * pi;
        theta_history = -pi/2 + rand(nPoses,1) * pi;
        psi_history = rand(nPoses,1) * 2 * pi;
        
    case 5
        
        % Test the positions and orientations from the team's light painting.
        
        % Initialize the function that calculates positions, orientations, and
        % colors by calling it once with no arguments.  It returns the duration of
        % the light painting in seconds.
        duration = team217_get_poc();
        
        % Create time vector.
        t = 0:0.1:duration;
        
        % Step through the time vector, filling the histories by calling
        % the function that retuns positions, orientations, and colors.
        for i = 1:length(t)
            [~, ox_history(i), oy_history(i), oz_history(i), phi_history(i), theta_history(i), psi_history(i)] = team217_get_poc(t(i));
        end
        
    otherwise
        error(['Unknown testType: ' num2str(testType)])
end    
    

%% TEST

% Notify the user that we're starting the test.
disp('Starting the test.')

% Show a message to explain how to cancel the test and graphing.
disp('Click in this window and press control-c to stop the code.')

% Plot the robot once in the home position to get the plot handles.
figure(1)
h = plot_puma_kuchenbe(0,0,0,0,0,0,0,0,0,0,-pi/2,0,0,0,0);

% Step through the ox_history vector to test the inverse kinematics.
for i = 1:length(ox_history)
    
    % Pull the current values of ox, oy, and oz from their histories. 
    ox = ox_history(i);
    oy = oy_history(i);
    oz = oz_history(i);
    
    % Pull the current values of phi, theta, and psi from their histories. 
    phi = phi_history(i);
    theta = theta_history(i);
    psi = psi_history(i);

    % Send the desired pose into the inverse kinematics to obtain the joint
    % angles that will place the PUMA's end-effector at this position and
    % orientation relative to frame 0.
    thetas = team217_puma_ik(ox, oy, oz, phi, theta, psi);
    
    % For each of the columns in thetas.
    s = size(thetas);
    for j = 1:s(2)

        % Plot the robot at this IK solution.        
        plot_puma_kuchenbe(ox,oy,oz,phi,theta,psi,thetas(1,j),thetas(2,j),thetas(3,j),thetas(4,j),thetas(5,j),thetas(6,j),0,1,0,h);

        % Set the title to show the viewer which pose and result this is.
        title(['Test ' num2str(testType) ' - Pose ' num2str(i) ' - Solution ' num2str(j)])
        
        % Pause for a short duration so that the viewer can watch animation evolve over time.
        pause(GraphingTimeDelay)

    end
    
    % Pause for a short duration so that the viewer can watch animation evolve over time.
    pause(GraphingTimeDelay)
    
end

%% FINISHED

% Tell the use.
disp('Done with the test.')
