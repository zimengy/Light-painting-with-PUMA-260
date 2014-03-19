%% PUMA Light Painting
%
% This Matlab script is part of the starter code for the light painting
% part of Project 2 in MEAM 520 at the University of Pennsylvania. 
% It uses the team's inverse kinematics to create a light painting.


%% Clean up

% Clear all variables and functions.  You should do this before calling any PUMA functions.
clear all

% Move the cursor to the top of the command window so new text is easily seen.
home


%% Definitions

% Define team number.
teamnumber = 217;

% Define student names.
studentnames = 'Zimeng Yang, Yixuan Wu, and Bokang Wang';

% Initialize the function that calculates positions, orientations, and
% colors by calling it once with no arguments.  It returns the duration of
% the light painting in seconds.
duration = team217_get_poc();


%% Choose duration

% Set the start and stop times of the segment we want to test.
% To play the entire painting, set tstart = 0 and tstop = duration.
tstart = 0;
tstop = duration;


%% Start robot

% Open figure 1 and clear it.
figure(1)
clf

% Initialize the PUMA simulation.
pumaStart('LEDMarkerStyle', '.', 'LEDMarkerSize', 4.5)

% Set the view so we are looking from roughly where the camera will be.
view(80,20)

% Set the title so we know whose painting this is.
title(['PUMA Light Painting by ' studentnames])

% The PUMA always starts with all joints at zero except joint 5, which
% starts at -pi/2 radians.  You can check this by calling pumaAngles.
thetahome = [0 0 0 0 -pi/2 0]';

% Call pumaServo once to initialize timers.
pumaServo(thetahome);
%pumaMove(thetahome);

% Turn the LED on.
pumaLEDOn

% Set the color of the LED to black so that we don't see it yet.
pumaLEDSet(0,0,0);


%% Initialize painting

% Get the position and orientation where the robot's end-effector should
% start, as well as the color it should be when it gets there.  We ignore
% the first returned value, which is the duration.
[~, xs, ys, zs, phis, thetas, psis, rs, gs, bs] = team217_get_poc(tstart);

% Get all possible solutions to the PUMA's full inverse kinematics for
% placing frame 6 at the desired position and orientation.  The
% returned variable allSolutions is a matrix where each of the six rows
% corresponds to one of the PUMA's six joints, in order.  Each column
% is a separate solution to the IK problem.  Angles are in radians.
allSolutions = team217_puma_ik(xs, ys, zs, phis, thetas, psis);
    
% Choose the best solution based on the robot's home position.  You may
% want to change this if you prefer a different solution.
thetastart = team217_choose_solution(allSolutions, thetahome);

% Calculate time needed to get from home pose to starting pose moving at
% angular speed of 0.5 radians per second on the joint that has the
% farthest to go.
tprep = max(abs(thetastart - thetahome)) / .5;

% Start the built-in MATLAB timer.
tic

% Slowly servo the robot to its starting position, in case we're
% not starting at the beginning of the painting.
while(true)
    
    % Get the current time for preparation move.
    tnow = toc;
    
    % Check to see whether preparation move is done.
    if (tnow > tprep)
        
        % Servo the robot to the starting pose.
        pumaServo(thetastart)
        %pumaMove(thetastart)
        
        % Break out of the infinite while loop.
        break

    end
    
    % Calculate joint angles.
    thetanow = linear_trajectory_kuchenbe(tnow,0,tprep,thetahome,thetastart);

    % Servo the robot to this pose to prepare to paint.
    pumaServo(thetanow);
    %pumaMove(thetanow);


end

% Set the LED to the desired starting color.
pumaLEDSet(rs, gs, bs)


%% Initialize storage and start timer

% Initialize history vectors for holding time and angles.  We preallocate
% these for speed, making them much larger than we will need.
thistory = zeros(10000,1);
thetahistory = zeros(10000,6);

% Initialize our counter at zero.
i = 1;

% Put time = 0 and the home position at the start of the history.
thistory(i) = 0;
thetahistory(i,:) = thetahome';

% Start the built-in MATLAB timer so we can keep track of where we are in
% the painting.
tic


%% Paint

% Enter an infinite loop.
while(true)
    
    % Increment our counter.
    i = i+1;
    
    % Get the current time elapsed and add it to the time where we're
    % starting in the painting. Store this value in the thistory vector.   
    thistory(i) = toc + tstart;
    
    % Check if we have passed the end of the performance.
    if (thistory(i) > tstop)
        
        % Break out of the infinite loop.
        break

    end
    
    % Get the position, orientation, and color for the LED at this time.
    [~, x, y, z, phi, theta, psi, r, g, b] = team217_get_poc(thistory(i));
    
    % Set the LED to the desired color.
    pumaLEDSet(r, g, b)
    
    % Get all possible solutions to the PUMA's full inverse kinematics for
    % placing frame 6 at the desired position and orientation.  The
    % returned variable allSolutions is a matrix where each of the six rows
    % corresponds to one of the PUMA's six joints, in order.  Each column
    % is a separate solution to the IK problem.  Angles are in radians.
    allSolutions = team217_puma_ik(x, y, z, phi, theta, psi);
    
    % Choose the best solution based on the robot's current pose and other factors.
    thetas = team217_choose_solution(allSolutions, thetahistory(i-1,:)');
    
    % Check for any NaN values in the solution.
    if (any(isnan(thetas)))
        
        % Tell the user.
        disp('NaN value returned as best solution.')
        
        % Break out of the infinite loop.
        break
    
    else
        
        % Store this best solution in our theta history matrix.
        thetahistory(i,:) = thetas';
        
    end

    % Servo the robot to these new joint angles.
    pumaServo(thetahistory(i,:));
    %pumaMove(thetahistory(i,:));
end

% Turn off the LED.
pumaLEDOff

% Stop the PUMA robot.
pumaStop
        
% Remove the unused ends of the history vector and matrix, which we
% preallocated for speed.
thistory(i:end) = [];
thetahistory(i:end,:) = [];

