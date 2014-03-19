function thetas = team217_puma_ik(x, y, z, phi, theta, psi)
%% team200_puma_ik.m
%
% Calculates the full inverse kinematics for the PUMA 260.
%
% This Matlab file provides the starter code for the PUMA 260 inverse
% kinematics function of project 2 in MEAM 520 at the University of
% Pennsylvania.  The original was written by Professor Katherine J.
% Kuchenbecker. Students will work in teams modify this code to create
% their own script. Post questions on the class's Piazza forum. 
%
% The first three input arguments (x, y, z) are the desired coordinates of
% the PUMA's end-effector tip in inches, specified in the base frame.  The
% origin of the base frame is where the first joint axis (waist) intersects
% the table. The z0 axis points up, and the x0 axis points out away from
% the robot, perpendicular to the front edge of the table.  These arguments
% are mandatory.
%
%     x: x-coordinate of the origin of frame 6 in frame 0, in inches
%     y: y-coordinate of the origin of frame 6 in frame 0, in inches
%     z: z-coordinate of the origin of frame 6 in frame 0, in inches
%
% The fourth through sixth input arguments (phi, theta, psi) represent the
% desired orientation of the PUMA's end-effector in the base frame using
% ZYZ Euler angles in radians.  These arguments are mandatory.
%
%     phi: first ZYZ Euler angle to represent orientation of frame 6 in frame 0, in radians
%     theta: second ZYZ Euler angle to represent orientation of frame 6 in frame 0, in radians
%     psi: third ZYZ Euler angle to represent orientation of frame 6 in frame 0, in radians
%
% The output (thetas) is a matrix that contains the joint angles needed to
% place the PUMA's end-effector at the desired position and in the desired
% orientation. The first row is theta1, the second row is theta2, etc., so
% it has six rows.  The number of columns is the number of inverse
% kinematics solutions that were found; each column should contain a set
% of joint angles that place the robot's end-effector in the desired pose.
% These joint angles are specified in radians according to the
% order, zeroing, and sign conventions described in the documentation.  If
% this function cannot find a solution to the inverse kinematics problem,
% it will pass back NaN (not a number) for all of the thetas.
%
% Please change the name of this file and the function declaration on the
% first line above to include your team number rather than 200.


%% CHECK INPUTS

% Look at the number of arguments the user has passed in to make sure this
% function is being called correctly.
if (nargin < 6)
    error('Not enough input arguments.  You need six.')
elseif (nargin == 6)
    % This the correct way to call this function, so we don't need to do
    % anything special.
elseif (nargin > 6)
    error('Too many input arguments.  You need six.')
end


%% CALCULATE INVERSE KINEMATICS SOLUTION(S)

% For now, just set the first solution to NaN (not a number) and the second
% to zero radians.  You will need to update this code.
% NaN is what you should output if there is no solution to the inverse
% kinematics problem for the position and orientation that were passed in.
% For example, this would be the correct output if the desired position for
% the end-effector was outside the robot's reachable workspace.  We use
% this sentinel value of NaN to be sure that the code calling this function
% can tell that something is wrong and shut down the PUMA.

% Set the end-effector orientation that we want.

%% Inverse position kinematics
% Define the robot's measurements.  These correspond are constant.
a = 13.0; % inches
b =  2.5; % inches
c =  8.0; % inches
d =  2.5; % inches
e =  8.0; % inches
f =  2.5; % inches

% Calculate the end effector frame R06.
Rzphi = [cos(phi) -sin(phi) 0; ...
         sin(phi) cos(phi)  0; ...
          0         0       1];
Rytheta = [cos(theta) 0 sin(theta); ...
           0          1      0    ; ...
           -sin(theta) 0 cos(theta)];
Rzpsi = [cos(psi) -sin(psi) 0; ...
         sin(psi)  cos(psi) 0; ...
           0         0      1];
R06 = Rzphi * Rytheta * Rzpsi;
 
% Calculate where the wrist center position oc needs to be.
oc = [x y z]' - f*R06(:,3);

% Pull out the components of wrist center and store in xc, yc, zc.
xc = oc(1);
yc = oc(2);
zc = oc(3);

% Different situations lead to different solutions
if xc^2+yc^2-(b+d)^2 < 0
    % There is no solution for theta1.
    th1 = [NaN];
    th2 = [NaN];
    th3 = [NaN];
    th4 = [NaN];
    th5 = [NaN];
    th6 = [NaN];
    
else

    % Do inverse position kinematics and calculate theta1, theta2, theta3.
    % Calculate theta1.
    if xc^2+yc^2-(b+d)^2 == 0 
        % Singularity configurations. Only one solution for theta1. Here we
        % assign two identical solutions to theta1, which will be eliminated to
        % one set at the end.

        theta1_1 = atan2(yc,xc)- pi/2;
        theta1_2 = atan2(yc,xc)- pi/2;
    else
        theta1_1 = atan2(yc,xc) - atan2(b+d, sqrt(xc^2+yc^2-(b+d)^2));
        theta1_2 = atan2(yc,xc) + atan2(-(b+d), -sqrt(xc^2+yc^2-(b+d)^2));
    end
    % Calculate theta3.
    r = sqrt(xc^2+yc^2-(b+d)^2);
    s = zc-a;
    cos_temp = (e^2+c^2-r^2-s^2)/2/e/c;
    if abs(cos_temp)>1
        % This is when the desired position is outside of the robot's reachable
        % workspace.
        th1 = [NaN];
        th2 = [NaN];
        th3 = [NaN];
        th4 = [NaN];
        th5 = [NaN];
        th6 = [NaN];
    else
        temp = atan2(sqrt(1-cos_temp^2),cos_temp);

        theta3_1 = pi/2 - temp;
        theta3_2 = pi/2 + temp;

        % Calculate theta2.
        theta2_1 = atan2(s,r) - atan2(e*cos(theta3_1), c-e*sin(theta3_1));
        theta2_2 = atan2(s,r) - atan2(e*cos(theta3_2), c-e*sin(theta3_2));

        theta3_3 = pi-theta3_1;
        theta2_3 = pi-theta2_1;

        theta3_4 = pi-theta3_2;
        theta2_4 = pi-theta2_2;  

        % Thus the four solutions for the arm are: 
        % 1. theta1_1, theta2_1, theta3_1; 
        % 2. theta1_1, theta2_2, theta3_2; 
        % 3. theta1_2, theta2_3, theta3_3; 
        % 4. theta1_2, theta2_4, theta3_4; 

        %% Inverse orientation kinematics

        % Calculate R03 using forward kinematics.
        %%% 1st pair of solutions
        A1 = dh_kuchenbe(0,  pi/2,   a, theta1_1);
        A2 = dh_kuchenbe(c,     0,  -b, theta2_1);
        A3 = dh_kuchenbe(0, -pi/2,  -d, theta3_1);

        A03 = A1*A2*A3;
        R03 = A03(1:3, 1:3);

        % Calculate R36.
        R36 = R03' * R06;

        ctheta = R36(3,3);
        stheta_pos = sqrt(1-ctheta^2);
        stheta_neg = - sqrt(1-ctheta^2);
        theta5_1 = -atan2(stheta_pos, ctheta);
        theta4_1 = atan2(R36(2,3), R36(1,3));
        theta6_1 = atan2(R36(3,2), -R36(3,1));

        theta5_2 = -atan2(stheta_neg, ctheta);
        theta4_2 = atan2(-R36(2,3), -R36(1,3));
        theta6_2 = atan2(-R36(3,2), R36(3,1));

        if R36(2,3) ==0 && R36(1,3) == 0
            theta4_1 = 0;
            theta6_1 = 0;
            theta4_2 = pi;
            theta6_2 = pi;
        end

        %%% 2nd pair of solutions
        A1 = dh_kuchenbe(0,  pi/2,   a, theta1_1);
        A2 = dh_kuchenbe(c,     0,  -b, theta2_2);
        A3 = dh_kuchenbe(0, -pi/2,  -d, theta3_2);

        A03 = A1*A2*A3;
        R03 = A03(1:3, 1:3);

        % Calculate R36.
        R36 = R03' * R06;

        ctheta = R36(3,3);
        stheta_pos = sqrt(1-ctheta^2);
        stheta_neg = - sqrt(1-ctheta^2);
        theta5_3 = -atan2(stheta_pos, ctheta);
        theta4_3 = atan2(R36(2,3), R36(1,3));
        theta6_3 = atan2(R36(3,2), -R36(3,1));

        theta5_4 = -atan2(stheta_neg, ctheta);
        theta4_4 = atan2(-R36(2,3), -R36(1,3));
        theta6_4 = atan2(-R36(3,2), R36(3,1));

        if R36(2,3) ==0 && R36(1,3) == 0
            theta4_3 = 0;
            theta6_3 = 0;
            theta4_4 = pi;
            theta6_4 = pi;
        end

        %%% 3rd pair of solutions
        A1 = dh_kuchenbe(0,  pi/2,   a, theta1_2);
        A2 = dh_kuchenbe(c,     0,  -b, theta2_3);
        A3 = dh_kuchenbe(0, -pi/2,  -d, theta3_3);

        A03 = A1*A2*A3;
        R03 = A03(1:3, 1:3);

        % Calculate R36.
        R36 = R03' * R06;

        ctheta = R36(3,3);
        stheta_pos = sqrt(1-ctheta^2);
        stheta_neg = - sqrt(1-ctheta^2);
        theta5_5 = -atan2(stheta_pos, ctheta);
        theta4_5 = atan2(R36(2,3), R36(1,3));
        theta6_5 = atan2(R36(3,2), -R36(3,1));

        theta5_6 = -atan2(stheta_neg, ctheta);
        theta4_6 = atan2(-R36(2,3), -R36(1,3));
        theta6_6 = atan2(-R36(3,2), R36(3,1));

        if R36(2,3) ==0 && R36(1,3) == 0
            theta4_5 = 0;
            theta6_5 = 0;
            theta4_6 = pi;
            theta6_6 = pi;
        end

        %%% 4th pair of solutions
        A1 = dh_kuchenbe(0,  pi/2,   a, theta1_2);
        A2 = dh_kuchenbe(c,     0,  -b, theta2_4);
        A3 = dh_kuchenbe(0, -pi/2,  -d, theta3_4);

        A03 = A1*A2*A3;
        R03 = A03(1:3, 1:3);

        % Calculate R36.
        R36 = R03' * R06;

        ctheta = R36(3,3);
        stheta_pos = sqrt(1-ctheta^2);
        stheta_neg = - sqrt(1-ctheta^2);
        theta5_7 = -atan2(stheta_pos, ctheta);
        theta4_7 = atan2(R36(2,3), R36(1,3));
        theta6_7 = atan2(R36(3,2), -R36(3,1));

        theta5_8 = -atan2(stheta_neg, ctheta);
        theta4_8 = atan2(-R36(2,3), -R36(1,3));
        theta6_8 = atan2(-R36(3,2), R36(3,1));

        if R36(2,3) ==0 && R36(1,3) == 0
            theta4_7 = 0;
            theta6_7 = 0;
            theta4_8 = pi;
            theta6_8 = pi;
        end
        % Put all the solutions of theta together
        if xc^2+yc^2-(b+d)^2 == 0 
            % Under singularity configurations, there are four solutions.
            th1 = [theta1_1 theta1_1 theta1_1 theta1_1 ];
            th2 = [theta2_1 theta2_2 theta2_1 theta2_2 ];
            th3 = [theta3_1 theta3_2 theta3_1 theta3_2 ];
            th4 = [theta4_1 theta4_3 theta4_2 theta4_4 ];
            th5 = [theta5_1 theta5_3 theta5_2 theta5_4 ];
            th6 = [theta6_1 theta6_3 theta6_2 theta6_4 ];
        else
            % Normal configurations, there are eight solutions.
            th1 = [theta1_1 theta1_1 theta1_2 theta1_2 theta1_1 theta1_1 theta1_2 theta1_2];
            th2 = [theta2_1 theta2_2 theta2_3 theta2_4 theta2_1 theta2_2 theta2_3 theta2_4];
            th3 = [theta3_1 theta3_2 theta3_3 theta3_4 theta3_1 theta3_2 theta3_3 theta3_4];
            th4 = [theta4_1 theta4_3 theta4_5 theta4_7 theta4_2 theta4_4 theta4_6 theta4_8];
            th5 = [theta5_1 theta5_3 theta5_5 theta5_7 theta5_2 theta5_4 theta5_6 theta5_8];
            th6 = [theta6_1 theta6_3 theta6_5 theta6_7 theta6_2 theta6_4 theta6_6 theta6_8];
        end
    end
end

% You should update this section of the code with your IK solution.
% Please comment your code to explain what you are doing at each step.
% Feel free to create additional functions as needed - please name them all
% to start with team2XX_, where 2XX is your team number.  For example, it
% probably makes sense to handle inverse position kinematics and inverse
% orientation kinematics separately.


%% FORMAT OUTPUT

% Put all of the thetas into a column vector to return.
thetas = [th1; th2; th3; th4; th5; th6];

% By the very end, each column of thetas should hold a set of joint angles
% in radians that will put the PUMA's end-effector in the desired
% configuration.  If the desired configuration is not reachable, set all of
% the joint angles to NaN.