function [points_to_plot, x06, y06, z06] = puma_fk_kuchenbe(theta1, theta2, theta3, theta4, theta5, theta6)
%
% Given the six joint angles, this function returns the Cartesian
% coordinates of points along the PUMA 260 as well as the Cartesian
% coordinates for the starting and ending points of vectors along the x, y,
% and z axes of the robot's sixth frame (the end-effector frame).

%% ROBOT PARAMETERS

% This problem is about the PUMA 260 robot, a 6-DOF manipulator.

% Define the robot's measurements.  These correspond are constant.
a = 13.0; % inches
b =  2.5; % inches
c =  8.0; % inches
d =  2.5; % inches
e =  8.0; % inches
f =  2.5; % inches

% Length of coordinate frame vectors, in inches.
vlen = 8;


%% DH MATRICES

% Calculate the six A matrices using the provided DH function.
A1 = dh_kuchenbe(0,  pi/2,   a, theta1);
A2 = dh_kuchenbe(c,     0,  -b, theta2);
A3 = dh_kuchenbe(0, -pi/2,  -d, theta3);
A4 = dh_kuchenbe(0,  pi/2,   e, theta4);
A5 = dh_kuchenbe(0, -pi/2,   0, theta5);
A6 = dh_kuchenbe(0,     0,   f, theta6);


%% ORIGIN POSITIONS

% Define the homogeneous representation of the origin of any frame.
o = [0 0 0 1]';

% Calculate the position of the origin of each frame in the base frame.
o0 = o;
o1 = A1*o;
extra_point = A1*[0 0 -b 1]';
o2 = A1*A2*o;
o3 = A1*A2*A3*o;
o4 = A1*A2*A3*A4*o;
o5 = A1*A2*A3*A4*A5*o;
o6 = A1*A2*A3*A4*A5*A6*o;

% Put the seven origin points together for plotting.
points_to_plot = [o0 o1 extra_point o2 o3 o4 o5 o6];


%% END-EFFECTOR COORDINATE FRAME 

% Save the full transformation in T06.
T06 = A1*A2*A3*A4*A5*A6;

% Calculate the coordinates of the x, y, and z unit vectors of frame 6 in
% frame 0. Each of these vectors starts at the origin of frame 6 and ends
% at the distance vlen along the designated axis.  We calculate the
% location of the end by multiplying T06 into a scaled unit vector in the
% correct direction.
x06 = [o6 (T06 * [vlen 0 0 1]')];
y06 = [o6 (T06 * [0 vlen 0 1]')];
z06 = [o6 (T06 * [0 0 vlen 1]')];
