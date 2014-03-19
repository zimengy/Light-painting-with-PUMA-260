function A = dh_kuchenbe(a, alpha, d, theta)

% This function calculates the four-by-four transformation matrix A for a
% given set of DH parameters: a, alpha, d, and theta.  The angles are in
% radians.

% IMPORTANT: change the name of this file and the function definition on
% the first line of this file to include your PennKey rather than
% "starter".  The name of the file and the line above should match.

% Calculate the A matrix that corresponds to a, alpha, d, and theta.  
% This is taken from equation (3.10) in SHV.
A = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
     sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
     0                      sin(alpha)             cos(alpha)            d;
     0                               0                      0            1];

% Another good way to do this is to build the four constituent matrices
% (rotation by theta around z, translation by d along z, translation by a
% along x, and rotation by alpha around x).