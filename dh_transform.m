function T = dh_transform(d, theta, a, alpha)
%DH_TRANSFORM  Homogeneous transform from Denavit-Hartenberg parameters.
%
%   T = dh_transform(d, theta, a, alpha) returns the 4x4 homogeneous
%   transformation matrix that describes the pose of frame i relative to
%   frame i-1, using the standard (distal) Denavit-Hartenberg convention.
%
%   Inputs:
%       d     - link offset along the previous z-axis   [mm]
%       theta - joint angle about the previous z-axis    [rad]
%       a     - link length along the (rotated) x-axis   [mm]
%       alpha - link twist about the (rotated) x-axis     [rad]
%
%   Output:
%       T     - 4x4 homogeneous transformation matrix

    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
