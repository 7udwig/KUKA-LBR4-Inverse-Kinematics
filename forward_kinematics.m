function tcpPose = forward_kinematics(q1, q2, q3, q4, q5, q6, q7)
%FORWARD_KINEMATICS  Forward kinematics of the KUKA LBR 4+ (7 DOF).
%
%   tcpPose = forward_kinematics(q1, ..., q7) returns the 4x4 homogeneous
%   transformation of the tool centre point (TCP) relative to the robot
%   base, given the seven joint angles.
%
%   Inputs:
%       q1 ... q7 - joint angles [rad]
%
%   Output:
%       tcpPose   - 4x4 homogeneous transformation (base -> TCP)
%
%   The robot is modelled with the standard DH parameters listed below.
%   Joint q3 corresponds to the redundancy ("extra") axis E1.

    % Denavit-Hartenberg parameters
    %          1        2       3      4      5       6     7
    d     = [310.5;    0;    400;    0;    390;    0;    78];   % link offset [mm]
    a     = [0;        0;      0;    0;      0;    0;     0];   % link length [mm]
    alpha = [pi/2;  -pi/2;  -pi/2; pi/2;  pi/2; -pi/2;    0];   % link twist  [rad]

    % Per-joint transforms (joint angles are passed in radians)
    T1 = dh_transform(d(1), q1, a(1), alpha(1));
    T2 = dh_transform(d(2), q2, a(2), alpha(2));
    T3 = dh_transform(d(3), q3, a(3), alpha(3));
    T4 = dh_transform(d(4), q4, a(4), alpha(4));
    T5 = dh_transform(d(5), q5, a(5), alpha(5));
    T6 = dh_transform(d(6), q6, a(6), alpha(6));
    T7 = dh_transform(d(7), q7, a(7), alpha(7));

    % Compose to obtain the base -> TCP transform
    tcpPose = T1 * T2 * T3 * T4 * T5 * T6 * T7;

end
