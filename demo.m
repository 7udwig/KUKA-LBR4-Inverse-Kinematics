% demo.m
% ---------------------------------------------------------------------------
% Example usage of the KUKA LBR 4+ inverse kinematics package.
%
% Run this file from the repository root in MATLAB:
%   >> demo
% ---------------------------------------------------------------------------

clc; clear;

% Make the function library available on the path
addpath(fileparts(mfilename('fullpath')));

%% 1) Forward kinematics ----------------------------------------------------
% Joint configuration in degrees. NOTE: KUKA reports axis 2 with a -90 deg
% home offset relative to the DH angle, hence the (... - 90) term below.
thetaDeg = [-74.77; 154.09 - 90; 0; 92.93; 2.61; -38.49; 1.39];
q = deg2rad(thetaDeg);

tcpPose = forward_kinematics(q(1), q(2), q(3), q(4), q(5), q(6), q(7));
rpyDeg  = rotation_to_euler(tcpPose(1:3, 1:3), 'RPY');

fprintf('Forward kinematics\n');
fprintf('  TCP position [mm] : X=%.2f  Y=%.2f  Z=%.2f\n', ...
        tcpPose(1, 4), tcpPose(2, 4), tcpPose(3, 4));
fprintf('  TCP orientation (RPY) [deg]: A=%.2f  B=%.2f  C=%.2f\n\n', ...
        rpyDeg(1), rpyDeg(2), rpyDeg(3));

%% 2) Inverse kinematics ----------------------------------------------------
% Solve for a Cartesian target pose: position [mm] and RPY orientation [deg].
X = -24.11;  Y = 96.88;  Z = 857.01;
A = -65.90;  B = 67.14;  C = 6.55;

solutions = inverse_kinematics(X, Y, Z, A, B, C);

fprintf('Inverse kinematics\n');
fprintf('  Target: X=%.2f Y=%.2f Z=%.2f | A=%.2f B=%.2f C=%.2f\n', X, Y, Z, A, B, C);
fprintf('  Found %d valid solution(s) [deg]:\n', size(solutions, 1));
disp(array2table(solutions, 'VariableNames', ...
     {'A1','A2','A3_E1','A4','A5','A6','A7'}));

%% 3) Null space / redundancy axis -----------------------------------------
% Re-solve the same pose with the redundancy axis (E1) set to 20 deg.
% Sweeping this angle traces the arm's self-motion.
nullspace = inverse_kinematics_nullspace(q(1), q(2), deg2rad(20), q(4), q(5), q(6), q(7));

fprintf('Null-space solutions for E1 = 20 deg\n');
fprintf('  Found %d valid solution(s) [deg]:\n', size(nullspace, 1));
disp(array2table(nullspace, 'VariableNames', ...
     {'A1','A2','A3_E1','A4','A5','A6','A7'}));
