% validate.m
% ---------------------------------------------------------------------------
% Numerical validation of the KUKA LBR 4+ inverse kinematics.
%
% Two independent checks are run:
%
%   1) Round-trip FK -> IK -> FK over N random configurations.
%      For every random joint vector q, the TCP pose P = FK(q) is computed,
%      the inverse kinematics is solved for P, and FK is evaluated again for
%      EVERY returned solution. Each solution must reproduce P, so the
%      residual ||FK(IK(FK(q))) - FK(q)|| must fall to machine precision.
%
%   2) Null-space consistency. For one configuration the redundancy axis E1
%      is swept; every null-space solution must reproduce the SAME TCP pose.
%
% Convention note: the IK returns axis 2 as the geometric angle d21, which is
% the DH angle theta2 plus 90 deg. To feed a solution back into the forward
% kinematics, 90 deg is subtracted from axis 2 (exactly what the IK does
% internally when it rebuilds the wrist frame).
%
% Run from the repository root:
%   >> validate
% ---------------------------------------------------------------------------

clc; clear;
addpath(fileparts(mfilename('fullpath')));

rng(0);                 % reproducible random configurations
N        = 1000;        % number of random configurations
TOL_IMAG = 1e-6;        % treat a solution as complex above this imaginary part

% Joint sampling ranges in the DH convention [deg] (axis 3 = E1 fixed to 0).
% Axis 2 DH range is the geometric limit (-30..210) shifted by -90 deg.
lo = [-170, -120, 0, -120, -170, -120, -170];
hi = [ 170,  120, 0,  120,  170,  120,  170];

posErr = [];            % position error per solution [mm]
oriErr = [];            % orientation error per solution [deg]
nSolutions = zeros(N, 1);
nSkipped   = 0;

for k = 1:N
    % --- random source configuration (DH convention) ---
    qDeg    = lo + (hi - lo) .* rand(1, 7);
    qDeg(3) = 0;
    q       = deg2rad(qDeg);

    % --- forward kinematics: target pose ---
    P   = forward_kinematics(q(1), q(2), q(3), q(4), q(5), q(6), q(7));
    rpy = rot2euler_safe(P(1:3, 1:3));      % [A B C] in degrees, RPY

    % --- inverse kinematics for that pose ---
    sols = inverse_kinematics(P(1, 4), P(2, 4), P(3, 4), rpy(1), rpy(2), rpy(3));

    if isempty(sols)
        nSkipped = nSkipped + 1;
        continue
    end

    rowsThisPose = 0;
    for r = 1:size(sols, 1)
        s = sols(r, :);
        if any(abs(imag(s)) > TOL_IMAG)     % discard complex solutions
            continue
        end
        s = real(s);

        % --- reconstruct DH angles and run FK again ---
        qc    = deg2rad(s);
        qc(2) = qc(2) - pi/2;               % geometric -> DH angle for axis 2
        Pc = forward_kinematics(qc(1), qc(2), qc(3), qc(4), qc(5), qc(6), qc(7));

        % --- errors ---
        dp = norm(Pc(1:3, 4) - P(1:3, 4));                 % position [mm]
        Re = P(1:3, 1:3)' * Pc(1:3, 1:3);                  % relative rotation
        ang = real(acos(max(-1, min(1, (trace(Re) - 1) / 2))));
        do  = rad2deg(ang);                                % orientation [deg]

        posErr(end+1, 1) = dp; %#ok<AGROW>
        oriErr(end+1, 1) = do; %#ok<AGROW>
        rowsThisPose = rowsThisPose + 1;
    end
    nSolutions(k) = rowsThisPose;
end

% --- report: round trip ---
fprintf('========================================================\n');
fprintf(' Round-trip validation  (FK -> IK -> FK)\n');
fprintf('========================================================\n');
fprintf(' Random configurations sampled : %d\n', N);
fprintf(' Configurations with no real IK solution (skipped): %d\n', nSkipped);
fprintf(' Solutions checked in total     : %d\n', numel(posErr));
fprintf(' Mean solutions per pose        : %.2f\n\n', mean(nSolutions(nSolutions > 0)));

fprintf(' Position error [mm]   : max %.3e | mean %.3e | median %.3e\n', ...
        maxOr0(posErr), meanOr0(posErr), medianOr0(posErr));
fprintf(' Orientation error [deg]: max %.3e | mean %.3e | median %.3e\n\n', ...
        maxOr0(oriErr), meanOr0(oriErr), medianOr0(oriErr));

% --- null-space consistency ---
fprintf('========================================================\n');
fprintf(' Null-space consistency  (sweep E1, same TCP pose)\n');
fprintf('========================================================\n');

qRefDeg = [30, 40, 0, 60, 20, 50, 10];      % reference config (DH conv.)
qRef    = deg2rad(qRefDeg);
Pref    = forward_kinematics(qRef(1), qRef(2), 0, qRef(4), qRef(5), qRef(6), qRef(7));

e1Sweep   = -120:10:120;                    % redundancy angle E1 [deg]
nsPosErr  = [];
nsValid   = 0;
for e1 = e1Sweep
    ns = inverse_kinematics_nullspace(qRef(1), qRef(2), deg2rad(e1), ...
                                      qRef(4), qRef(5), qRef(6), qRef(7));
    if isempty(ns)
        continue
    end
    for r = 1:size(ns, 1)
        s = ns(r, :);
        if any(abs(imag(s)) > TOL_IMAG)
            continue
        end
        s = real(s);
        qc    = deg2rad(s);
        qc(2) = qc(2) - pi/2;
        qc(3) = deg2rad(s(3));              % E1 used directly (no offset)
        Pc = forward_kinematics(qc(1), qc(2), qc(3), qc(4), qc(5), qc(6), qc(7));
        nsPosErr(end+1, 1) = norm(Pc(1:3, 4) - Pref(1:3, 4)); %#ok<AGROW>
        nsValid = nsValid + 1;
    end
end

fprintf(' E1 values swept           : %d\n', numel(e1Sweep));
fprintf(' Valid null-space solutions: %d\n', nsValid);
fprintf(' Position error [mm]       : max %.3e | mean %.3e | median %.3e\n', ...
        maxOr0(nsPosErr), meanOr0(nsPosErr), medianOr0(nsPosErr));
fprintf('========================================================\n');


% ===================== helper functions =====================

function rpy = rot2euler_safe(R)
    % rotation_to_euler returns RPY in degrees; this wrapper just forwards it.
    rpy = rotation_to_euler(R, 'RPY');
end

function v = maxOr0(x)
    if isempty(x), v = 0; else, v = max(x); end
end

function v = meanOr0(x)
    if isempty(x), v = 0; else, v = mean(x); end
end

function v = medianOr0(x)
    if isempty(x), v = 0; else, v = median(x); end
end
