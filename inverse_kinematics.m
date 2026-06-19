function valid = inverse_kinematics(X, Y, Z, A, B, C)
%INVERSE_KINEMATICS  Analytic inverse kinematics of the KUKA LBR 4+ (7 DOF).
%
%   valid = inverse_kinematics(X, Y, Z, A, B, C) computes the joint angle
%   solutions that place the tool centre point (TCP) at the requested
%   Cartesian pose.
%
%   Inputs:
%       X, Y, Z - TCP position [mm]
%       A, B, C - TCP orientation as roll-pitch-yaw (RPY) angles [deg]
%
%   Output:
%       valid - N x 7 matrix; each row is one joint solution [deg],
%               [q1 q2 q3 q4 q5 q6 q7]. Only solutions inside the joint
%               limits are returned (see apply_axis_limits).
%
%   The redundant 7-DOF arm is resolved by fixing the redundancy axis
%   (axis 3, "E1") to 0 deg. The remaining six joints are solved
%   analytically: axes 1, 2, 4 from the wrist position, axes 5, 6, 7 from
%   the wrist orientation. Each branch yields two wrist orientation
%   variants, giving up to eight candidate solutions before limit filtering.

    % --- Robot / DH parameters --------------------------------------------
    d1 = 310.5;
    d2 = 0;
    d3 = 400;
    d4 = 0;
    d5 = 390;
    d6 = 0;
    d7 = 78;

    alpha_1 =  pi/2;
    alpha_2 = -pi/2;
    alpha_3 = -pi/2;
    alpha_4 =  pi/2;
    alpha_5 =  pi/2;
    alpha_6 = -pi/2;
    alpha_7 =  0;

    % --- Build the target TCP pose ----------------------------------------
    % Orientation angles are supplied in degrees -> convert to radians.
    A_rad = deg2rad(A);
    B_rad = deg2rad(B);
    C_rad = deg2rad(C);

    % 4x4 homogeneous transform of the requested TCP pose
    orientation = euler_to_rotation(A_rad, B_rad, C_rad, 'RPY');
    TCP_Pose = eye(4);
    TCP_Pose(1:3, 1:3) = orientation;
    TCP_Pose(1, 4) = X;
    TCP_Pose(2, 4) = Y;
    TCP_Pose(3, 4) = Z;

    % Constant transform from the TCP back to the wrist point along z by d7.
    T_TCP_S = eye(4);
    T_TCP_S(3, 4) = 78;

    % Wrist point pose expressed in the base frame
    T_S_0 = TCP_Pose * inv(T_TCP_S);

    % Wrist point coordinates
    X_S = T_S_0(1, 4);
    Y_S = T_S_0(2, 4);
    Z_S = T_S_0(3, 4);

    %================= Axis 1 =================
    delta_1_1 = atan2(Y_S, X_S);

    % Case distinction so that both shoulder solutions stay in range.
    if Y_S < 0
        delta_1_1 = mod(delta_1_1, pi) + pi;
        delta_1_2 = delta_1_1 + pi;
    else
        delta_1_1 = mod(delta_1_1, pi);
        delta_1_2 = delta_1_1 - pi;
    end

    % Store the two axis-1 solutions (rows 1-4 and 5-8)
    output(1, 1) = rad2deg(delta_1_1);
    output(2, 1) = rad2deg(delta_1_1);
    output(3, 1) = rad2deg(delta_1_1);
    output(4, 1) = rad2deg(delta_1_1);
    output(5, 1) = rad2deg(delta_1_2);
    output(6, 1) = rad2deg(delta_1_2);
    output(7, 1) = rad2deg(delta_1_2);
    output(8, 1) = rad2deg(delta_1_2);

    %================= Axis 2 =================
    % Planar two-link geometry in the arm plane.
    b   = 390;     % effective upper-arm length (l3)
    a   = 400;     % effective forearm length   (l2)
    l_1 = 310.5;   % base height (d1)

    % --- Branch for delta_1_1 (positive arm reach) ---
    B = T_S_0(3, 4) - l_1;
    A = sqrt(T_S_0(1, 4)^2 + T_S_0(2, 4)^2);   % radial distance (offset e = 0)

    C_alpha = (A^2 + B^2 - b^2 + a^2) / (2*a);

    % Half-angle (tangent) substitution to obtain both elbow configurations.
    t_a_1 = B/(A + C_alpha) + sqrt(B^2 - C_alpha^2 + A^2)/(A + C_alpha);
    t_a_2 = B/(A + C_alpha) - sqrt(B^2 - C_alpha^2 + A^2)/(A + C_alpha);

    delta_2_1 = atan2((2*t_a_1)/(1 + t_a_1^2), (1 - t_a_1^2)/(1 + t_a_1^2));
    delta_2_2 = atan2((2*t_a_2)/(1 + t_a_2^2), (1 - t_a_2^2)/(1 + t_a_2^2));

    delta_2_1_deg = rad2deg(delta_2_1);
    delta_2_2_deg = rad2deg(delta_2_2);

    output(1, 2) = delta_2_1_deg;
    output(2, 2) = delta_2_1_deg;
    output(3, 2) = delta_2_2_deg;
    output(4, 2) = delta_2_2_deg;

    % --- Branch for delta_1_2 (A taken with the opposite sign) ---
    b   = 390;
    a   = 400;
    l_1 = 310.5;

    B = T_S_0(3, 4) - l_1;
    A = -sqrt(T_S_0(1, 4)^2 + T_S_0(2, 4)^2);   % opposite radial sign

    C_alpha = (A^2 + B^2 - b^2 + a^2) / (2*a);

    if (A + C_alpha) == 0
        % Degenerate geometry: arm fully stretched along the axis.
        t_a_1 = 0;
        t_a_2 = 0;
        delta_2_1 = pi;
        delta_2_2 = pi;
    else
        t_a_1 = B/(A + C_alpha) + sqrt(B^2 - C_alpha^2 + A^2)/(A + C_alpha);
        t_a_2 = B/(A + C_alpha) - sqrt(B^2 - C_alpha^2 + A^2)/(A + C_alpha);

        delta_2_1 = atan2((2*t_a_1)/(1 + t_a_1^2), (1 - t_a_1^2)/(1 + t_a_1^2));
        delta_2_2 = atan2((2*t_a_2)/(1 + t_a_2^2), (1 - t_a_2^2)/(1 + t_a_2^2));
    end

    delta_2_3_deg = rad2deg(delta_2_1);
    delta_2_4_deg = rad2deg(delta_2_2);

    output(5, 2) = delta_2_3_deg;
    output(6, 2) = delta_2_3_deg;
    output(7, 2) = delta_2_4_deg;
    output(8, 2) = delta_2_4_deg;

    %================= Axis 3 (redundancy axis E1) =================
    % The redundant degree of freedom is fixed to 0 deg here.
    output(1:8, 3) = 0;

    %================= Axis 4 =================
    % NOTE: A, B, delta_2_1, delta_2_2 still hold the values from the
    % delta_1_2 branch computed above. This ordering is intentional and
    % must be preserved.
    beta_1 = atan2(B - a*sin(delta_2_1), A - a*cos(delta_2_1));
    beta_2 = atan2(B - a*sin(delta_2_2), A - a*cos(delta_2_2));

    delta_4_1 = delta_2_1 - beta_1;
    delta_4_2 = delta_2_2 - beta_2;

    delta_4_1_deg = rad2deg(delta_4_1);
    delta_4_2_deg = rad2deg(delta_4_2);

    if delta_4_1_deg == 360
        delta_4_1_deg = 0;
    end
    if delta_4_2_deg == 360
        delta_4_2_deg = 0;
    end

    output(1, 4) = delta_4_1_deg;
    output(2, 4) = delta_4_1_deg;
    output(3, 4) = delta_4_2_deg;
    output(4, 4) = delta_4_2_deg;
    output(5, 4) = delta_4_1_deg;
    output(6, 4) = delta_4_1_deg;
    output(7, 4) = delta_4_2_deg;
    output(8, 4) = delta_4_2_deg;

    %================= Axes 5, 6 and 7 =================
    % With axes 1, 2 and 4 known, evaluate the wrist orientation and solve
    % the spherical wrist. Each call returns the two orientation variants.
    function out = last_3_axis(delta_1, delta_2, delta_4)

        delta_1_rad = deg2rad(delta_1);
        delta_2_rad = deg2rad(delta_2) - pi/2;
        delta_3_rad = deg2rad(0);          % redundancy axis E1 fixed to 0 deg
        delta_4_rad = deg2rad(delta_4);

        T1 = dh_transform(d1, delta_1_rad, 0, alpha_1);
        T2 = dh_transform(d2, delta_2_rad, 0, alpha_2);
        T3 = dh_transform(d3, 0,           0, alpha_3);
        T4 = dh_transform(d4, delta_4_rad, 0, alpha_4);

        T_4_0 = T1 * T2 * T3 * T4;
        D_4_0 = T_4_0(1:3, 1:3);
        D_7_0 = TCP_Pose(1:3, 1:3);

        % Orientation of the wrist relative to frame 4
        D_7_4 = D_4_0' * D_7_0;

        % Variant 1 (wrist flip 0 ... 180 deg)
        delta_5_1 = atan2(D_7_4(2, 3), D_7_4(1, 3));
        delta_6_1 = -acos(D_7_4(3, 3));
        delta_7_1 = atan2(D_7_4(3, 2), -D_7_4(3, 1));

        % Variant 2 (wrist flip 180 ... 360 deg)
        delta_5_2 = delta_5_1 - pi;
        delta_6_2 = -delta_6_1;
        delta_7_2 = delta_7_1 + pi;

        % NOTE: the original code contains the chained comparison
        % "delta_1 == delta_2 == delta_4 == 0", which MATLAB evaluates
        % left-to-right rather than as a logical AND. The behaviour is kept
        % unchanged here to reproduce the original results.
        if delta_1 == delta_2 == delta_4 == 0 && delta_5_1 == -delta_7_1
            delta_5_1 = 0;
            delta_5_2 = 0;
            delta_7_1 = 0;
            delta_7_2 = 0;
        end

        out(1, 1) = rad2deg(delta_5_1);
        out(2, 1) = rad2deg(delta_6_1);
        out(3, 1) = rad2deg(delta_7_1);
        out(1, 2) = rad2deg(delta_5_2);
        out(2, 2) = rad2deg(delta_6_2);
        out(3, 2) = rad2deg(delta_7_2);
    end

    % Fill in axes 5-7 for every candidate. Each call covers two variants,
    % so the loop steps by 2.
    for i = 1:2:8
        Erg = last_3_axis(output(i, 1), output(i, 2), output(i, 4));
        output(i, 5) = Erg(1, 1);
        output(i, 6) = Erg(2, 1);
        output(i, 7) = Erg(3, 1);

        output(i+1, 5) = Erg(1, 2);
        output(i+1, 6) = Erg(2, 2);
        output(i+1, 7) = Erg(3, 2);
    end

    %================= Filter against joint limits =================
    candidates = [];
    for i = 1:8
        cleanedRow = apply_axis_limits(output(i, :));
        if isnumeric(cleanedRow(1, 1))
            [~, nCols] = size(cleanedRow);
            if nCols == 7
                candidates = [candidates; cleanedRow];
            end
        end
    end

    %================= Verify against the forward kinematics =========
    % The half-angle substitution (squaring) can introduce extraneous roots
    % that satisfy the joint limits but do not reproduce the requested pose.
    % Each candidate is checked with the forward kinematics and only kept if
    % its TCP pose matches the target to numerical precision. Axis 2 is the
    % geometric angle, so the DH angle is (A2 - 90 deg).
    POS_TOL = 1e-3;     % position tolerance [mm]
    ORI_TOL = 1e-4;     % orientation tolerance [rad]
    valid = [];
    for i = 1:size(candidates, 1)
        row = candidates(i, :);
        if any(abs(imag(row)) > 1e-9)
            continue                       % skip complex (near-singular) roots
        end
        row = real(row);

        qc    = deg2rad(row);
        qc(2) = qc(2) - pi/2;              % geometric -> DH angle for axis 2
        Pc = forward_kinematics(qc(1), qc(2), qc(3), qc(4), qc(5), qc(6), qc(7));

        dPos = norm(Pc(1:3, 4) - TCP_Pose(1:3, 4));
        Re   = TCP_Pose(1:3, 1:3)' * Pc(1:3, 1:3);
        dOri = acos(max(-1, min(1, (trace(Re) - 1) / 2)));

        if dPos < POS_TOL && dOri < ORI_TOL
            valid = [valid; row];
        end
    end

end
