function nullspace = inverse_kinematics_nullspace(A1, A2, A3, A4, A5, A6, A7)
%INVERSE_KINEMATICS_NULLSPACE  Inverse kinematics for a prescribed redundancy angle.
%
%   nullspace = inverse_kinematics_nullspace(A1, ..., A7) re-solves the 7-DOF arm for
%   the SAME TCP pose but with the redundancy axis (axis 3, "E1") set to a
%   chosen value A3. Sweeping A3 traces the self-motion (null space) of the
%   redundant manipulator, i.e. the family of joint configurations that all
%   reach the identical TCP pose.
%
%   Inputs:
%       A1 ... A7 - joint angles [rad]. A3 is the redundancy angle to solve
%                   for; the reference TCP pose is taken from the remaining
%                   joints (with axis 3 = 0 during pose evaluation).
%
%   Output:
%       nullspace - M x 7 matrix of valid joint solutions [deg].
%
%   The structure mirrors inverse_kinematics: axes 1, 2 from the wrist
%   position (now also depending on A3 and A4), and axes 5, 6, 7 from the
%   wrist orientation. Solutions are filtered by apply_axis_limits.

    % Reference TCP pose (axis 3 evaluated at 0 for the pose itself)
    TCP_Pose = forward_kinematics(A1, A2, 0, A4, A5, A6, A7);

    % Constant transform from TCP back to the wrist point along z by d7
    T_TCP_S = eye(4);
    T_TCP_S(3, 4) = 78;

    % Wrist point pose in the base frame
    T_S_0 = TCP_Pose * inv(T_TCP_S);

    % Wrist point coordinates
    X_S = T_S_0(1, 4);
    Y_S = T_S_0(2, 4);
    Z_S = T_S_0(3, 4);

    d1 = 310.5;
    d3 = 400;
    d5 = 390;

    %================= Axis 2 =================
    a_z = d3 + d5 * cos(A4);
    b_z = d5 * sin(A4) * cos(A3);
    c_z = Z_S - d1;

    % If the discriminant is negative, the requested redundancy angle A3 (E1)
    % leaves the TCP pose unreachable: there is no real solution. Return empty
    % instead of letting a complex value propagate into atan2.
    disc_z = b_z^2 - c_z^2 + a_z^2;
    if disc_z < 0
        nullspace = [];
        return
    end

    t_2_1 = b_z/(a_z + c_z) + sqrt(disc_z)/(a_z + c_z);
    t_2_2 = b_z/(a_z + c_z) - sqrt(disc_z)/(a_z + c_z);

    delta2_1 = atan2(2*t_2_1/(1 + t_2_1^2), (1 - t_2_1^2)/(1 + t_2_1^2));
    delta2_2 = atan2(2*t_2_2/(1 + t_2_2^2), (1 - t_2_2^2)/(1 + t_2_2^2));

    %================= Axis 1 =================
    a_y   = -d5 * sin(A4) * sin(A3);
    b_y_1 = sin(delta2_1)*d3 + d5*sin(delta2_1)*cos(A4) - d5*sin(A4)*cos(A3)*cos(delta2_1);
    b_y_2 = sin(delta2_2)*d3 + d5*sin(delta2_2)*cos(A4) - d5*sin(A4)*cos(A3)*cos(delta2_2);
    c_y   = Y_S;

    a_x_1 = b_y_1;
    a_x_2 = b_y_2;
    b_x   = -a_y;
    c_x   = X_S;

    s_1_1 = (a_y*c_x - a_x_1*c_y) / (a_y*b_x - a_x_1*b_y_1);
    s_1_2 = (a_y*c_x - a_x_2*c_y) / (a_y*b_x - a_x_2*b_y_2);
    c_1_1 = (b_y_1*c_x - b_x*c_y) / (b_y_1*a_x_1 - b_x*a_y);
    c_1_2 = (b_y_2*c_x - b_x*c_y) / (b_y_2*a_x_2 - b_x*a_y);

    delta1_1 = atan2(s_1_1, c_1_1) - pi;
    delta1_2 = atan2(s_1_2, c_1_2) - pi;

    %================= Candidate set for the first 4 axes =================
    output(1, 1) = delta1_1;
    output(1, 2) = delta2_1 + pi/2;
    output(2, 1) = delta1_1;
    output(2, 2) = delta2_1 + pi/2;

    output(3, 1) = delta1_2;
    output(3, 2) = delta2_2 + pi/2;
    output(4, 1) = delta1_2;
    output(4, 2) = delta2_2 + pi/2;

    output(1, 3) = A3;
    output(2, 3) = A3;
    output(3, 3) = A3;
    output(4, 3) = A3;
    output(1, 4) = A4;
    output(2, 4) = A4;
    output(3, 4) = A4;
    output(4, 4) = A4;

    %================= Axes 5, 6 and 7 =================
    % Solve the spherical wrist. Angles arrive in radians here; the overall
    % matrix is converted to degrees only after this loop.
    function out = last_3_axis(delta_1, delta_2, delta_3, delta_4)

        delta_1_rad = delta_1;
        delta_2_rad = delta_2 - pi/2;
        delta_3_rad = delta_3;       % redundancy axis E1
        delta_4_rad = delta_4;

        T1 = dh_transform(d1, delta_1_rad, 0,  pi/2);
        T2 = dh_transform(0,  delta_2_rad, 0, -pi/2);
        T3 = dh_transform(d3, delta_3_rad, 0, -pi/2);
        T4 = dh_transform(0,  delta_4_rad, 0,  pi/2);

        T_4_0 = T1 * T2 * T3 * T4;
        D_4_0 = T_4_0(1:3, 1:3);
        D_7_0 = TCP_Pose(1:3, 1:3);

        D_7_4 = D_4_0' * D_7_0;

        % Variant 1 (wrist flip 0 ... 180 deg)
        delta_5_1 = atan2(D_7_4(2, 3), D_7_4(1, 3));
        delta_6_1 = -acos(D_7_4(3, 3));
        delta_7_1 = atan2(D_7_4(3, 2), -D_7_4(3, 1));

        % Variant 2 (wrist flip 180 ... 360 deg)
        delta_5_2 = delta_5_1 - pi;
        delta_6_2 = -delta_6_1;
        delta_7_2 = delta_7_1 + pi;

        out(1, 1) = delta_5_1;
        out(2, 1) = delta_6_1;
        out(3, 1) = delta_7_1;
        out(1, 2) = delta_5_2;
        out(2, 2) = delta_6_2;
        out(3, 2) = delta_7_2;
    end

    % Each call covers two variants, so the loop steps by 2.
    for i = 1:2:4
        Erg = last_3_axis(output(i, 1), output(i, 2), output(i, 3), output(i, 4));
        output(i, 5) = Erg(1, 1);
        output(i, 6) = Erg(2, 1);
        output(i, 7) = Erg(3, 1);

        output(i+1, 5) = Erg(1, 2);
        output(i+1, 6) = Erg(2, 2);
        output(i+1, 7) = Erg(3, 2);
    end

    output = rad2deg(output);

    %================= Filter against joint limits =================
    candidates = [];
    for i = 1:4
        cleanedRow = apply_axis_limits(output(i, :));
        if isnumeric(cleanedRow(1, 1))
            candidates = [candidates; cleanedRow];
        end
    end

    %================= Verify against the forward kinematics =========
    % Remove extraneous roots (from the half-angle squaring) that pass the
    % joint limits but do not reproduce the target pose. Axis 2 is the
    % geometric angle, so the DH angle is (A2 - 90 deg); E1 (axis 3) is used
    % directly.
    POS_TOL = 1e-3;     % position tolerance [mm]
    ORI_TOL = 1e-4;     % orientation tolerance [rad]
    valid = [];
    for i = 1:size(candidates, 1)
        row = candidates(i, :);
        if any(abs(imag(row)) > 1e-9)
            continue
        end
        row = real(row);

        qc    = deg2rad(row);
        qc(2) = qc(2) - pi/2;
        Pc = forward_kinematics(qc(1), qc(2), qc(3), qc(4), qc(5), qc(6), qc(7));

        dPos = norm(Pc(1:3, 4) - TCP_Pose(1:3, 4));
        Re   = TCP_Pose(1:3, 1:3)' * Pc(1:3, 1:3);
        dOri = acos(max(-1, min(1, (trace(Re) - 1) / 2)));

        if dPos < POS_TOL && dOri < ORI_TOL
            valid = [valid; row];
        end
    end
    nullspace = valid;

end
