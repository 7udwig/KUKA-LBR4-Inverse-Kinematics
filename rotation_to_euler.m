function eulerAngles = rotation_to_euler(R, convention)
%ROTATION_TO_EULER  Euler/Tait-Bryan angles from a rotation matrix.
%
%   eulerAngles = rotation_to_euler(R, convention) extracts the three angles
%   (in degrees) from the 3x3 rotation matrix R for the given convention.
%
%   Inputs:
%       R          - 3x3 rotation matrix
%       convention - 'ZYZ' (proper Euler angles) or
%                    'RPY' (roll-pitch-yaw, X-Y-Z fixed axes)
%
%   Output:
%       eulerAngles - [phi, theta, psi] in degrees
%
%   NOTE: k selects one of the two possible solution branches. Only the
%   positive branch (k = 1) is returned here.

    k = 1;

    switch convention
        case 'ZYZ'
            theta = rad2deg(atan2(k*sqrt(1-R(3,3)), R(3,3)));
            phi   = rad2deg(atan2(k*R(2,3),  k*R(1,3)));
            psi   = rad2deg(atan2(k*R(3,2), -k*R(3,1)));

        case 'RPY'
            phi   = rad2deg(atan2(k*R(2,1), k*R(1,1)));
            theta = rad2deg(atan2(-R(3,1),  k*sqrt(1-R(3,1)^2)));
            psi   = rad2deg(atan2(k*R(3,2), k*R(3,3)));

        otherwise
            error('rotation_to_euler:invalidConvention', ...
                  'Unknown convention "%s". Use ''ZYZ'' or ''RPY''.', convention);
    end

    eulerAngles = [phi, theta, psi];
end
