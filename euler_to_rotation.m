function R = euler_to_rotation(phi, theta, psi, convention)
%EULER_TO_ROTATION  Rotation matrix from a set of Euler/Tait-Bryan angles.
%
%   R = euler_to_rotation(phi, theta, psi, convention) builds the 3x3 rotation
%   matrix for the given angles (in radians) and rotation convention.
%
%   Inputs:
%       phi, theta, psi - the three angles [rad]
%       convention      - 'ZYZ' (proper Euler angles) or
%                         'RPY' (roll-pitch-yaw, X-Y-Z fixed axes)
%
%   Output:
%       R - 3x3 rotation matrix

    switch convention

        case 'ZYZ'
            R = [cos(phi)*cos(theta)*cos(psi)-sin(phi)*sin(psi), -cos(phi)*cos(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*sin(theta);
                 sin(phi)*cos(theta)*cos(psi)+cos(phi)*sin(psi), -sin(phi)*cos(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*sin(theta);
                 -sin(theta)*cos(psi),                            sin(theta)*sin(psi),                            cos(theta)];

        case 'RPY'
            R = [cos(phi)*cos(theta), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                 sin(phi)*cos(theta), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
                 -sin(theta),         cos(theta)*sin(psi),                            cos(theta)*cos(psi)];

        otherwise
            error('euler_to_rotation:invalidConvention', ...
                  'Unknown convention "%s". Use ''ZYZ'' or ''RPY''.', convention);
    end

end
