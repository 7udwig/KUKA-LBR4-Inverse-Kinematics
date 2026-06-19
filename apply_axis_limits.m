function output = apply_axis_limits(raw)
%APPLY_AXIS_LIMITS  Validate and normalise a joint solution against the limits.
%
%   output = apply_axis_limits(raw) takes a 1x7 row of joint angles (in
%   degrees) and:
%       * normalises angles that are outside [-180, 180] but still reach a
%         physically valid pose via a +/-360 deg wrap, and
%       * rejects the solution if any axis lies in its forbidden range.
%
%   KUKA LBR 4+ joint limits:
%       Axis 1 (A1)       : +/- 170 deg
%       Axis 2 (A2)       :  -30 ... +210 deg
%       Axis 3 (A3 = E1)  : +/- 170 deg
%       Axis 4 (A4)       : +/- 120 deg
%       Axis 5 (A5)       : +/- 170 deg
%       Axis 6 (A6)       : +/- 120 deg
%       Axis 7 (A7)       : +/- 170 deg
%
%   Return value:
%       * a valid 1x7 row of normalised angles, OR
%       * a sentinel that is NOT numeric when the solution is invalid
%         (logical false for axes 1/3-7, the string "false" for axis 2).
%
%   NOTE: callers distinguish valid from invalid results with isnumeric(),
%   which is why the invalid case returns a non-numeric sentinel rather
%   than an empty array (indexing an empty array would error).

    for i = 1:7

        w = raw(1, i);

        if (i ~= 2)
            % ----- Axes 1, 3, 4, 5, 6, 7 --------------------------------
            % Symmetric limit: 170 deg for the odd axes (A1, A3, A5, A7),
            % 120 deg for the even axes (A4, A6).
            if mod(i, 2)
                limit = 170;
            else
                limit = 120;
            end

            % Forbidden range -> reject the whole solution
            if (((-(360 - limit) < w) && (w < -limit)) || ((limit < w) && (w < 360 - limit)))
                output = false;
                output(1, :) = false;
                return
            end

            % Already inside the valid range -> keep as is
            if (((-limit <= w) && (w <= 0)) || ((0 <= w) && (w <= limit)))
                output(1, i) = w;
            end

            % Just below +360 deg -> wrap into the negative valid range
            if ((360 - limit < w) && (w < 360))
                output(1, i) = -(360 - w);
            end

            % Just above -360 deg -> wrap into the positive valid range
            if ((-360 < w) && (w < -(360 - limit)))
                output(1, i) = 360 + w;
            end

        else
            % ----- Axis 2 (asymmetric range -30 ... +210 deg) -----------
            % Forbidden range -> reject the whole solution
            if (((w < -30) && (w > -150)) || (w > 210) && (w < 330))
                output = [];
                output = "false";
                return
            end

            % Already inside the valid range -> keep as is
            if ((w >= -30) && (w <= 210))
                output(1, i) = w;
            end

            % Wrap from the lower side back into the valid range
            if ((w <= -150) && (w > -360))
                output(1, i) = w + 360;
            end

            % Wrap from the upper side back into the valid range
            if ((w > 330) && (w < 360))
                output(1, i) = w - 360;
            end
        end

    end

end
