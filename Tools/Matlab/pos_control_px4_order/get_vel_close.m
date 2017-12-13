function [vel_close] = get_vel_close(get_cruising_speed_xy,unit_prev_to_current, unit_current_to_next)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
min_cruise_speed = 1.0;
if ((get_cruising_speed_xy - min_cruise_speed) < 0.001)
    vel_close = get_cruising_speed_xy;
else
    % middle cruise speed is a number between maximum cruising speed and minimum cruising speed and corresponds to speed at angle of 90degrees
    % it needs to be always larger than minimum cruise speed */
    %float
    %par_pos.cruise_speed_90
    middle_cruise_speed = 3;

    if ((middle_cruise_speed - min_cruise_speed) < 0.001)
        middle_cruise_speed = min_cruise_speed + 0.001;
    end

    if ((get_cruising_speed_xy - middle_cruise_speed) < 0.001)
        middle_cruise_speed = (get_cruising_speed_xy + min_cruise_speed) * 0.5;
    end

    %* if middle cruise speed is exactly in the middle, then compute
    %* vel_close linearly
    %bool
    use_linear_approach = 0;

    if (((get_cruising_speed_xy + min_cruise_speed) * 0.5) - middle_cruise_speed < 0.001)
        use_linear_approach = 1;
    end

    % angle = cos(x) + 1.0
    % angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0 */
    %float
    angle = 2.0;

    if (normest(unit_current_to_next) > 0.001)
        angle = dot(unit_current_to_next , (unit_prev_to_current * -1.0)) + 1.0;
    end

    % compute velocity target close to waypoint */


    if (use_linear_approach)

        % velocity close to target adjusted to angle
        % vel_close =  m*x+q

        %float
        slope = -(get_cruising_speed_xy - min_cruise_speed) / 2.0;
        vel_close = slope * angle + get_cruising_speed_xy;

    else

        %* velocity close to target adjusted to angle
        %* vel_close = a *b ^x + c; where at angle = 0 -> vel_close = vel_cruise; angle = 1 -> vel_close = middle_cruise_speed (this means that at 90degrees
        %* the velocity at target is middle_cruise_speed);
        %* angle = 2 -> vel_close = min_cruising_speed */

        %* from maximum cruise speed, minimum cruise speed and middle cruise speed compute constants a, b and c */
        %float
        a = -((middle_cruise_speed -  get_cruising_speed_xy) * (middle_cruise_speed -  get_cruising_speed_xy)) / (2.0 * middle_cruise_speed - get_cruising_speed_xy - min_cruise_speed);
        %float
        c =  get_cruising_speed_xy - a;
        %float
        b = (middle_cruise_speed - c) / a;
        vel_close = a * b^angle + c;
    end

    % vel_close needs to be in between max and min */
    vel_close =  constrain(vel_close, min_cruise_speed, get_cruising_speed_xy);
end

end