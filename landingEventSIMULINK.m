function [value, isterminal, direction] = landingEventSIMULINK(~, u)
% landingEvent - ODE solver event to stop integration when pitch is -0.1 rad

    value = atan2( u(4) , u(2) ) + 0.1; % pitch plus 0.1 rad offset
    isterminal = 1; % stop the integration when value = 0
    direction = -1; % only detect value = 0 when altitude is decreasing

end
