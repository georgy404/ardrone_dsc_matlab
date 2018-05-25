function [ norm_angle ] = NormalizeAngle( angle )
    while (angle > pi)
        angle = angle - 2 * pi;
    end
    while (angle < -pi)
        angle = angle + 2 * pi;
    end
    
    norm_angle = angle;
end

