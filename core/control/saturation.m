function s_bar = saturation(s,epsilon)
%SATURATION
arguments (Input)
    s (:,1) double
    epsilon double
end
arguments (Output)
    s_bar (:,1) double
end
    s_bar = zeros(size(s));
    for i=1:length(s)
        if s(i) < -epsilon
            s_bar(i) = -1;
        elseif s(i) > epsilon
            s_bar(i) = 1;
        else
            s_bar(i) = s(i) / epsilon;
        end
    end
end

