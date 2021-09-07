function QMDP = scheduleQMDP(xH, uH, QMDP_left, QMDP_right)
% function QMDP = scheduleQMDP(xH, uH, QMDP_left, QMDP_right)
%     Determine which QMDP policy to use based on human's current state
%     and action
    vH = xH(3);
    vLatH = uH(2);
    if vLatH <= 0
        QMDP = QMDP_left;
    elseif vLatH > 0
        QMDP = QMDP_right;
    else
        QMDP = QMDP_left;
    end
end