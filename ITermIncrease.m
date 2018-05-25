function [ newITerm ] = ITermIncrease( iTerm, error, cap )
    if(error < 0 && iTerm > 0) 
        iTerm = max(0.0, iTerm + 2.5 * error);
    elseif(error > 0 && iTerm < 0) 
        iTerm = min(0.0, iTerm + 2.5 * error);
    else
        iTerm = iTerm + error;
    end
    
    if(iTerm > cap) iTerm = cap; end
    if(iTerm < -cap) iTerm = -cap; end
    
    newITerm = iTerm;
end