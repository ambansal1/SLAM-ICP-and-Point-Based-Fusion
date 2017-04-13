function is_use = isUsableInputPoints(is_close, is_similar, is_first)

    %==== TODO: Output a boolean matrix which represents if each input point should be added into the fusion map based on the three input boolean matrices ====
    
    % Write your code here...
    
    is_use = (is_close & is_similar) | (is_first) ;
    
    

end