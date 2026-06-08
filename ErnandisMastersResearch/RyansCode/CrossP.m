function mat = CrossP(vec)
if length(vec) == 3
    mat = [0 -vec(3) vec(2); 
           vec(3) 0 -vec(1); 
          -vec(2) vec(1) 0];
    
elseif length(vec) > 3
    mat = [0, -vec(1), vec(2), vec(4);
            vec(3), 0, -vec(2), vec(5);
            -vec(2), vec(1), 0, vec(6);
            0, 0, 0, 0];
end
end
