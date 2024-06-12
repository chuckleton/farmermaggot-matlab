function v_cross = cross_rep(v)
arguments (Input)
    v (3,1) double
end
arguments (Output)
    v_cross (3,3) double
end
v_cross = [ 
        0      -v(3)    v(2);
        v(3)    0      -v(1);
        -v(2)    v(1)    0
];  % Cross-product representation of v
end

