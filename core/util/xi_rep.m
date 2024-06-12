function q_xi = xi_rep(q)
arguments (Input)
    q (4,1) double
end
arguments (Output)
    q_xi (4,3) double
end
    q_xi = [
        -q(2:4)';
        q(1)*eye(3)+cross_rep(q(2:4))
    ];  % Xi representation of q (scalar-first)
end

