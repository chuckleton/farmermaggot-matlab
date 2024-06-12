function new_state = rk4(t,dt,state)
% RK4
    state_vec = state.SpacecraftState.full_state_vec();
    f1=dt*state_derivative(t,state_vec,state);
    f2=dt*state_derivative(t,state_vec+0.5*f1,state);
    f3=dt*state_derivative(t,state_vec+0.5*f2,state);
    f4=dt*state_derivative(t,state_vec+f3,state);
    new_state=state_vec+1/6*(f1+2*f2+2*f3+f4);
end

