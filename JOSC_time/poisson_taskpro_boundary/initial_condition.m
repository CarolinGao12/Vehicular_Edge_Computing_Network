function [V_postion_x_right_initial,V_postion_y_right_initial,V_postion_x_left_initial,V_postion_y_left_initial] = initial_condition(Z,L_m,L_rd,N_initial_right,N_initial_left)

%   This function gives the initial prime condition

V_postion_x_right_initial=Z*L_m + L_rd.*rand(N_initial_right,1);
V_postion_y_right_initial =rand(N_initial_right,1);
for ii=1:N_initial_right
    if V_postion_y_right_initial(ii) >=0.5
        V_postion_y_right_initial(ii)=8;
    else
        V_postion_y_right_initial(ii)=6;
    end
end


V_postion_x_left_initial=Z*L_m +L_rd.*rand(N_initial_left,1);
V_postion_y_left_initial =rand(N_initial_left,1);
for ii=1:N_initial_left
    if V_postion_y_left_initial(ii) >=0.5
        V_postion_y_left_initial(ii)=4;
    else
        V_postion_y_left_initial(ii)=2;
    end
end


end

