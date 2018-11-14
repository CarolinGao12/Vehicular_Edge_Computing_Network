clc;clear ;
Z=4;
M=7*Z;                          % The number of server
L_m=20;                         % The segment length
L_rd=5*Z*L_m;                   % The road length
L_rd_total=7*Z*L_m;             % The total length(m)  
v_speed=120*(1000/3600);        % speed of cars (m/s)
B_RSU=1.25*1000 ;               % Bandwith of the each RSU (HZ)
P_v=100*(10^(-3));              % the transmission power of each vehicle(watt)
N_v=10^(-13);                   % the noise power(watt) 

c_car = 0.4;
F_limit=15*ones(1,M);
buffer_number=4;

%initial condition 
%initial postion for the car driving to right
N_initial_right=randi([10,25]);
V_postion_x_right_initial=Z*L_m + 5*Z*L_m.*rand(N_initial_right,1);
V_postion_y_right_initial =rand(N_initial_right,1);
for ii=1:N_initial_right
    if V_postion_y_right_initial(ii) >=0.5
        V_postion_y_right_initial(ii)=8;
    else
        V_postion_y_right_initial(ii)=6;
    end
end
% initial position for the car driving to left 
N_initial_left=randi([10,25]);
V_postion_x_left_initial=Z*L_m +5*Z*L_m.*rand(N_initial_left,1);
V_postion_y_left_initial =rand(N_initial_left,1);
for ii=1:N_initial_left
    if V_postion_y_left_initial(ii) >=0.5
        V_postion_y_left_initial(ii)=4;
    else
        V_postion_y_left_initial(ii)=2;
    end
end
vehicle_position_initial=[V_postion_x_right_initial, V_postion_y_right_initial ;V_postion_x_left_initial,V_postion_y_left_initial];

% the new car came in with distribution of poisson 
recy_time=6;
%the car add to the direction right
lambda_right=10;
N_right_add=poissrnd(lambda_right);
V_positon_x_right_add= Z*L_m*ones(N_right_add,1);
V_positon_y_right_add= rand(N_right_add,1);

for ii=1: N_right_add
    if V_positon_y_right_add(ii) >=0.5
        V_positon_y_right_add(ii)=8;
    else
        V_positon_y_right_add(ii)=6;
    end
end


%the car added to the direction left
lambda_left=10;
N_left_add=poissrnd(lambda_left);
V_postion_x_left_add=6*Z*L_m*ones(N_left_add,1);
V_postion_y_left_add =rand(N_left_add,1);
for ii=1:N_left_add
    if V_postion_y_left_add(ii) >=0.5
        V_postion_y_left_add(ii)=4;
    else
        V_postion_y_left_add(ii)=2;
    end
end

% the total car position
V_postion_x_right_initial=V_postion_x_right_initial+v_speed* recy_time;
V_postion_x_left_initial =V_postion_x_left_initial -v_speed* recy_time;
%update
vehicle_position_initial=[V_postion_x_right_initial, V_postion_y_right_initial ;V_postion_x_left_initial,V_postion_y_left_initial];
vehicle_position=[V_positon_x_right_add,V_positon_y_right_add;vehicle_position_initial;V_postion_x_left_add,V_postion_y_left_add];

%take out the car not in the range 
for ii= 1: length(vehicle_position)
    if (vehicle_position(ii,1)>= Z*L_m) && (vehicle_position(ii,1)<= 6*Z*L_m)
        vehicle_position(ii,1)=vehicle_position(ii,1);
    else
        vehicle_position(ii,1)=0;
    end
end


vehicle_position_delete=vehicle_position;
delete_index=find(vehicle_position_delete(:,1)==0);
vehicle_position_delete(delete_index,:)=[];



% get the number of car right and left
N_right=0; 
for ii=1: length(vehicle_position_delete)
    if (vehicle_position_delete(ii,2) == 6 ) |  (vehicle_position_delete(ii,2)==8 )
        
        N_right =N_right+1;
    end
end
vehicle_position_delete_right = vehicle_position_delete(1:N_right,:);

N_left=0;
for ii=1: length(vehicle_position_delete)
    if (vehicle_position_delete(ii,2) == 2 ) |  (vehicle_position_delete(ii,2)==4 )
        N_left =N_left+1;
    end
end
vehicle_position_delete_left = vehicle_position_delete(N_right+1:end,:);

% Driving Time 
L_all_right=zeros(N_right,M);
for ii=1:N_right
    for jj=1:M
        L_all_right(ii,jj)= -vehicle_position_delete_right(ii)+L_m*(jj-1);
%         if L_all_right(ii,jj)<0
%             L_all_right(ii,jj)=0;
%         end
        
    end
end

L_all_left=zeros(N_left,M);
for ii=1:N_left
    for jj=1:M
        L_all_left(ii,jj)=vehicle_position_delete_left(ii)-L_m*jj;
%         if  L_all_left(ii,jj)<0
%             L_all_left(ii,jj)=0;
%         end
        
    end
end
L_all=[L_all_right ; L_all_left];    

T_d_all = (zeros(length(vehicle_position_delete),M+1));
for ii = 1:length(vehicle_position_delete)
    T_d_all(ii,2:end) = L_all(ii,:) ./ v_speed;
end 

%communication time 
h_length= vehicle_position_delete(:,2);
w_length=10*ones(length(vehicle_position_delete),1);
d_t=zeros(length(vehicle_position_delete),1);
r_c=zeros(length(vehicle_position_delete),1);
dataSize = (100+200*rand(length(vehicle_position_delete),1));

d_t=sqrt(w_length.^2+h_length.^2);
r_c= B_RSU* log2(1+(P_v *  (d_t.^(-2.5))  )/N_v);
T_c= dataSize./r_c;

T_c_all = zeros(length(vehicle_position_delete),M+1);
for ii = 2:M+1
    T_c_all(:,ii) = T_c;
end

% The computing time 
c_i=0.5+rand(length(vehicle_position_delete),1);
c_i_all = ones(length(vehicle_position_delete),M+1);
for ii = 1:M+1
    c_i_all(:,ii) = c_i;
end  
T_max = (8+2*rand(length(vehicle_position_delete),1));


cvx_begin
    cvx_solver MOSEK
    variable f_ij_all(length(vehicle_position_delete),M);
    variable j_selection(length(vehicle_position_delete),M+1) binary;
%   variable j_selection(N,M+1) ;
    expression TT;

    
    TT=(j_selection.*(T_d_all+T_c_all));
    TT(:,1)=TT(:,1)+j_selection(:,1).*(c_i_all(:,1) / c_car);

    for ii=1:length(vehicle_position_delete)
        for jj=1:M
           TT(ii,jj+1)=TT(ii,jj+1)+ c_i_all(ii,jj+1)*quad_over_lin(j_selection(ii,jj+1),f_ij_all(ii,jj));
        end
    end
    
    minimize sum(sum(TT));

subject to 
    
    for ii = 1:length(vehicle_position_delete)
        sum(j_selection(ii,:))==1;          %condition (8c)
    end
    
    sum(f_ij_all) <= F_limit;                 %condition (8d)
    
    for ii=1:length(vehicle_position_delete)
        for jj=1:M
            f_ij_all(ii,jj)<=F_limit(jj)*j_selection(ii,jj+1);         %condition (8e)
            f_ij_all(ii,jj)>=0;
        end
    end

      sum(TT,2)<= T_max;                    %condition (9b)
    
      sum(j_selection(:,2:end))<=buffer_number;        % each server can not connect more than 4 cars(buffer)
      
      for ii= 1:length(vehicle_position_delete)
          for jj= 1:M
              if L_all(ii,jj)<0
                  j_selection(ii,jj+1)==0;
              end
          end
      end
      
      for ii= 1:length(vehicle_position_delete)
          for jj= 1:M
              if L_all(ii,jj)> 4*L_m
                  j_selection(ii,jj+1)==0;
              end
          end
      end
    
cvx_end

j_selection_matrix = int64(full(j_selection));
f_ij_all_matrix=full(f_ij_all);

for ii= 1:length(vehicle_position_delete)
    for jj =1:M
        if f_ij_all_matrix (ii,jj)< 0.001
            f_ij_all_matrix(ii,jj) = 0 ;
        end
    end
end  

result_opti=cvx_optval;
TT_matrix=full(TT);