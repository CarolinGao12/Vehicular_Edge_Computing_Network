function [result_opti]= after_opti_f(M,L,v_speed,B_RSU,P_v,N_v,N_right,N_left)
% clc;clear ;
% M=5;
% L=100;                    % the total length of the raod (m)
% 
L_m=L/M;                  %each RSU length (m)
% 
% v_speed=120*(1000/3600);        % speed of cars (m/s)
% B_RSU=1.25*1000 ;             % Bandwith of the each RSU (HZ)
% P_v=100*(10^(-3));        % the transmission power of each vehicle(watt)
% N_v=10^(-13);             % the noise power(watt) 


% M_postion_x=[10;30;50;70;90];
% M_postion_y=[ 0; 0; 0; 0; 0];

%get the positon of the cars
% The drving cars towards to right direction 
% lambda_right=20;

% N_right=poissrnd(lambda_right);
V_postion_x_right=60.*rand(N_right,1);
V_postion_y_right =rand(N_right,1);
for ii=1:N_right
    if V_postion_y_right(ii) >=0.5
        V_postion_y_right(ii)=8;
    else
        V_postion_y_right(ii)=6;
    end
end

% The drving cars towards to left direction 
% lambda_left=20;

% N_left=poissrnd(lambda_left);
V_postion_x_left=60+40.*rand(N_left,1);
V_postion_y_left =rand(N_left,1);
for ii=1:N_left
    if V_postion_y_left(ii) >=0.5
        V_postion_y_left(ii)=4;
    else
        V_postion_y_left(ii)=2;
    end
end
vehicle_position=[V_postion_x_right, V_postion_y_right ;V_postion_x_left,V_postion_y_left];

% The driving time
L_all_right=zeros(N_right,5);
for ii=1:N_right
    for jj=1:M
        L_all_right(ii,jj)=-V_postion_x_right(ii)+L_m*(jj-1);

        
    end
end


L_all_left=zeros(N_left,5);
for ii=1:N_left
    for jj=1:M
        L_all_left(ii,jj)=V_postion_x_left(ii)-L_m*jj;

        
    end
end

L_all=[L_all_right ; L_all_left];
T_d_all = (zeros(length(vehicle_position),M+1));
for ii = 1:length(vehicle_position)
    T_d_all(ii,2:end) = L_all(ii,:) ./ v_speed;
end 

% The communication time


h_length= vehicle_position(:,2);
w_length=10*ones(length(vehicle_position),1);

d_t=zeros(length(vehicle_position),1);
r_c=zeros(length(vehicle_position),1);
dataSize = (100+200*rand(length(vehicle_position),1));

d_t=sqrt(w_length.^2+h_length.^2);
r_c= B_RSU* log2(1+(P_v *  (d_t.^(-2.5))  )/N_v);
T_c= dataSize./r_c;

T_c_all = zeros(length(vehicle_position),M+1);
for ii = 2:M+1
    T_c_all(:,ii) = T_c;
end

% The computing time 
c_i=0.5+rand(length(vehicle_position),1);
c_i_all = ones(length(vehicle_position),M+1);
for ii = 1:M+1
    c_i_all(:,ii) = c_i;
end

F_limit=[15,15,15,15,15];
c_car = 0.4;
T_max = (8+2*rand(length(vehicle_position),1));


cvx_begin
    cvx_solver MOSEK
    variable f_ij_all(length(vehicle_position),M);
    variable j_selection(length(vehicle_position),M+1) binary;
%   variable j_selection(N,M+1) ;
    expression TT;

    
    TT=(j_selection.*(T_d_all+T_c_all));
    TT(:,1)=TT(:,1)+j_selection(:,1).*(c_i_all(:,1) / c_car);

    for ii=1:length(vehicle_position)
        for jj=1:M
           TT(ii,jj+1)=TT(ii,jj+1)+ c_i_all(ii,jj+1)*quad_over_lin(j_selection(ii,jj+1),f_ij_all(ii,jj));
        end
    end
    
    minimize sum(sum(TT));
    
subject to 
    
    for ii = 1:length(vehicle_position)
        sum(j_selection(ii,:))==1;          %condition (8c)
    end
    
    sum(f_ij_all)<=F_limit;                 %condition (8d)
    
    for ii=1:length(vehicle_position)
        for jj=1:M
            f_ij_all(ii,jj)<=F_limit(jj)*j_selection(ii,jj+1);         %condition (8e)
            f_ij_all(ii,jj)>=0;
        end
    end

      sum(TT,2)<= T_max;                    %condition (9b)
    
      sum(j_selection(:,2:end))<=10;
      
      for ii= 1:length(vehicle_position)
          for jj= 1:M
              if L_all(ii,jj)<0
                  j_selection(ii,jj+1)==0;
              end
          end
      end
      
    
cvx_end
j_selection_matrix= full(j_selection);
f_ij_all_matrix=full(f_ij_all);
result_opti=cvx_optval;

% ver_line=[20;40;60;80;100];
% hori_line=[2;4;6;8];
% figure
% plot(M_postion_x,M_postion_y,'o', 'MarkerSize', 10);
% hold on 
% plot(vehicle_position(:,1),vehicle_position(:,2),'x', 'MarkerSize', 10);
% hold on 
% plot([ver_line, ver_line],ylim,'g--');
% hold on 
% plot(xlim, [hori_line hori_line],'b--'); % Plot Horizontal Line

end
        
        
        
        
        
        
        
        
        
        
        
        
