clc;clear ;
% constant value
Z=4;
M=7*Z;                          % The number of server
L_m=20;                         % The segment length
M_C=4*M;                      % The number of cores       
 
v_speed=120*(1000/3600);        % speed of cars (m/s)
B_RSU=1.25*1000 ;               % Bandwith of the each RSU (HZ)
P_v=100*(10^(-3));              % the transmission power of each vehicle(watt)
N_v=10^(-13);                   % the noise power(watt) 

c_car = 0.4;
% F_limit=15*ones(1,M);
F_limit_core=15/4*ones(1,M_C);
buffer_number=4;
recy_time=0.5;
Z_L_m=Z*L_m;

%initial condition
N_initial_right=randi([10,25]);
N_initial_left=randi([10,25]);
V_postion_x_right_initial=7*Z_L_m.*rand(N_initial_right,1);
V_postion_y_right_initial =rand(N_initial_right,1);
for ii=1:N_initial_right
    if V_postion_y_right_initial(ii) >=0.5
        V_postion_y_right_initial(ii)=8;
    else
        V_postion_y_right_initial(ii)=6;
    end
end


V_postion_x_left_initial=7*Z_L_m.*rand(N_initial_left,1);
V_postion_y_left_initial =rand(N_initial_left,1);
for ii=1:N_initial_left
    if V_postion_y_left_initial(ii) >=0.5
        V_postion_y_left_initial(ii)=4;
    else
        V_postion_y_left_initial(ii)=2;
    end
end




% take_average= 200;
% mean_number=15;
% % result_opti=zeros(mean_number,take_average);
% count_mean=zeros(mean_number,1);

% for ii= 1:mean_number
%     for t_a=1:take_average
%         N_right_add=poissrnd(ii+10);
%         N_left_add=poissrnd(ii+10);
%         count_mean(ii)=ii+10;
%         [result_opti(ii,t_a),vehicle_position_delete_right,vehicle_position_delete_left] =add_car_opti(N_right_add,N_left_add,L_m,v_speed,recy_time,V_postion_x_right_initial,V_postion_y_right_initial,V_postion_x_left_initial,V_postion_y_left_initial,Z_L_m,B_RSU,M,P_v,N_v,c_car,F_limit,buffer_number);
%         V_postion_x_right_initial=vehicle_position_delete_right(:,1);
%         V_postion_y_right_initial=vehicle_position_delete_right(:,2);
%         V_postion_x_left_initial =vehicle_position_delete_left(:,1);
%         V_postion_y_left_initial=vehicle_position_delete_left(:,2);
% 
%     end
% end
% 
% 
% result_opti_sum= sum(result_opti,2);
% result_opti_sum_average= result_opti_sum/take_average;
% sort_result_opti=sort(result_opti,2);
% 
% result_opti_max=sort_result_opti(:,191:end);
% result_opti_max_average=sum(result_opti_max,2)/10;
% 
% result_opti_min=sort_result_opti(:,1:10);
% result_opti_min_average= sum(result_opti_min,2)/10; 
% 
% figure
% plot(count_mean,result_opti_sum_average);
% hold on 
% plot(count_mean,result_opti_max_average);
% hold on 
% plot(count_mean,result_opti_min_average);
% 
% save('result_opti.mat','result_opti','result_opti_sum_average','result_opti_max_average','result_opti_min_average','result_opti_max','result_opti_min');


take_average= 300;
mean_number=15;
result_opti_core=zeros(mean_number,take_average);
count_mean=zeros(mean_number,1);
Time_delay=zeros(1,M_C+1);



for ii=1:mean_number
    for t_a=1:take_average
        N_right_add=poissrnd(ii+10);
        N_left_add=poissrnd(ii+10);
        count_mean(ii)=ii+10;
        [result_opti_core(ii,t_a),TT,vehicle_position_delete_right,vehicle_position_delete_left,j_selection,vehicle_position_task]=choose_core(F_limit_core,N_right_add,N_left_add,M,L_m,v_speed,B_RSU,P_v,N_v,c_car,recy_time,Z_L_m,Time_delay,V_postion_x_right_initial,V_postion_y_right_initial,V_postion_x_left_initial,V_postion_y_left_initial);
        TT_matrix=full(TT);
        
        for i= 1:length(vehicle_position_task)
            for jj =1:M_C+1
                if TT_matrix (i,jj)< 0.001
                TT_matrix(i,jj) = 0 ;
                end
            end
        end  
        Time_delay=sum(TT_matrix)-recy_time;

        for jj=1:M_C+1
            if Time_delay(:,jj)<0
                Time_delay(:,jj)=0;
            else
                Time_delay(:,jj)= Time_delay(:,jj);
            end
        end
        
        V_postion_x_right_initial=vehicle_position_delete_right(:,1);
        V_postion_y_right_initial=vehicle_position_delete_right(:,2);
        V_postion_x_left_initial =vehicle_position_delete_left(:,1);
        V_postion_y_left_initial=vehicle_position_delete_left(:,2);
        
    end
end
