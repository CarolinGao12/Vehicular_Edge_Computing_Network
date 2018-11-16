clc;clear ;
% constant value
Z=4;
M=7*Z;                          % The number of server
L_m=20;                         % The segment length

L_rd=5*Z*L_m;                   % The road length
% L_rd_total=7*Z*L_m;             % The total length(m)  
v_speed=120*(1000/3600);        % speed of cars (m/s)
B_RSU=1.25*1000 ;               % Bandwith of the each RSU (HZ)
P_v=100*(10^(-3));              % the transmission power of each vehicle(watt)
N_v=10^(-13);                   % the noise power(watt) 

c_car = 0.4;
F_limit=15*ones(1,M);
buffer_number=4;
recy_time=6;

%initialization
N_initial_right=randi([10,25]);
N_initial_left=randi([10,25]);
[V_postion_x_right_initial,V_postion_y_right_initial,V_postion_x_left_initial,V_postion_y_left_initial]=initial_condition(Z,L_m,L_rd,N_initial_right,N_initial_left);

mean_number =15;
take_average=250;
result_opti=zeros(mean_number,take_average);
count_mean=zeros(mean_number,1);
%add new car 
for jj=1:mean_number
       for t_a =1:take_average 
            N_right_add=poissrnd(jj+10);
            N_left_add=poissrnd(jj+10);
            count_mean(jj)=jj+10;
            [result_opti(jj,t_a)] = add_new_car_opti(Z,L_m,M,B_RSU,P_v,N_v,c_car,F_limit,buffer_number,N_right_add,N_left_add,V_postion_x_right_initial,V_postion_y_right_initial,V_postion_x_left_initial,V_postion_y_left_initial,v_speed,recy_time);

       end
end
 
result_opti_sum= sum(result_opti,2);
result_opti_sum_average= result_opti_sum/take_average;

sort_result_opti=sort(result_opti,2);

result_opti_max=sort_result_opti(:,241:end);
result_opti_max_average=sum(result_opti_max,2)/10;

result_opti_min=sort_result_opti(:,1:10);
result_opti_min_average= sum(result_opti_min,2)/10; 

figure
plot(count_mean,result_opti_sum_average);
hold on 
plot(count_mean,result_opti_max_average);
hold on 
plot(count_mean,result_opti_min_average);




save('result_opti.mat','result_opti','result_opti_sum_average','result_opti_max_average','result_opti_min_average','result_opti_max','result_opti_min');