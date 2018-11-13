clc;clear ;

M=5;
L=100;                    % the total length of the raod (m)
v_speed=120*(1000/3600);        % speed of cars (m/s)
B_RSU=1.25*1000 ;             % Bandwith of the each RSU (HZ)
P_v=100*(10^(-3));        % the transmission power of each vehicle(watt)
N_v=10^(-13);             % the noise power(watt) 

% lambda_right=20;
% 
% lambda_left=20;

number_mean=15;
take_average=250;
result=zeros(number_mean,1);
mean_count=zeros(number_mean,1);

for jj=1:number_mean
   for t_a=1:take_average
    N_right=poissrnd(jj+10);
    N_left=poissrnd(jj+10);
    mean_count(jj)=jj+10;
    result(jj,t_a) = after_opti_f(M,L,v_speed,B_RSU,P_v,N_v,N_right,N_left);
   end
end
result_sum= sum(result,2);
result_sum_average= result_sum/take_average;
result_max= max(result,[],2);
result_min=min(result,[],2);

figure
plot(mean_count,result_sum_average);
hold on 
plot(mean_count,result_max);
hold on 
plot(mean_count,result_min);

save('result.mat','result','result_sum_average','result_max','result_min');