clear; clc; 
addpath("Misc_DS\")
% close all;
figure()
save = 0;
% name = "V2 Charging Testing - Charge.csv";
% name = "V2 Charging Testing - Drain.csv";
% name = "V2 Charging Testing - V1.csv"
% name = "V2 Charging Testing - V1_With_Delay.csv"
% name = "V2 Charging Testing - V1_Low_PWR_Delay_Sleep.csv"
% name = "V2 Charging Testing - V1_Long_Charge.csv"
% name = "V2 Charging Testing - V1_Super_Long_Charge.csv"
name = "power_profile_BOHR.csv";

offset = 1;
data = csvread(name,offset);
datatrim = data(1:length(data)-offset ,:); %get rid of last failed link

N = length(datatrim(:,1));              % number of readings (change this)
dt = 5e-3;             % X ms in seconds
dt_min = dt / 60;      % convert step to minutes

x = (0:N-1) * dt_min;  % time array in minutes


subplot(1,2,1);
hold on
plot(x,datatrim(:,1)*10^-3,'DisplayName','Battery Voltage')
xlabel('Time (s)')
ylabel('Voltage (V)')
title('Voltage')
subplot(1,2,2);
hold on
plot(x,datatrim(:,2)*10^-3,'DisplayName','Battery Current')
xlabel('Time (s)')
ylabel('Current (A)')
title('Current')

% legend()
titleStr = sprintf('VI Cycle for: %s', name); 
sgtitle(titleStr,'Interpreter','None')


saveStr = char(name)
saveStr = saveStr(1:end-4)
saveStr = strcat(saveStr,'.png')

if save
    saveas(gcf,'akjjkfnsfjldnjlsd.png');
end
