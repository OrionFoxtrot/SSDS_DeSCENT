clear;clc;close all;

% filename= "V2_5_0_R1_Recieve_Logs.txt";
% filename= "V2_5_0_R1_Recieve_Logs_T2.txt";
% filename= "V2_5_0_R1_Recieve_Logs_T3.txt";
% filename = "V2_5_0_R1_Recieve_Logs_Really_Bad.txt";
% filename = "V2_5_0_R1_Recieve_Logs_T4_CS6.txt";
% filename = "CS6_Post_Recharge.txt"
% filename ="CS6_recievelogs_2.txt";


data = parseAndPlotPacketCounter(filename);


title("ChipSat: "+string(data.CSID(1)))
hold on
yyaxis("left")
ylabel("Packet Counter")
plot(data.ReceiveTime, data.Packet_counter);
yyaxis("right")
ylabel("Battery Charge %")
plot(data.ReceiveTime, data.Cell_percentage_percent);
ylim([0,100]);