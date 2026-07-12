clear; clc; close all;

filename = "V2_6_0_R0_CSID_3_PowerProfile.csv";
% filename = "V2_5_0_R0_CSID_2_PowerProfile.csv";
df = readtable(filename);

Time = df.Time;
MET = seconds(Time - Time(1));
fprintf("Max I %.3f \n", max(df.MaxCurrent))
maskLen = length(MET);
% or 
maskLen = 500;

MET = MET(1:maskLen);
V = df.Voltage(1:maskLen);
I = df.Current(1:maskLen);
maxI = df.MaxCurrent(1:maskLen);

subplot(1,2,1);
plot(MET, V)
ylim([0,4000])

subplot(1,2,2)
hold on
plot(MET,I)
plot(MET,maxI)
xlabel("MET")
ylim([0,120]);

hold off