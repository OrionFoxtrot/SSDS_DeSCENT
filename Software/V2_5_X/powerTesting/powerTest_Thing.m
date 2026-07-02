clear;clc;close all;
df = readtable("Recieve_Logs_V2_5_0_ChipSat.csv");

Time = df.Time;
Counter = df.Counter_n_;
Voltage = df.Voltage_V_;
SoC = df.SoC___;
Temp = df.Temp_c_;

yyaxis left
plot(Time, Counter)
ylabel("Counter Variable")
yyaxis right
plot(Time, Voltage)
ylabel("Battery Voltage")