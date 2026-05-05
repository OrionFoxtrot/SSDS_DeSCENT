clear;clc;close all;
df = readtable("Full_V2_3_X_ChipSat_Discharge_Charge.txt")


hold on
plot(df.time, df.batVoltage/1000)
title("batVoltage");
ylim([3 4]);
hold off
figure()
hold on
title("loadVoltage")
plot(df.time, df.loadVoltage/1000)
hold off