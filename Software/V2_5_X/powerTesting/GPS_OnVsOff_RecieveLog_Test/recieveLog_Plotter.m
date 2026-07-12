% Tests power plot from recieve logs. Used to test performance
% of GPS and no GPS modules.

clear; clc; close all;

df = readtable("Recieve_Logs_V2_5_0_ChipSat_GPSOFF_Long.csv");

rawTime = df.Time;

if isduration(rawTime)
    tDay = rawTime;
elseif isdatetime(rawTime)
    tDay = timeofday(rawTime);
else
    tDay = duration(string(rawTime), "InputFormat", "hh:mm:ss.SSS");
end

tDay = tDay(:);

% Detect rollover from 23:xx to 00:xx
dayOffset = days(cumsum([0; diff(tDay) < seconds(0)]));

% Continuous time, but still based on time of day
TimeContinuous = tDay + dayOffset;

% Convert to fake datetime so axis can display 24-hour clock labels
fakeStartDate = datetime(2026,1,1);
TimePlot = fakeStartDate + TimeContinuous;

Counter = df.Counter_n_;
Voltage = df.Voltage_V_;

figure

yyaxis left
scatter(TimePlot, Counter)
ylabel("Counter Variable")

yyaxis right
scatter(TimePlot, Voltage)
ylabel("Battery Voltage")

xlabel("Time of Day")
xtickformat("HH:mm:ss")   % display only 00:00:00 to 23:59:59
grid on