clear;clc;close all;
% filename ="Three_Four_Seven_Overnight_Recieve_Logs.txt";
% filename= "Two_Seven_Recieve_Logs_Day.txt";
% filename = "two_five_seven.txt";
% filename = "overnight_most_chipsats_2026_Jul_22 23_48_14.txt";
filename = "four_newcap_six.txt";

allData = parseAndPlotPacketCounter(filename);
all_CSIDs = unique(allData.CSID)

target_CSIDs = [];
target_CSIDs = all_CSIDs;

for i = 1:length(target_CSIDs)
    target_CSID = target_CSIDs(i);

    % Extract packets from the current ChipSat
    CSID_table = allData(allData.CSID == target_CSID, :);

    % Sort packets chronologically
    CSID_table = sortrows(CSID_table, "ReceiveTime");

    figure;

    % Packet counter on left axis
    yyaxis left
    plot(CSID_table.ReceiveTime, CSID_table.Packet_counter, ...
         "DisplayName", "Packet counter");
    ylabel("Packet counter");
    ylim([0 inf])

    % Battery state of charge on right axis
    yyaxis right
    plot(CSID_table.ReceiveTime, CSID_table.Cell_percentage_percent, ...
         "DisplayName", "Battery SOC");
    ylabel("Battery SOC (%)");
    ylim([0 100]);

    xlabel("Receive time");
    title("ChipSat CSID " + target_CSID);
    grid on;
end