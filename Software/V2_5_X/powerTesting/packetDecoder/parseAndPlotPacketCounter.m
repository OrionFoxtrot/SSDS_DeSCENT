function data = parseAndPlotPacketCounter(filename)
%PARSEANDPLOTPACKETCOUNTER Parse a timestamp-prefixed CSV log and plot
%Packet_counter versus receive time.
%
%   data = parseAndPlotPacketCounter(filename)
%
% Expected format:
%   Header1,Header2,...,Packet_counter,...
%   15:03:40.922 -> value1,value2,...,0,...
%
% Non-data lines (for example CRC errors or corrupted text) are skipped.
% Timestamp rollover at midnight is handled automatically.

    arguments
        filename (1,1) string
    end

    fid = fopen(filename, 'rt');
    if fid == -1
        error('Could not open file: %s', filename);
    end
    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

    % Read and clean the CSV header. The regex also removes a UTF-8 BOM.
    headerLine = fgetl(fid);
    if ~ischar(headerLine)
        error('The file is empty or does not contain a header.');
    end
    headerLine = regexprep(headerLine, '^[^A-Za-z_]+', '');
    headers = strtrim(strsplit(headerLine, ','));
    numColumns = numel(headers);

    counterColumn = find(strcmpi(headers, 'Packet_counter'), 1);
    if isempty(counterColumn)
        error('The header does not contain a Packet_counter column.');
    end

    values = zeros(0, numColumns);
    absoluteSeconds = zeros(0, 1);

    previousTimeOfDay = NaN;
    dayOffsetSeconds = 0;
    skippedLines = 0;

    % A valid line contains a receive timestamp, an arrow, and CSV values.
    linePattern = ['^\s*(\d{2}:\d{2}:\d{2}\.\d{3})' ...
                   '\s*->\s*(.*)\s*$'];

    while true
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end

        token = regexp(line, linePattern, 'tokens', 'once');
        if isempty(token)
            skippedLines = skippedLines + 1;
            continue;
        end

        csvFields = strsplit(token{2}, ',');
        if numel(csvFields) ~= numColumns
            skippedLines = skippedLines + 1;
            continue;
        end

        numericRow = str2double(csvFields);
        if any(isnan(numericRow))
            skippedLines = skippedLines + 1;
            continue;
        end

        timeParts = sscanf(token{1}, '%d:%d:%f');
        if numel(timeParts) ~= 3
            skippedLines = skippedLines + 1;
            continue;
        end

        timeOfDaySeconds = timeParts(1) * 3600 + ...
                           timeParts(2) * 60 + timeParts(3);

        % Detect a midnight rollover, while ignoring small timestamp jitter.
        if ~isnan(previousTimeOfDay) && ...
                timeOfDaySeconds < previousTimeOfDay - 12 * 3600
            dayOffsetSeconds = dayOffsetSeconds + 24 * 3600;
        end
        previousTimeOfDay = timeOfDaySeconds;

        values(end + 1, :) = numericRow; %#ok<AGROW>
        absoluteSeconds(end + 1, 1) = ...
            timeOfDaySeconds + dayOffsetSeconds; %#ok<AGROW>
    end

    if isempty(values)
        error('No valid data records were found in %s.', filename);
    end

    % Build a MATLAB table containing all parsed columns plus receive time.
    variableNames = matlab.lang.makeValidName(headers);
    variableNames = matlab.lang.makeUniqueStrings(variableNames);

    data = array2table(values, 'VariableNames', variableNames);
    receiveTime = datetime(2000, 1, 1) + seconds(absoluteSeconds);
    receiveTime.Format = 'HH:mm:ss.SSS';
    data = addvars(data, receiveTime, 'Before', 1, ...
                   'NewVariableNames', 'ReceiveTime');

    % packetCounter = values(:, counterColumn);
    % 
    % % Use stairs because Packet_counter is an integer-valued counter.
    % figure('Name', 'Packet counter versus time');
    % stairs(receiveTime, packetCounter, '-o', ...
    %        'LineWidth', 1.2, 'MarkerSize', 3);
    % grid on;
    % xlabel('Receive time');
    % ylabel('Packet counter');
    % title('Packet counter versus receive time');
    % 
    % % Mark points where the transmitted counter resets or wraps.
    % resetIndices = find(diff(packetCounter) < 0) + 1;
    % if ~isempty(resetIndices)
    %     hold on;
    %     plot(receiveTime(resetIndices), packetCounter(resetIndices), ...
    %          'rv', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    %     legend('Packet counter', 'Counter reset/wrap', ...
    %            'Location', 'best');
    %     hold off;
    % end
    % 
    % fprintf('Parsed %d records and skipped %d malformed/status lines.\n', ...
    %         height(data), skippedLines);
end
