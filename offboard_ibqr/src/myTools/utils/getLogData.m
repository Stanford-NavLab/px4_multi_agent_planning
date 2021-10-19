function ret = getLogData(fName)
%% Import data from text file
%% Set up the Import Options and import the data
    opts = delimitedTextImportOptions("NumVariables", 14);

    % Specify range and delimiter
    opts.DataLines = [2, Inf];
    opts.Delimiter = " ";

    % Specify column names and types
    opts.VariableNames = ["timestamp", ...
                          "rx", "ry", "rz", ...
                          "vx", "vy", "vz", ...
                          "qx", "qy", "qz", "qw", ...
                          "r", "p", "y"];

    opts.VariableTypes = ["double", ...
                          "single", "single", "single", ...
                          "single", "single", "single", ...
                          "single", "single", "single", "single", ...
                          "single", "single", "single"];

    % Specify file level properties
    opts.ExtraColumnsRule = "ignore";
    opts.EmptyLineRule = "read";
    opts.ConsecutiveDelimitersRule = "join";
    opts.LeadingDelimitersRule = "ignore";

    % Import the data
    test = readtable(fName, opts);

%% Convert to output type
    ret = rearrange(table2array(test));

%% Clear temporary variables
    clear opts
end

