function newdata = getData(data, filePath, range)
    opts = spreadsheetImportOptions("NumVariables", 21);

    % Specify sheet and range
    opts.Sheet = "Sheet1";
    opts.DataRange = "A500:U1000";

    % Specify column names and types
    opts.VariableNames = ["globalTime", "timeNow", "rb1x", "rb1y", "rb1z", "rb2x", "rb2y", "rb2z", "rb3x", "rb3y", "rb3z", "rb4x", "rb4y", "rb4z", "rb5x", "rb5y", "rb5z", "u1", "u2", "u3", "u4"];
    opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
    
    newdata = [data ; readtable(filePath,opts,"UseExcel", false)];
end