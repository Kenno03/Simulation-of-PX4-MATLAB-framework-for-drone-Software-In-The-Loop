function Result = get_propeller_coefficients(Propeller)
%GET_PROPELLER_COEFFICIENTS
% Load previously calculated BEMT coefficients if available.
% Otherwise calculate and save them.

%% File location

propellerFolder = fullfile(pwd, "Propeller");

fileName = Propeller.ID + "_BEMT.mat";
filePath = fullfile(propellerFolder, fileName);

%% Load or calculate

if isfile(filePath)

    data = load(filePath, "Result");

    Result = data.Result;

    fprintf("\nLoaded existing BEMT data for: %s\n", ...
        Propeller.Name);

else

    fprintf("\nNo stored BEMT data found for: %s\n", ...
        Propeller.Name);

    fprintf("Calculating BEMT coefficients...\n");

    Result = compute_propeller_coefficients(Propeller);

end

end