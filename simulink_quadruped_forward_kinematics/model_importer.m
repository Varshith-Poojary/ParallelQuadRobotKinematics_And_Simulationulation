%% Open your Simulink model
open_system('robot_model');

% Create a Simscape Multibody importer
importer = robotics.SMImporter();

% Set the 'BreakChains' property to 'remove-joints' to break closed chains
importer.BreakChains = 'remove-joints';

%Generate a rigidBodyTree and an import information object from the Simulink model
[robot, importInfo] = importrobot(importer, gcs);

% See import details
showdetails(importInfo);

% Close the open system
close_system(gcs, 0);

