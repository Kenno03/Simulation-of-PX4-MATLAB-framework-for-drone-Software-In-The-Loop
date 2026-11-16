%INITIALIZATION Start of every session (first section of Main.mlx): paths and selection lists.
%   Clears the workspace, makes the project root the current folder, adds the project folders to the
%   MATLAB path and creates the option lists that Main.mlx's drop-downs use (motors, propellers,
%   frames, plant types, payloads and PX4 gain profiles). Run it first; it clears the base workspace,
%   so do not run it in the middle of an experiment.

clc;
clear all;
close all;

%% ------------------------------------------------------------------------
% Locate project root
% -------------------------------------------------------------------------

thisFile = mfilename('fullpath');

% initialization.m is located directly in the project root
projectRoot = fileparts(thisFile);

%% ------------------------------------------------------------------------
% Set project root as current folder
% -------------------------------------------------------------------------

cd(projectRoot);

%% ------------------------------------------------------------------------
% Add project folders to MATLAB path
% -------------------------------------------------------------------------

% Root, code folders and asset folders (the list is in project_paths.m)
addpath(projectRoot);
project_paths(projectRoot);

%% ------------------------------------------------------------------------
% Project information
% -------------------------------------------------------------------------

fprintf("Project initialized.\n");
fprintf("Project root: %s\n", projectRoot);

%% ------------------------------------------------------------------------
% Hardware selection options
% -------------------------------------------------------------------------

MotorOptions = motor_parameters();

PropellerOptions = propeller_parameters();

DroneOptions = drone_parameters();

PlantOptions = plant_parameters();

TopPayloadOptions = top_payload_parameters();

BottomPayloadOptions = bottom_payload_parameters();

PX4TuneOptions = ...
    px4_tuning_parameters();