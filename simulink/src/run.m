clear; close all;

SRAAM6_params;

simOut = sim('sraam');

plot_results(simOut, SCENARIO, false);
