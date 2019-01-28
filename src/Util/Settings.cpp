#include "Settings.h"

// for benchmarking different undistortion settings
float benchmarkSetting_fxfyfac = 0;
int benchmarkSetting_width = 0;
int benchmarkSetting_height = 0;
float benchmark_varNoise = 0;
float benchmark_varBlurNoise = 0;
float benchmark_initializerSlackFactor = 1;
int benchmark_noiseGridsize = 3;

bool setting_render_display3D = true;
bool settings_showCurrentCamera = true;
bool settings_showTrajectory = true;
bool settings_showGroundTruth = false;