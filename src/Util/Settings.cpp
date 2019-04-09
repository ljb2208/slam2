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
bool settings_showGroundTruth = true;

int settings_featureAgeDiscrim = 6;

std::string settings_gtFileName = "/home/lbarnett/development/odometry/poses/07.txt";
std::string settings_gtOutputFileName = "/home/lbarnett/development/odometry/poses/gt.csv";
std::string settings_inputPath = "/home/lbarnett/development/odometry/07";
std::string settings_paramFile = "/home/lbarnett/development/odometry/07/param/camera.txt";
std::string settings_calibFile = "/home/lbarnett/development/odometry/07/param/camera.txt";
std::string settings_outputFile = "outputs.csv";

int settings_imageOffset = 0;

float settings_nccTolerance = 0.85;
int settings_maxFeatures = 6;

bool settings_exitOnEnd = false;
