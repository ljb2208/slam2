#pragma once

#include <string.h>
#include <string>
#include <cmath>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <locale>
#include <algorithm>

extern float benchmarkSetting_fxfyfac;
extern int benchmarkSetting_width;
extern int benchmarkSetting_height;
extern float benchmark_varNoise;
extern float benchmark_varBlurNoise;
extern int benchmark_noiseGridsize;
extern float benchmark_initializerSlackFactor;

extern bool setting_render_display3D;
extern bool settings_showCurrentCamera;
extern bool settings_showTrajectory;
extern bool settings_showGroundTruth;
extern int  settings_featureAgeDiscrim;
extern std::string settings_gtFileName;
extern std::string settings_gtOutputFileName;
extern std::string settings_inputPath;
extern std::string settings_paramFile;
extern std::string settings_calibFile;
extern std::string settings_outputFile;
extern int settings_imageOffset;
extern float settings_nccTolerance;
extern int settings_maxFeatures;
extern bool settings_exitOnEnd;


inline void split(const std::string& src, const std::string& delim, std::vector<std::string>& dest)
{
    std::string str = src;
    std::string::size_type start = 0, index;
    std::string substr;

    index = str.find_first_of(delim, start);    //在str中查找(起始：start) delim的任意字符的第一次出现的位置
//    while(index != std::string::npos)
    while(1)
    {
        substr = str.substr(start, index-start);
        dest.push_back(substr);
        start = str.find_first_not_of(delim, index);    //在str中查找(起始：index) 第一个不属于delim的字符出现的位置
        if(start == std::string::npos) return;

        index = str.find_first_of(delim, start);
    }
}

inline void loadSettings(std::string fileName)
{
    std::ifstream settingFile(fileName.c_str());
    std::string temp;
    std::string delim = ":";

    while (std::getline(settingFile, temp))
    {
        std::vector<std::string> results;
        split(temp, delim, results);

        if (results.size() < 2)
            continue;

        std::string setting = results[0];
        std::transform(setting.begin(), setting.end(), setting.begin(), ::tolower);

        if (setting.compare("gtfilename") == 0)
        {
            settings_gtFileName = results[1];
            printf("Settings: Using ground truth: %s\n", settings_gtFileName.c_str());
            continue;
        }

        if (setting.compare("gtoutputfilename") == 0)
        {
            settings_gtOutputFileName = results[1];
            printf("Settings: Using ground truth output file: %s\n", settings_gtOutputFileName.c_str());
            continue;
        }

        if (setting.compare("inputpath") == 0)
        {
            settings_inputPath = results[1];
            printf("Settings: Using input path: %s\n", settings_inputPath.c_str());
            continue;
        }

        if (setting.compare("outputfile") == 0)
        {
            settings_outputFile = results[1];
            printf("Settings: Using output file: %s\n", settings_outputFile.c_str());
            continue;
        }

        if (setting.compare("paramfile") == 0)
        {
            settings_paramFile = results[1];
            printf("Settings: Using parameter file: %s\n", settings_paramFile.c_str());
            continue;
        }

        if (setting.compare("calibfile") == 0)
        {
            settings_calibFile = results[1];
            printf("Settings: Using calibration file: %s\n", settings_calibFile.c_str());
            continue;
        }

        if (setting.compare("imageoffset") == 0)
        {
            settings_imageOffset = std::stoi(results[1]);
            printf("Settings: Using image offset: %i\n", settings_imageOffset);
            continue;
        }

        if (setting.compare("ncctolerance") == 0)
        {
            settings_nccTolerance = std::stof(results[1]);
            printf("Settings: Using NCC tolerance: %f\n", settings_nccTolerance);
            continue;
        }

        if (setting.compare("maxfeatures") == 0)
        {
            settings_maxFeatures = std::stoi(results[1]);
            printf("Settings: Using max features: %i\n", settings_maxFeatures);
            continue;
        }

        if (setting.compare("exitonend") == 0)
        {
            settings_exitOnEnd = std::stoi(results[1]);
            printf("Settings: Using Exit On End: %i\n", settings_exitOnEnd);
            continue;
        }
    } 

    settingFile.close();
}