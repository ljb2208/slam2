#pragma once

#include "Odometry/Matches.h"
#include <vector>

class Histogram
{
    public:
        Histogram();
        Histogram(std::vector<Matches::p_match> p_matched, int width, int height, int bucketWidth, int bucketHeight);
        ~Histogram();
        int calculateSAD(Histogram* compHistogram);

        struct hist_bucket
        {
            int count[4];
        };

        std::vector<hist_bucket> histBuckets;

    private:
        
        int32_t bucket_cols;
        int32_t bucket_rows;
};