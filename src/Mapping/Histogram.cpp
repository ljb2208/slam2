#include "Histogram.h"

Histogram::Histogram(std::vector<std::shared_ptr<Matches::p_match>> p_matched, int width, int height, int bucketWidth, int bucketHeight)
{
    bucket_cols = (int32_t)floor(width/bucketWidth)+1;
    bucket_rows = (int32_t)floor(height/bucketHeight)+1;
    
    for (int i=0; i < bucket_cols; i++)
    {
        for (int j=0; j < bucket_rows;j++)
        {
            hist_bucket h;
            histBuckets.push_back(h);
        }
    }

    for (int i=0; i < p_matched.size(); i++)
    {
        int32_t u = (int32_t)floor(p_matched[i]->u1c/bucketWidth);
        int32_t v = (int32_t)floor(p_matched[i]->v1c/bucketHeight);  

        histBuckets[v* bucket_cols + u].count[p_matched[i]->max1.c]++;
    }
}

Histogram::Histogram()
{
    bucket_cols = bucket_rows = 0;
}

Histogram::~Histogram()
{
}

int Histogram::calculateSAD(Histogram* compHistogram)
{
    int count = bucket_cols * bucket_rows;
    int value = 0;

    if (histBuckets.size() != count || compHistogram->histBuckets.size() != count)
        return -1;

    for (int i=0; i < count; i++)
    {
        for (int y=0; y < 4; y++)
        {
            value += abs(histBuckets[i].count[y] - compHistogram->histBuckets[i].count[y]);
        }
    }

    return value;
}