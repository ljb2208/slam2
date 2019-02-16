#include "Matches.h"
#include <stdlib.h>


Matches::Matches(int imageWidth, int imageHeight)
{
    this->imageHeight = imageHeight;
    this->imageWidth = imageWidth;
    map_matched = (Matches::p_match**)calloc(imageWidth*imageHeight*8, sizeof(Matches::p_match*));
}

Matches::~Matches()
{
    free(map_matched);
}

void Matches::ageFeaturePoints()
{
    for (int i=0; i < p_matched.size(); i++)
    {
        p_matched[i].u1p3 = p_matched[i].u1p2;
        p_matched[i].v1p3 = p_matched[i].v1p2;
        p_matched[i].u1p2 = p_matched[i].u1p;
        p_matched[i].v1p2 = p_matched[i].v1p;
        p_matched[i].max1p = p_matched[i].max1;
        p_matched[i].max2p = p_matched[i].max2;
        p_matched[i].age++;
    }
}

void Matches::resetMatches()
{
    for (int i=0; i < p_matched.size(); i++)
    {
      p_matched[i].active = false;
      p_matched[i].matched = false;
      p_matched[i].outlier = false;
    }

    inlierMatches.clear();
    selectedMatches.clear();
}

void Matches::clearOutliers()
{
    inlierMatches.erase(std::remove_if(inlierMatches.begin(), inlierMatches.end(),
                [](Matches::p_match* pmatch) { return pmatch->outlier;}), inlierMatches.end());
}

int32_t Matches::getInlierCount()
{
    return inlierMatches.size();
}

int32_t Matches::getTotalMatches()
{
    return p_matched.size();
}

bool Matches::push_back(Matches::p_match match, bool current)
{
    // if match not found add
    if (!matchExists(match, current))
    {
        Matches::p_match* mtch = (Matches::p_match*) malloc(sizeof(Matches::p_match));

        std::memcpy(mtch, &match, sizeof(Matches::p_match));

        mtch->matched = true;
        mtch->active = true;
        mtch->outlier = false;
        mtch->imax1 = mtch->max1;
        mtch->imax2 = mtch->max2;
        inlierMatches.push_back(mtch);
        p_matched.push_back(*mtch);
        addToMap(*mtch);
    }    
}

void Matches::addToMap(Matches::p_match match)
{
    int32_t u,v;
    u = (int32_t) match.u1c;
    v = (int32_t) match.v1c;    
    int32_t addr = getAddressOffsetMatches(u, v, match.max1.c, imageWidth, imageHeight, 0);
    map_matched[addr] = &match;    

    u = (int32_t) match.u2c;
    v = (int32_t) match.v2c;    
    addr = getAddressOffsetMatches(u, v, match.max2.c, imageWidth, imageHeight, 1);
    map_matched[addr] = &match;    
    
}

void Matches::clearMap()
{
    for (int i=0; i < imageHeight * imageWidth * 4; i++)
        map_matched[i] = NULL;
}

Matches::p_match* Matches::getMatchbyMaxima(Matches::maximum max, bool right)
{
    int32_t right_val = 0;

    if (right)
        right_val = 1;
    Matches::p_match* result = map_matched[getAddressOffsetMatches(max.u, max.v, max.c, imageWidth, imageHeight, right_val)];

    return result;
}

bool Matches::matchExists(Matches::p_match match, bool current)
{
    bool found = false;
    for (int i=0; i < p_matched.size(); i++)
    {
        if (current)
        {
            if (p_matched[i].i1c == match.i1c)
                found = true;
        }
        else
        {
            if (getKey(p_matched[i]) == getPriorKey(match))
                found = true;
        }

        if (found)
        {
            p_matched[i].outlier = false;
            p_matched[i].u1p = match.u1p;
            p_matched[i].v1p = match.v1p;
            p_matched[i].i1p = match.i1p;
            p_matched[i].u2p = match.u2p;
            p_matched[i].v2p = match.v2p;
            p_matched[i].i2p = match.i2p;
            p_matched[i].u1c = match.u1c;
            p_matched[i].v1c = match.v1c;
            p_matched[i].i1c = match.i1c;
            p_matched[i].u2c = match.u2c;
            p_matched[i].v2c = match.v2c;
            p_matched[i].i2c = match.i2c;
            p_matched[i].max1 = match.max1;
            p_matched[i].max2 = match.max2;
                    
            if (p_matched[i].active == false)
            {
                inlierMatches.push_back(&p_matched[i]);
                addToMap(p_matched[i]);
            }

            p_matched[i].active = true;    
            p_matched[i].matched = true;
            return true;            
        }

    }

    return false;
}

void Matches::clear()
{
    deleteStaleMatches();
    resetMatches();
    ageFeaturePoints();
}

void Matches::deleteStaleMatches()
{
    if (p_matched.empty())
        return;

    // remove where no current matches
    p_matched.erase(std::remove_if(p_matched.begin(), p_matched.end(),
           [](p_match pm){ return pm.matched == false;}), p_matched.end());  
}

void Matches::bucketFeatures(int32_t max_features,float bucket_width,float bucket_height)
{
    // find max values
    float u_max = 0;
    float v_max = 0;
    for (int i=0; i < inlierMatches.size(); i++)
    {
        if (inlierMatches[i]->u1c>u_max) u_max=inlierMatches[i]->u1c;
        if (inlierMatches[i]->v1c>v_max) v_max=inlierMatches[i]->v1c;
    }

    // allocate number of buckets needed
    int32_t bucket_cols = (int32_t)floor(u_max/bucket_width)+1;
    int32_t bucket_rows = (int32_t)floor(v_max/bucket_height)+1;
    std::vector<Matches::p_match*> *buckets = new std::vector<Matches::p_match*>[bucket_cols*bucket_rows*4];

    printf("umax: %f vmax: %f bucket cols: %i rows: %i\n", u_max, v_max, bucket_cols, bucket_rows);

    // assign matches to their buckets
    for (int i=0; i < inlierMatches.size(); i++)
    {
        int32_t u = (int32_t)floor(inlierMatches[i]->u1c/bucket_width);
        int32_t v = (int32_t)floor(inlierMatches[i]->v1c/bucket_height);        
        buckets[v*bucket_cols+u+inlierMatches[i]->max1.c*bucket_cols].push_back(inlierMatches[i]);
    }

    // refill p_matched from buckets
    for (int32_t c=0; c<bucket_cols; c++) 
    {
        for (int32_t r=0; r<bucket_rows; r++)  
        {            
            int32_t ind = r*bucket_cols +c;
            // sort buckets       
            std::sort(buckets[ind].begin(), buckets[ind].end(), compareMatches);
            std::sort(buckets[ind+1*bucket_cols].begin(), buckets[ind+1*bucket_cols].end(), compareMatches);
            std::sort(buckets[ind+2*bucket_cols].begin(), buckets[ind+2*bucket_cols].end(), compareMatches);
            std::sort(buckets[ind+3*bucket_cols].begin(), buckets[ind+3*bucket_cols].end(), compareMatches);
            
            // add up to max_features features from this bucket to p_matched
            int32_t k=0;

            int32_t cnt0 = buckets[ind].size();
            int32_t cnt1 = buckets[ind+1*bucket_cols].size();
            int32_t cnt2 = buckets[ind+2*bucket_cols].size();
            int32_t cnt3 = buckets[ind+3*bucket_cols].size();

            while  (cnt0 >k || cnt1 >k || cnt2 > k || cnt3 >k)
            {
                if (cnt0 > k)
                    selectedMatches.push_back(buckets[ind].at(k));

                if (cnt1 > k)
                    selectedMatches.push_back(buckets[ind + 1*bucket_cols].at(k));

                if (cnt2 > k)
                    selectedMatches.push_back(buckets[ind + 2*bucket_cols].at(k));
                
                if (cnt3 > k)
                    selectedMatches.push_back(buckets[ind + 3*bucket_cols].at(k));

                k++;

                if (k >= (max_features/2))
                    break;                
            }      
        }
    }

    // free buckets
    delete []buckets;
}

int32_t Matches::getSelectedCount()
{
    return selectedMatches.size();
}

int32_t Matches::getKey(Matches::p_match match)
{
    return getKey(&match);
}

int32_t Matches::getKey(Matches::p_match* match)
{
    float u1c = roundf(match->u1c);
    float v1c = roundf(match->v1c);
    return (match->max1.c * 10000000 + u1c * 10000 + v1c);
}

int32_t Matches::getPriorKey(Matches::p_match match)
{
    float u1p = roundf(match.u1p);
    float v1p = roundf(match.v1p);
    return (match.max1.c * 10000000 + u1p * 10000 + v1p);
}