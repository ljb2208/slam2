#include "Matches.h"
#include <stdlib.h>


Matches::Matches(int imageWidth, int imageHeight)
{
    this->imageHeight = imageHeight;
    this->imageWidth = imageWidth;
    map_matched = std::make_unique<std::shared_ptr<Matches::p_match>[]>(imageWidth*imageHeight*8);
    // map_matched = (Matches::p_match**)calloc(imageWidth*imageHeight*8, sizeof(Matches::p_match*));

    matchesAdded = matchesFound = 0;
}

Matches::~Matches()
{
    // free(map_matched);
}

void Matches::ageFeaturePoints()
{
    for (int i=0; i < p_matched.size(); i++)
    {
        p_matched[i]->u1p3 = p_matched[i]->u1p2;
        p_matched[i]->v1p3 = p_matched[i]->v1p2;
        p_matched[i]->u1p2 = p_matched[i]->u1p;
        p_matched[i]->v1p2 = p_matched[i]->v1p;        
        p_matched[i]->age++;
    }
}

void Matches::resetMatches()
{
    for (int i=0; i < p_matched.size(); i++)
    {
      p_matched[i]->active = false;
      p_matched[i]->matched = false;
      p_matched[i]->outlier = false;
    }

    inlierMatches.clear();
    selectedMatches.clear();

    matchesAdded = matchesFound = 0;
}

void Matches::clearOutliers()
{
    inlierMatches.erase(std::remove_if(inlierMatches.begin(), inlierMatches.end(),
                [](std::shared_ptr<Matches::p_match> pmatch) { return pmatch->outlier;}), inlierMatches.end());
}

int32_t Matches::getInlierCount()
{
    return inlierMatches.size();
}

int32_t Matches::getTotalMatches()
{
    return p_matched.size();
}

bool Matches::push_back(std::shared_ptr<Matches::p_match> match, bool current)
{

    bool r = validateMatch(match);

    if (r)
        printf("Orig Match corrupt.\n");
    // if match not found add
    if (!matchExists(match, current))
    {
        //Matches::p_match* mtch = (Matches::p_match*) malloc(sizeof(Matches::p_match));

        // Matches::p_match mtch;// = (Matches::p_match*) malloc(sizeof(Matches::p_match));
        std::shared_ptr<Matches::p_match> mtch = std::make_shared<Matches::p_match>();

        std::memcpy(mtch.get(), match.get(), sizeof(Matches::p_match));

        // bool r = validateMatch(&match);

        if (r)
            printf("Orig Match corrupt2.\n");

        mtch->matched = true;
        mtch->active = true;
        mtch->outlier = false;
        mtch->imax1 = mtch->max1;
        mtch->imax2 = mtch->max2;
        p_matched.push_back(mtch);
        inlierMatches.push_back(mtch);

        r = validateMatch(mtch);

        if (r)
            printf("New Match corrupt.\n");


        addToMap(mtch);
        matchesAdded++;

        r = validateMatch(mtch);

        if (r)
            printf("New Match corrupt2.\n");

    }    

    validateMatches("PushBack");
}

void Matches::addToMap(std::shared_ptr<Matches::p_match> match)
{
    int32_t u,v;
    u = (int32_t) match->u1c;
    v = (int32_t) match->v1c;    
    int32_t addr = getAddressOffsetMatches(u, v, match->max1.c, imageWidth, imageHeight, 0);
    map_matched[addr] = match;    

    u = (int32_t) match->u2c;
    v = (int32_t) match->v2c;    
    addr = getAddressOffsetMatches(u, v, match->max2.c, imageWidth, imageHeight, 1);
    map_matched[addr] = match;    
    
}

void Matches::clearMap()
{
    for (int i=0; i < imageHeight * imageWidth * 4; i++)
        map_matched[i] = NULL;
}

std::shared_ptr<Matches::p_match> Matches::getMatchbyMaxima(int32_t u, int32_t v, int32_t c, bool right)
{
    int32_t right_val = 0;

    if (right)
        right_val = 1;
    
    return map_matched[getAddressOffsetMatches(u, v, c, imageWidth, imageHeight, right_val)];
}

bool Matches::matchExists(std::shared_ptr<Matches::p_match> match, bool current)
{
    bool found = false;
    for (int i=0; i < p_matched.size(); i++)
    {
        if (current)
        {
            if (p_matched[i]->i1c == match->i1c)
                found = true;
        }
        else
        {
            if (getKey(p_matched[i]) == getPriorKey(match))
                found = true;
        }

        if (found)
        {
            p_matched[i]->outlier = false;
            p_matched[i]->u1p = match->u1p;
            p_matched[i]->v1p = match->v1p;
            p_matched[i]->i1p = match->i1p;
            p_matched[i]->u2p = match->u2p;
            p_matched[i]->v2p = match->v2p;
            p_matched[i]->i2p = match->i2p;
            p_matched[i]->u1c = match->u1c;
            p_matched[i]->v1c = match->v1c;
            p_matched[i]->i1c = match->i1c;
            p_matched[i]->u2c = match->u2c;
            p_matched[i]->v2c = match->v2c;
            p_matched[i]->i2c = match->i2c;
            p_matched[i]->max1 = match->max1;
            p_matched[i]->max2 = match->max2;
                    
            if (p_matched[i]->active == false)
            {
                inlierMatches.push_back(p_matched[i]);
                addToMap(p_matched[i]);
            }

            p_matched[i]->active = true;    
            p_matched[i]->matched = true;
            matchesFound++;
            // printf("Match found.\n");
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
           [](std::shared_ptr<Matches::p_match> pm){ return pm->matched == false;}), p_matched.end());  
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
    std::vector<std::shared_ptr<Matches::p_match>> *buckets = new std::vector<std::shared_ptr<Matches::p_match>>[bucket_cols*bucket_rows*4];

    printf("umax: %f vmax: %f cols: %i rows: %i\n", u_max, v_max, bucket_cols, bucket_rows);

    // assign matches to their buckets
    for (int i=0; i < inlierMatches.size(); i++)
    {
        int32_t u = (int32_t)floor(inlierMatches[i]->u1c/bucket_width);
        int32_t v = (int32_t)floor(inlierMatches[i]->v1c/bucket_height);        
        buckets[v*bucket_cols+u+inlierMatches[i]->max1.c*bucket_cols].push_back(inlierMatches[i]);
    }

    int32_t b0, b1, b2, b3, b4;
    b0 = b1 = b2 = b3 = b4;

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

            if ((cnt0+cnt1+cnt2+cnt3) == 0)
                b0++;

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
                {
                    int val = std::min(cnt0, k) + std::min(cnt1, k) + std::min(cnt2, k) + std::min(cnt3, k);

                    if (val == 0)
                        b0++;
                    if (val == 1)
                        b1++;
                    if (val == 2)
                        b2++;
                    if (val == 3)
                        b3++;
                    if (val == 4)
                        b4++;
                    
                    break;
                }                
            }      
        }
    }

    printf("Buckets. Total: %i 0:%i 1:%i 2:%i 3:%i 4:%i accum:%i\n", bucket_cols* bucket_rows, 
            b0,b1,b2,b3,b4, b0+b1+b2+b3+b4);

    // free buckets
    delete []buckets;
}

int32_t Matches::getSelectedCount()
{
    return selectedMatches.size();
}

int32_t Matches::getKey(std::shared_ptr<Matches::p_match> match)
{
    float u1c = roundf(match->u1c);
    float v1c = roundf(match->v1c);
    return (match->max1.c * 10000000 + u1c * 10000 + v1c);
}

int32_t Matches::getPriorKey(std::shared_ptr<Matches::p_match> match)
{
    float u1p = roundf(match->u1p);
    float v1p = roundf(match->v1p);
    return (match->max1.c * 10000000 + u1p * 10000 + v1p);
}

std::vector<Matches::p_match> Matches::copySelectedMatches()
{
    std::vector<Matches::p_match> matches;

    for (int i=0; i < selectedMatches.size(); i++)
    {
        matches.push_back(*selectedMatches[i]);
    }

    return matches;
}

void Matches::printStats()
{
    int selCount = 0;
    int activeCount = 0;
    int inlierCount = 0;
    int inlierVecCount = inlierMatches.size();
    int totalCount = p_matched.size();
    int invalidMatches = 0;
    int invalidInlierMatches = 0;

    for (int i=0; i < p_matched.size(); i++)
    {
        if (p_matched[i]->active)
            activeCount++;
        if (!p_matched[i]->outlier)
            inlierCount++;

        if (p_matched[i]->u1c < 1 || p_matched[1]->u1c > 1000 || p_matched[i]->v1c < 1
            || p_matched[i]->v1c > 1000)
            invalidMatches++;
    }

    for (int i=0; i < inlierMatches.size(); i++)
    {
        if (inlierMatches[i]->u1c < 1 || inlierMatches[1]->u1c > 1000 || inlierMatches[i]->v1c < 1
            || inlierMatches[i]->v1c > 1000)
            invalidInlierMatches++;
    }

    selCount = selectedMatches.size();

    printf("Matches stats. Total: %i Invalid: %i InvalidInliers: %i Added: %i Found: %i Active: %i inliers: %i inliervec: %i selected: %i\n", 
        totalCount, invalidMatches, invalidInlierMatches, matchesAdded, matchesFound, activeCount, inlierCount, inlierVecCount, selCount);
}

void Matches::validateMatches(std::string ref)
{
    return;
    
    int badMatches = 0;

    for (int i=0; i < p_matched.size(); i++)
    {
        if (validateMatch(p_matched[i]))
            badMatches++;
    }

    int badInlierMatches = 0;

    for (int i=0; i < inlierMatches.size(); i++)
    {
        if (validateMatch(inlierMatches[i]))
            badInlierMatches++;
    }

    if (badMatches > 0 || badInlierMatches > 0)
    {
        printf("ValidateMatches. Ref: %s bad matches: %i(%i) bad inlier matches: %i(%i)\n", ref.c_str(), badMatches, 
            static_cast<int>(p_matched.size()), badInlierMatches, static_cast<int>(inlierMatches.size()));
    }
}


bool Matches::validateMatch(std::shared_ptr<Matches::p_match> match)
{
    bool bad = false;

    if (match->u1c < 0 || match->u1c > 640)
    {
        printf("bad u1c: %f\n", match->u1c);
        bad = true;
    }

    if (match->u2c < 0 || match->u2c > 640)
    {
        printf("bad u2c: %f\n", match->u2c);
        bad = true;
    }

    if (match->v1c < 0 || match->v1c > 640)
    {
        printf("bad v1c: %f\n", match->v1c);
        bad = true;
    }

    if (match->v2c < 0 || match->v2c > 640)
    {
        printf("bad v2c: %f\n", match->v2c);
        bad = true;
    }

    if (match->max1.c < 0 || match->max1.c > 4)
    {
        printf("bad m1c: %i\n", match->max1.c);
        bad = true;
    }
    
    if (match->max2.c < 0 || match->max2.c > 4)
    {
        printf("bad m2c: %i\n", match->max2.c);
        bad = true;
    }

    return bad;
}