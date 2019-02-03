# include "Matches.h"

Matches::Matches()
{
    activeMatches = 0;
}

void Matches::ageFeaturePoints()
{
    for (int i=0; i < p_matched.size(); i++)
    {
        p_matched[i].u1p3 = p_matched[i].u1p2;
        p_matched[i].v1p3 = p_matched[i].v1p2;
        p_matched[i].u1p2 = p_matched[i].u1p;
        p_matched[i].v1p2 = p_matched[i].v1p;
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

    activeMatches = 0;
    inliers = 0;
}

bool Matches::includeMatch(Matches::p_match* match)
{
    if (!match->active)
        return false;

    if (match->outlier)
        return false;

    if (!match->matched)
        return false;

    return true;
}

void Matches::setActiveFlag(bool active, Matches::p_match match)
{
    // find match object
    Matches::p_match* pm = map_matched[getKey(match)];

    if (pm->active != active)
    {
        if (active)
            activeMatches++;
        else
            activeMatches--;
    }

    pm->active = active;
}

void Matches::setOutlierFlag(bool outlier, Matches::p_match match)
{
    // find match object
    Matches::p_match* pm = map_matched[getKey(match)];

    if (pm->outlier != outlier)
    {
        if (!outlier)
            inliers++;
    }

    //if (pm != NULL)
    pm->outlier = outlier;
}

int32_t Matches::getInlierCount()
{
    return inliers;
}


int32_t Matches::getActiveMatches()
{
    return activeMatches;
}

int32_t Matches::getTotalMatches()
{
    return p_matched.size();
}

bool Matches::push_back(Matches::p_match match)
{
    // if match not found add
    if (!matchExists(match))
    {
        Matches::p_match* mtch = (Matches::p_match*) malloc(sizeof(Matches::p_match));

        std::memcpy(mtch, &match, sizeof(Matches::p_match));

        mtch->matched = true;
        mtch->active = true;
        mtch->outlier = false;
        activeMatches++;
        p_matched.push_back(*mtch);
        map_matched[getKey(*mtch)] = mtch;
    }    
}

bool Matches::matchExists(Matches::p_match match)
{
    int32_t priorKey = getPriorKey(match);
    if (map_matched.find(priorKey) == map_matched.end())
        return false;

    Matches::p_match* prior_match = map_matched[priorKey];
    prior_match->matched = true;

    if (!prior_match->active)
        activeMatches++;
        
    prior_match->active = true;    
    prior_match->outlier = false;
    prior_match->u1p = match.u1p;
    prior_match->v1p = match.v1p;
    prior_match->i1p = match.i1p;
    prior_match->u2p = match.u2p;
    prior_match->v2p = match.v2p;
    prior_match->i2p = match.i2p;
    prior_match->u1c = match.u1c;
    prior_match->v1c = match.v1c;
    prior_match->i1c = match.i1c;
    prior_match->u2c = match.u2c;
    prior_match->v2c = match.v2c;
    prior_match->i2c = match.i2c;
    prior_match->max1 = match.max1;
    prior_match->max2 = match.max2;        
    return true;
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

    for (auto it = map_matched.begin(); it != map_matched.end();)
    {
        if (it->second->matched == false)
        {
            it = map_matched.erase(it);
        }
        else    
            ++it;
    }

    // remove where no current matches
    p_matched.erase(std::remove_if(p_matched.begin(), p_matched.end(),
           [](p_match pm){ return pm.matched == false;}), p_matched.end());  
}

void Matches::bucketFeatures(int32_t max_features,float bucket_width,float bucket_height)
{
    // find max values
    float u_max = 0;
    float v_max = 0;
    for (std::vector<Matches::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
        if (!it->matched || !it->active || it->outlier)
            continue;

        if (it->u1c>u_max) u_max=it->u1c;
        if (it->v1c>v_max) v_max=it->v1c;
    }

    // allocate number of buckets needed
    int32_t bucket_cols = (int32_t)floor(u_max/bucket_width)+1;
    int32_t bucket_rows = (int32_t)floor(v_max/bucket_height)+1;
    std::vector<Matches::p_match> *buckets = new std::vector<Matches::p_match>[bucket_cols*bucket_rows*4];

    printf("umax: %f vmax: %f bucket cols: %i rows: %i\n", u_max, v_max, bucket_cols, bucket_rows);

    // assign matches to their buckets
    for (std::vector<Matches::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {    
        if (!it->matched || !it->active || it->outlier)
            continue;

        int32_t u = (int32_t)floor(it->u1c/bucket_width);
        int32_t v = (int32_t)floor(it->v1c/bucket_height);        
        buckets[v*bucket_cols+u+it->max1.c*bucket_cols].push_back(*it);
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
                   buckets[ind].at(k).selected = true;

                if (cnt1 > k)
                    buckets[ind + 1*bucket_cols].at(k).selected = true;

                if (cnt2 > k)
                    buckets[ind + 2*bucket_cols].at(k).selected = true;
                
                if (cnt3 > k)
                    buckets[ind + 3*bucket_cols].at(k).selected = true;

                k++;

                if (k >= (max_features/2))
                    break;                
            }      
        }
    }

    // free buckets
    delete []buckets;
}

int32_t Matches::getKey(Matches::p_match match)
{
    float u1c = roundf(match.u1c);
    float v1c = roundf(match.v1c);
    return (match.max1.c * 1000000 + u1c * 10000 + v1c);
}

int32_t Matches::getPriorKey(Matches::p_match match)
{
    float u1p = roundf(match.u1p);
    float v1p = roundf(match.v1p);
    return (match.max1.c * 1000000 + u1p * 10000 + v1p);
}