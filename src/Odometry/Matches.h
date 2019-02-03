#pragma once
#include <vector>
#include <map>
#include <math.h>
#include <algorithm>
#include <stdio.h>
#include <cstring>

class Matches
{
    public:
        Matches();

        // structure for storing interest points
        struct maximum {
            int32_t u;   // u-coordinate
            int32_t v;   // v-coordinate
            int32_t val; // value
            int32_t c;   // class
            int32_t d1,d2,d3,d4,d5,d6,d7,d8; // descriptor
            maximum() {}
            maximum(int32_t u,int32_t v,int32_t val,int32_t c):u(u),v(v),val(val),c(c) {}
        };

        // structure for storing matches
        struct p_match {
            float   u1p2,v1p2; // u,v-coordinates in 2nd previous left  image
            float   u1p3,v1p3; // u,v-coordinates in 3rd previous left  image
            float   u1p,v1p; // u,v-coordinates in previous left  image
            int32_t i1p;     // feature index (for tracking)
            float   u2p,v2p; // u,v-coordinates in previous right image
            int32_t i2p;     // feature index (for tracking)
            float   u1c,v1c; // u,v-coordinates in current  left  image
            int32_t i1c;     // feature index (for tracking)
            float   u2c,v2c; // u,v-coordinates in current  right image
            int32_t i2c;     // feature index (for tracking)
            maximum imax1;  // initial feature left image
            maximum imax2;  // initial feature right image
            maximum max1;   // current feature left image
            maximum max2;   // current feature right image
            int32_t age;  // feature age
            int32_t noMatchCount;            
            float   depth;  // depth
            bool matched;
            bool selected;
            bool active;
            bool outlier;
            p_match(){ age = -1; matched = false; depth = 0.0; outlier=0; noMatchCount=0; selected=false; active=false; outlier=false;}
            p_match(float u1p,float v1p,int32_t i1p,float u2p,float v2p,int32_t i2p,
                    float u1c,float v1c,int32_t i1c,float u2c,float v2c,int32_t i2c):
                    u1p(u1p),v1p(v1p),i1p(i1p),u2p(u2p),v2p(v2p),i2p(i2p),
                    u1c(u1c),v1c(v1c),i1c(i1c),u2c(u2c),v2c(v2c),i2c(i2c) { age = -1; matched = false; outlier=0; noMatchCount=0;depth =0.0;selected=false;active=false;outlier==false;}
        };


        void ageFeaturePoints();
        void resetMatches();
        int32_t getActiveMatches();
        int32_t getTotalMatches();
        void setActiveFlag(bool active, Matches::p_match match);
        void setOutlierFlag(bool outlier, Matches::p_match match);
        bool push_back(Matches::p_match match);
        void clear();
        void bucketFeatures(int32_t max_features,float bucket_width,float bucket_height);

        std::vector<Matches::p_match> p_matched;

    private:
        bool matchExists(Matches::p_match match);
        int32_t getKey(Matches::p_match match);
        int32_t getPriorKey(Matches::p_match match);
        void deleteStaleMatches();

        
        std::map<int32_t, Matches::p_match*> map_matched;

        int32_t activeMatches = 0;

        static bool compareMatches(Matches::p_match p1, Matches::p_match p2)
        {
            int32_t ageDiscrim = 3;
            if (p1.age > p2.age && p1.age <= ageDiscrim)
            return true;
            
            if (p2.age > p1.age && p2.age <= ageDiscrim)
            return false;
            
            if (p1.max1.c == 0 || p1.max1.c ==2) // class 0 and 2 are minima so negative val
            {
            if (p1.max1.val < p2.max1.val)
                return true;

            if (p2.max1.val < p1.max1.val)
                return false;
            }
            else
            {
            if (p1.max1.val > p2.max1.val)
                return true;

            if (p2.max1.val > p1.max1.val)
                return false;
            }
            
            
            return (p1.i1p > p2.i1p);  
        };

};