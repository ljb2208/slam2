#include "LoopMatcher.h"
#include "Odometry/Filter.h"

using namespace std;

// constructor (with default parameters)
LoopMatcher::LoopMatcher(parameters param) : param(param) {

  // init match ring buffer to zero
  m1p1 = 0; n1p1 = 0;
  m1p2 = 0; n1p2 = 0;
  m1c1 = 0; n1c1 = 0;
  m1c2 = 0; n1c2 = 0;
  I1p    = 0; 
  I1c    = 0; 
  I1p_du = 0; I1p_dv = 0;
  I1c_du = 0; I1c_dv = 0;
  I1p_du_full = 0; I1p_dv_full = 0;
  I1c_du_full = 0; I1c_dv_full = 0;

  // margin needed to compute descriptor + sobel responses
  margin = 8+1;
  
  // adjust match radius on half resolution
  if (param.half_resolution)
    this->param.match_radius /= 2;  
}


// deconstructor
LoopMatcher::~LoopMatcher() {
  if (I1p)          _mm_free(I1p);
  if (I1c)          _mm_free(I1c);
  if (m1p1)         _mm_free(m1p1);
  if (m1p2)         _mm_free(m1p2);
  if (I1p_du)       _mm_free(I1p_du);
  if (I1p_dv)       _mm_free(I1p_dv);
  if (I1p_du_full)  _mm_free(I1p_du_full);
  if (I1p_dv_full)  _mm_free(I1p_dv_full);
  if (m1c1)         _mm_free(m1c1);
  if (m1c2)         _mm_free(m1c2);
  if (I1c_du)       _mm_free(I1c_du);
  if (I1c_dv)       _mm_free(I1c_dv);
  if (I1c_du_full)  _mm_free(I1c_du_full);
  if (I1c_dv_full)  _mm_free(I1c_dv_full);
}

bool LoopMatcher::updateMotion (slam2::Matrix* tr)
{  
  matchFeatures();
  bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);                          
  
  // estimate motion
  vector<double> tr_delta = estimateMotion(p_matched_2);
  
  // on failure
  if (tr_delta.size()!=6)
    return false;
  
  // set transformation matrix (previous to current frame)
  Tr_delta = transformationVectorToMatrix(tr_delta);
  Tr_valid = true;

  *tr = Tr_delta;
  
  // success
  return true;
}

void LoopMatcher::pushBack (uint8_t *I1,int32_t* dims,const bool replace) {

  // image dimensions
  int32_t width  = dims[0];
  int32_t height = dims[1];
  int32_t bpl    = dims[2];

  // sanity check
  if (width<=0 || height<=0 || bpl<width || I1==0) {
    cerr << "ERROR: Image dimension mismatch!" << endl;
    return;
  }

  if (replace) {
    if (I1c)         _mm_free(I1c);
    if (m1c1)        _mm_free(m1c1);
    if (m1c2)        _mm_free(m1c2);
    if (I1c_du)      _mm_free(I1c_du);
    if (I1c_dv)      _mm_free(I1c_dv);
    if (I1c_du_full) _mm_free(I1c_du_full);
    if (I1c_dv_full) _mm_free(I1c_dv_full);
  } else {
    if (I1p)         _mm_free(I1p);
    if (m1p1)        _mm_free(m1p1);
    if (m1p2)        _mm_free(m1p2);
    if (I1p_du)      _mm_free(I1p_du);
    if (I1p_dv)      _mm_free(I1p_dv);
    if (I1p_du_full) _mm_free(I1p_du_full);
    if (I1p_dv_full) _mm_free(I1p_dv_full);
    m1p1 = m1c1; n1p1 = n1c1;
    m1p2 = m1c2; n1p2 = n1c2;
    I1p         = I1c;
    I1p_du      = I1c_du;
    I1p_dv      = I1c_dv;
    I1p_du_full = I1c_du_full;
    I1p_dv_full = I1c_dv_full;
    dims_p[0]   = dims_c[0];
    dims_p[1]   = dims_c[1];
    dims_p[2]   = dims_c[2];
  }

  // set new dims (bytes per line must be multiple of 16)
  dims_c[0] = width;
  dims_c[1] = height;
  dims_c[2] = width + 15-(width-1)%16;

  // copy images to byte aligned memory
  I1c = (uint8_t*)_mm_malloc(dims_c[2]*dims_c[1]*sizeof(uint8_t),16);

  if (dims_c[2]==bpl) {
    memcpy(I1c,I1,dims_c[2]*dims_c[1]*sizeof(uint8_t));
  } else {
    for (int32_t v=0; v<height; v++) {
      memcpy(I1c+v*dims_c[2],I1+v*bpl,dims_c[0]*sizeof(uint8_t));
    }
  }

  // compute new features for current frame
  computeFeatures(I1c,dims_c,m1c1,n1c1,m1c2,n1c2,I1c_du,I1c_dv,I1c_du_full,I1c_dv_full);
}

void LoopMatcher::matchFeatures() {
  
    slam2::Matrix *delta = 0;

    if (m1p2==0 || n1p2==0 || m1c2==0 || n1c2==0)
        return;

    if (param.multi_stage)
        if (m1p1==0 || n1p1==0 || m1c1==0 || n1c1==0)
            return;

    // clear old matches
    p_matched_1.clear();
    p_matched_2.clear();

    // double pass matching
    if (param.multi_stage) {

        // 1st pass (sparse matches)
        matching(m1p1,m1c1,n1p1,n1c1,p_matched_1,false,delta);
        removeOutliers(p_matched_1);
        
        // compute search range prior statistics (used for speeding up 2nd pass)
        computePriorStatistics(p_matched_1);      

        // 2nd pass (dense matches)
        matching(m1p2,m1c2,n1p2,n1c2,p_matched_2,true,delta);
        if (param.refinement>0)
            refinement(p_matched_2);

        removeOutliers(p_matched_2);

    // single pass matching
    } else {
        matching(m1p2,m1c2,n1p2,n1c2,p_matched_2,false,delta);
        if (param.refinement>0)
            refinement(p_matched_2);

        removeOutliers(p_matched_2);
    }
}

void LoopMatcher::computeFeatures (uint8_t *I,const int32_t* dims,int32_t* &max1,int32_t &num1,int32_t* &max2,int32_t &num2,uint8_t* &I_du,uint8_t* &I_dv,uint8_t* &I_du_full,uint8_t* &I_dv_full) {
  
  int16_t *I_f1;
  int16_t *I_f2;
  
  int32_t dims_matching[3];
  memcpy(dims_matching,dims,3*sizeof(int32_t));
  
  // allocate memory for sobel images and filter images
  if (!param.half_resolution) {
    I_du = (uint8_t*)_mm_malloc(dims[2]*dims[1]*sizeof(uint8_t*),16);
    I_dv = (uint8_t*)_mm_malloc(dims[2]*dims[1]*sizeof(uint8_t*),16);
    I_f1 = (int16_t*)_mm_malloc(dims[2]*dims[1]*sizeof(int16_t),16);
    I_f2 = (int16_t*)_mm_malloc(dims[2]*dims[1]*sizeof(int16_t),16);
    filter::sobel5x5(I,I_du,I_dv,dims[2],dims[1]);
    filter::blob5x5(I,I_f1,dims[2],dims[1]);
    filter::checkerboard5x5(I,I_f2,dims[2],dims[1]);
  } else {
    uint8_t* I_matching = createHalfResolutionImage(I,dims);
    getHalfResolutionDimensions(dims,dims_matching);
    I_du      = (uint8_t*)_mm_malloc(dims_matching[2]*dims_matching[1]*sizeof(uint8_t*),16);
    I_dv      = (uint8_t*)_mm_malloc(dims_matching[2]*dims_matching[1]*sizeof(uint8_t*),16);
    I_f1      = (int16_t*)_mm_malloc(dims_matching[2]*dims_matching[1]*sizeof(int16_t),16);
    I_f2      = (int16_t*)_mm_malloc(dims_matching[2]*dims_matching[1]*sizeof(int16_t),16);
    I_du_full = (uint8_t*)_mm_malloc(dims[2]*dims[1]*sizeof(uint8_t*),16);
    I_dv_full = (uint8_t*)_mm_malloc(dims[2]*dims[1]*sizeof(uint8_t*),16);
    filter::sobel5x5(I_matching,I_du,I_dv,dims_matching[2],dims_matching[1]);
    filter::sobel5x5(I,I_du_full,I_dv_full,dims[2],dims[1]);
    filter::blob5x5(I_matching,I_f1,dims_matching[2],dims_matching[1]);
    filter::checkerboard5x5(I_matching,I_f2,dims_matching[2],dims_matching[1]);
    _mm_free(I_matching);
  }
  
  // extract sparse maxima (1st pass) via non-maximum suppression
  vector<LoopMatcher::maximum> maxima1;
  if (param.multi_stage) {
    int32_t nms_n_sparse = param.nms_n*3;
    if (nms_n_sparse>10)
      nms_n_sparse = max(param.nms_n,10);
    nonMaximumSuppression(I_f1,I_f2,dims_matching,maxima1,nms_n_sparse);
    computeDescriptors(I_du,I_dv,dims_matching[2],maxima1);
  }
  
  // extract dense maxima (2nd pass) via non-maximum suppression
  vector<LoopMatcher::maximum> maxima2;
  nonMaximumSuppression(I_f1,I_f2,dims_matching,maxima2,param.nms_n);
  computeDescriptors(I_du,I_dv,dims_matching[2],maxima2);

  // release filter images
  _mm_free(I_f1);
  _mm_free(I_f2);  
  
  // get number of interest points and init maxima pointer to NULL
  num1 = maxima1.size();
  num2 = maxima2.size();
  max1 = 0;
  max2 = 0;
  
  int32_t s = 1;
  if (param.half_resolution)
    s = 2;

  // return sparse maxima as 16-bytes aligned memory
  if (num1!=0) {
    max1 = (int32_t*)_mm_malloc(sizeof(LoopMatcher::maximum)*num1,16);
    int32_t k=0;
    for (vector<LoopMatcher::maximum>::iterator it=maxima1.begin(); it!=maxima1.end(); it++) {
      *(max1+k++) = it->u*s;  *(max1+k++) = it->v*s;  *(max1+k++) = 0;        *(max1+k++) = it->c;
      *(max1+k++) = it->d1;   *(max1+k++) = it->d2;   *(max1+k++) = it->d3;   *(max1+k++) = it->d4;
      *(max1+k++) = it->d5;   *(max1+k++) = it->d6;   *(max1+k++) = it->d7;   *(max1+k++) = it->d8;
    }
  }
  
  // return dense maxima as 16-bytes aligned memory
  if (num2!=0) {
    max2 = (int32_t*)_mm_malloc(sizeof(LoopMatcher::maximum)*num2,16);
    int32_t k=0;
    for (vector<LoopMatcher::maximum>::iterator it=maxima2.begin(); it!=maxima2.end(); it++) {
      *(max2+k++) = it->u*s;  *(max2+k++) = it->v*s;  *(max2+k++) = 0;        *(max2+k++) = it->c;
      *(max2+k++) = it->d1;   *(max2+k++) = it->d2;   *(max2+k++) = it->d3;   *(max2+k++) = it->d4;
      *(max2+k++) = it->d5;   *(max2+k++) = it->d6;   *(max2+k++) = it->d7;   *(max2+k++) = it->d8;
    }
  }
}

void LoopMatcher::computeDescriptors (uint8_t* I_du,uint8_t* I_dv,const int32_t bpl,std::vector<LoopMatcher::maximum> &maxima) {
  
  // loop variables
  int32_t u,v;
  uint8_t *desc_addr;
  
  // for all maxima do
  for (vector<LoopMatcher::maximum>::iterator it=maxima.begin(); it!=maxima.end(); it++) {
    u = (*it).u;
    v = (*it).v;
    desc_addr = (uint8_t*)(&((*it).d1));
    computeDescriptor(I_du,I_dv,bpl,u,v,desc_addr);    
  }
}

inline void LoopMatcher::computeDescriptor (const uint8_t* I_du,const uint8_t* I_dv,const int32_t &bpl,const int32_t &u,const int32_t &v,uint8_t *desc_addr) {
  
    // get address indices
  int32_t addr_m1 = getAddressOffsetImage(u,v-1,bpl);
  int32_t addr_m3 = addr_m1-2*bpl;
  int32_t addr_m5 = addr_m3-2*bpl;
  int32_t addr_p1 = addr_m1+2*bpl;
  int32_t addr_p3 = addr_p1+2*bpl;
  int32_t addr_p5 = addr_p3+2*bpl;
  
  // compute descriptor
  uint32_t k = 0;
  desc_addr[k++] = I_du[addr_m1-3];
  desc_addr[k++] = I_dv[addr_m1-3];
  desc_addr[k++] = I_du[addr_p1-3];
  desc_addr[k++] = I_dv[addr_p1-3];
  desc_addr[k++] = I_du[addr_m1-1];
  desc_addr[k++] = I_dv[addr_m1-1];
  desc_addr[k++] = I_du[addr_p1-1];
  desc_addr[k++] = I_dv[addr_p1-1];
  desc_addr[k++] = I_du[addr_m1+3];
  desc_addr[k++] = I_dv[addr_m1+3];
  desc_addr[k++] = I_du[addr_p1+3];
  desc_addr[k++] = I_dv[addr_p1+3];
  desc_addr[k++] = I_du[addr_m1+1];
  desc_addr[k++] = I_dv[addr_m1+1];
  desc_addr[k++] = I_du[addr_p1+1];
  desc_addr[k++] = I_dv[addr_p1+1];
  desc_addr[k++] = I_du[addr_m5-1];
  desc_addr[k++] = I_dv[addr_m5-1];
  desc_addr[k++] = I_du[addr_p5-1];
  desc_addr[k++] = I_dv[addr_p5-1];
  desc_addr[k++] = I_du[addr_m5+1];
  desc_addr[k++] = I_dv[addr_m5+1];
  desc_addr[k++] = I_du[addr_p5+1];
  desc_addr[k++] = I_dv[addr_p5+1];
  desc_addr[k++] = I_du[addr_m3-5];
  desc_addr[k++] = I_dv[addr_m3-5];
  desc_addr[k++] = I_du[addr_p3-5];
  desc_addr[k++] = I_dv[addr_p3-5];
  desc_addr[k++] = I_du[addr_m3+5];
  desc_addr[k++] = I_dv[addr_m3+5];
  desc_addr[k++] = I_du[addr_p3+5];
  desc_addr[k++] = I_dv[addr_p3+5];
}

void LoopMatcher::nonMaximumSuppression (int16_t* I_f1,int16_t* I_f2,const int32_t* dims,vector<LoopMatcher::maximum> &maxima,int32_t nms_n) {
  
  // extract parameters
  int32_t width  = dims[0];
  int32_t height = dims[1];
  int32_t bpl    = dims[2];
  int32_t n      = nms_n;
  int32_t tau    = param.nms_tau;
  
  // loop variables
  int32_t f1mini,f1minj,f1maxi,f1maxj,f2mini,f2minj,f2maxi,f2maxj;
  int32_t f1minval,f1maxval,f2minval,f2maxval,currval;
  int32_t addr;
  
  for (int32_t i=n+margin; i<width-n-margin;i+=n+1) {
    for (int32_t j=n+margin; j<height-n-margin;j+=n+1) {

      f1mini = i; f1minj = j; f1maxi = i; f1maxj = j;
      f2mini = i; f2minj = j; f2maxi = i; f2maxj = j;
      
      addr     = getAddressOffsetImage(i,j,bpl);
      f1minval = *(I_f1+addr);
      f1maxval = f1minval;
      f2minval = *(I_f2+addr);
      f2maxval = f2minval;

      for (int32_t i2=i; i2<=i+n; i2++) {
        for (int32_t j2=j; j2<=j+n; j2++) {
          addr    = getAddressOffsetImage(i2,j2,bpl);
          currval = *(I_f1+addr);
          if (currval<f1minval) {
            f1mini   = i2;
            f1minj   = j2;
            f1minval = currval;
          } else if (currval>f1maxval) {
            f1maxi   = i2;
            f1maxj   = j2;
            f1maxval = currval;
          }
          currval = *(I_f2+addr);
          if (currval<f2minval) {
            f2mini   = i2;
            f2minj   = j2;
            f2minval = currval;
          } else if (currval>f2maxval) {
            f2maxi   = i2;
            f2maxj   = j2;
            f2maxval = currval;
          }
        }
      }
      
      // f1 minimum
      for (int32_t i2=f1mini-n; i2<=min(f1mini+n,width-1-margin); i2++) {
        for (int32_t j2=f1minj-n; j2<=min(f1minj+n,height-1-margin); j2++) {
          currval = *(I_f1+getAddressOffsetImage(i2,j2,bpl));
          if (currval<f1minval && (i2<i || i2>i+n || j2<j || j2>j+n))
            goto failed_f1min;            
        }
      }
      if (f1minval<=-tau)
        maxima.push_back(LoopMatcher::maximum(f1mini,f1minj,f1minval,0));
      failed_f1min:;

      // f1 maximum
      for (int32_t i2=f1maxi-n; i2<=min(f1maxi+n,width-1-margin); i2++) {
        for (int32_t j2=f1maxj-n; j2<=min(f1maxj+n,height-1-margin); j2++) {
          currval = *(I_f1+getAddressOffsetImage(i2,j2,bpl));
          if (currval>f1maxval && (i2<i || i2>i+n || j2<j || j2>j+n))
            goto failed_f1max;
        }
      }
      if (f1maxval>=tau)
        maxima.push_back(LoopMatcher::maximum(f1maxi,f1maxj,f1maxval,1));
      failed_f1max:;
      
      // f2 minimum
      for (int32_t i2=f2mini-n; i2<=min(f2mini+n,width-1-margin); i2++) {
        for (int32_t j2=f2minj-n; j2<=min(f2minj+n,height-1-margin); j2++) {
          currval = *(I_f2+getAddressOffsetImage(i2,j2,bpl));
          if (currval<f2minval && (i2<i || i2>i+n || j2<j || j2>j+n))
            goto failed_f2min;
        }
      }
      if (f2minval<=-tau)
        maxima.push_back(LoopMatcher::maximum(f2mini,f2minj,f2minval,2));
      failed_f2min:;

      // f2 maximum
      for (int32_t i2=f2maxi-n; i2<=min(f2maxi+n,width-1-margin); i2++) {
        for (int32_t j2=f2maxj-n; j2<=min(f2maxj+n,height-1-margin); j2++) {
          currval = *(I_f2+getAddressOffsetImage(i2,j2,bpl));
          if (currval>f2maxval && (i2<i || i2>i+n || j2<j || j2>j+n))
            goto failed_f2max;
        }
      }
      if (f2maxval>=tau)
        maxima.push_back(LoopMatcher::maximum(f2maxi,f2maxj,f2maxval,3));
      failed_f2max:;
    }
  }
}

void LoopMatcher::getHalfResolutionDimensions(const int32_t *dims,int32_t *dims_half) {
  dims_half[0] = dims[0]/2;
  dims_half[1] = dims[1]/2;
  dims_half[2] = dims_half[0]+15-(dims_half[0]-1)%16;
}

uint8_t* LoopMatcher::createHalfResolutionImage(uint8_t *I,const int32_t* dims) {
  int32_t dims_half[3];
  getHalfResolutionDimensions(dims,dims_half);
  uint8_t* I_half = (uint8_t*)_mm_malloc(dims_half[2]*dims_half[1]*sizeof(uint8_t),16);
  for (int32_t v=0; v<dims_half[1]; v++)
    for (int32_t u=0; u<dims_half[0]; u++)
      I_half[v*dims_half[2]+u] =  (uint8_t)(((int32_t)I[(v*2+0)*dims[2]+u*2+0]+
                                             (int32_t)I[(v*2+0)*dims[2]+u*2+1]+
                                             (int32_t)I[(v*2+1)*dims[2]+u*2+0]+
                                             (int32_t)I[(v*2+1)*dims[2]+u*2+1])/4);
  return I_half;
}

void LoopMatcher::removeOutliers (vector<LoopMatcher::p_match> &p_matched) {

  // do we have enough points for outlier removal?
  if (p_matched.size()<=3)
    return;

  // input/output structure for triangulation
  struct triangulateio in, out;

  // inputs
  in.numberofpoints = p_matched.size();
  in.pointlist = (float*)malloc(in.numberofpoints*2*sizeof(float));
  int32_t k=0;
  
  // create copy of p_matched, init vector with number of support points
  // and fill triangle point vector for delaunay triangulation
  vector<LoopMatcher::p_match> p_matched_copy;  
  vector<int32_t> num_support;
  for (vector<LoopMatcher::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
    p_matched_copy.push_back(*it);
    num_support.push_back(0);
    in.pointlist[k++] = it->u1c;
    in.pointlist[k++] = it->v1c;
  }
  
  // input parameters
  in.numberofpointattributes = 0;
  in.pointattributelist      = NULL;
  in.pointmarkerlist         = NULL;
  in.numberofsegments        = 0;
  in.numberofholes           = 0;
  in.numberofregions         = 0;
  in.regionlist              = NULL;
  
  // outputs
  out.pointlist              = NULL;
  out.pointattributelist     = NULL;
  out.pointmarkerlist        = NULL;
  out.trianglelist           = NULL;
  out.triangleattributelist  = NULL;
  out.neighborlist           = NULL;
  out.segmentlist            = NULL;
  out.segmentmarkerlist      = NULL;
  out.edgelist               = NULL;
  out.edgemarkerlist         = NULL;

  // do triangulation (z=zero-based, n=neighbors, Q=quiet, B=no boundary markers)
  // attention: not using the B switch or using the n switch creates a memory leak (=> use valgrind!)
  char parameters[] = "zQB";
  triangulate(parameters, &in, &out, NULL);
  
  // for all triangles do
  for (int32_t i=0; i<out.numberoftriangles; i++) {
    
    // extract triangle corner points
    int32_t p1 = out.trianglelist[i*3+0];
    int32_t p2 = out.trianglelist[i*3+1];
    int32_t p3 = out.trianglelist[i*3+2];
    
    // 1. corner disparity and flow
    float p1_flow_u = p_matched_copy[p1].u1c-p_matched_copy[p1].u1p;
    float p1_flow_v = p_matched_copy[p1].v1c-p_matched_copy[p1].v1p;

    // 2. corner disparity and flow
    float p2_flow_u = p_matched_copy[p2].u1c-p_matched_copy[p2].u1p;
    float p2_flow_v = p_matched_copy[p2].v1c-p_matched_copy[p2].v1p;

    // 3. corner disparity and flow
    float p3_flow_u = p_matched_copy[p3].u1c-p_matched_copy[p3].u1p;
    float p3_flow_v = p_matched_copy[p3].v1c-p_matched_copy[p3].v1p;

    // consistency of 1. edge
    if (fabs(p1_flow_u-p2_flow_u)+fabs(p1_flow_v-p2_flow_v)<param.outlier_flow_tolerance) {
    num_support[p1]++;
    num_support[p2]++;
    }

    // consistency of 2. edge
    if (fabs(p2_flow_u-p3_flow_u)+fabs(p2_flow_v-p3_flow_v)<param.outlier_flow_tolerance) {
    num_support[p2]++;
    num_support[p3]++;
    }

    // consistency of 3. edge
    if (fabs(p1_flow_u-p3_flow_u)+fabs(p1_flow_v-p3_flow_v)<param.outlier_flow_tolerance) {
    num_support[p1]++;
    num_support[p3]++;
    }
     
  }
  
  // refill p_matched
  p_matched.clear();
  for (int i=0; i<in.numberofpoints; i++)
    if (num_support[i]>=4)
      p_matched.push_back(p_matched_copy[i]);
  
  // free memory used for triangulation
  free(in.pointlist);
  free(out.pointlist);
  free(out.trianglelist);
}

void LoopMatcher::refinement (vector<LoopMatcher::p_match> &p_matched) {
  
  // allocate aligned memory (32 bytes for 1 descriptors)
  uint8_t* desc_buffer = (uint8_t*)_mm_malloc(32*sizeof(uint8_t),16);
  
  // copy vector (for refill)
  vector<LoopMatcher::p_match> p_matched_copy = p_matched;
  p_matched.clear();
  
  // create matrices for least square fitting
  FLOAT A_data[9*6] = { 1, 1, 1,-1,-1, 1,
                        0, 1, 0, 0,-1, 1,
                        1, 1,-1, 1,-1, 1,
                        1, 0, 0,-1, 0, 1,
                        0, 0, 0, 0, 0, 1,
                        1, 0, 0, 1, 0, 1,
                        1, 1,-1,-1, 1, 1,
                        0, 1, 0, 0, 1, 1,
                        1, 1, 1, 1, 1, 1};
  slam2::Matrix A(9,6,A_data);
  slam2::Matrix At  = ~A;
  slam2::Matrix AtA = At*A;
  
  uint8_t* I1p_du_fit = I1p_du;
  uint8_t* I1p_dv_fit = I1p_dv;
  uint8_t* I1c_du_fit = I1c_du;
  uint8_t* I1c_dv_fit = I1c_dv;
  if (param.half_resolution) {
    I1p_du_fit = I1p_du_full;
    I1p_dv_fit = I1p_dv_full;
    I1c_du_fit = I1c_du_full;
    I1c_dv_fit = I1c_dv_full;
  }
  
  // for all matches do
  for (vector<LoopMatcher::p_match>::iterator it=p_matched_copy.begin(); it!=p_matched_copy.end(); it++) {
    
    
    if (param.refinement==2) {
        if (!parabolicFitting(I1c_du_fit,I1c_dv_fit,dims_c,I1p_du_fit,I1p_dv_fit,dims_p,
                                it->u1c,it->v1c,it->u1p,it->v1p,At,AtA,desc_buffer))
            continue;
    } else {
        relocateMinimum(I1c_du_fit,I1c_dv_fit,dims_c,I1p_du_fit,I1p_dv_fit,dims_p,
                    it->u1c,it->v1c,it->u1p,it->v1p,desc_buffer);
    }

    // add this match
    p_matched.push_back(*it);
  }
  
  // free memory
  _mm_free(desc_buffer);
}

void LoopMatcher::matching (int32_t *m1p,int32_t *m1c,int32_t n1p,int32_t n1c,
                        std::vector<LoopMatcher::p_match> &p_matched, bool use_prior,slam2::Matrix *Tr_delta) {

    // descriptor step size (number of int32_t elements in struct)
    int32_t step_size = sizeof(LoopMatcher::maximum)/sizeof(int32_t);
    
    // compute number of bins
    int32_t u_bin_num = (int32_t)ceil((float)dims_c[0]/(float)param.match_binsize);
    int32_t v_bin_num = (int32_t)ceil((float)dims_c[1]/(float)param.match_binsize);
    int32_t bin_num   = 4*v_bin_num*u_bin_num; // 4 classes
    
    // allocate memory for index vectors (needed for efficient search)
    vector<int32_t> *k1p = new vector<int32_t>[bin_num];
    vector<int32_t> *k2p = new vector<int32_t>[bin_num];
    vector<int32_t> *k1c = new vector<int32_t>[bin_num];
    vector<int32_t> *k2c = new vector<int32_t>[bin_num];
    
    // loop variables
    int32_t* M = (int32_t*)calloc(dims_c[0]*dims_c[1],sizeof(int32_t));
    int32_t i1p,i2p,i1c,i2c,i1c2,i1p2;
    int32_t u1p,v1p,u2p,v2p,u1c,v1c,u2c,v2c;
    
    double t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23;
    if (Tr_delta) {
        t00 = Tr_delta->val[0][0];
        t01 = Tr_delta->val[0][1];
        t02 = Tr_delta->val[0][2];
        t03 = Tr_delta->val[0][3];
        t10 = Tr_delta->val[1][0];
        t11 = Tr_delta->val[1][1];
        t12 = Tr_delta->val[1][2];
        t13 = Tr_delta->val[1][3];
        t20 = Tr_delta->val[2][0];
        t21 = Tr_delta->val[2][1];
        t22 = Tr_delta->val[2][2];
        t23 = Tr_delta->val[2][3];
    }

    // create position/class bin index vectors
    createIndexVector(m1p,n1p,k1p,u_bin_num,v_bin_num);
    createIndexVector(m1c,n1c,k1c,u_bin_num,v_bin_num);
    
    // for all points do
    for (i1c=0; i1c<n1c; i1c++) {

      // coordinates in previous left image
      u1c = *(m1c+step_size*i1c+0);
      v1c = *(m1c+step_size*i1c+1);

      // compute row and column of statistics bin to which this observation belongs
      int32_t u_bin = min((int32_t)floor((float)u1c/(float)param.match_binsize),u_bin_num-1);
      int32_t v_bin = min((int32_t)floor((float)v1c/(float)param.match_binsize),v_bin_num-1);
      int32_t stat_bin = v_bin*u_bin_num+u_bin;

      // match forward/backward
      findMatch(m1c,i1c,m1p,step_size,k1p,u_bin_num,v_bin_num,stat_bin,i1p, 0,true,use_prior);
      findMatch(m1p,i1p,m1c,step_size,k1c,u_bin_num,v_bin_num,stat_bin,i1c2,1,true,use_prior);

      // circle closure success?
      if (i1c2==i1c) {

        // extract coordinates
        u1p = *(m1p+step_size*i1p+0);
        v1p = *(m1p+step_size*i1p+1);

        // add match if this pixel isn't matched yet
        if (*(M+getAddressOffsetImage(u1c,v1c,dims_c[0]))==0) {
          p_matched.push_back(LoopMatcher::p_match(u1p,v1p,i1p,-1,-1,-1,u1c,v1c,i1c,-1,-1,-1));
          *(M+getAddressOffsetImage(u1c,v1c,dims_c[0])) = 1;
        }
      }
    }
    
    // free memory
    free(M);
    delete []k1p;
    delete []k1c;
}

void LoopMatcher::createIndexVector (int32_t* m,int32_t n,vector<int32_t> *k,const int32_t &u_bin_num,const int32_t &v_bin_num) {

  // descriptor step size
  int32_t step_size = sizeof(LoopMatcher::maximum)/sizeof(int32_t);
  
  // for all points do
  for (int32_t i=0; i<n; i++) {
    
    // extract coordinates and class
    int32_t u = *(m+step_size*i+0); // u-coordinate
    int32_t v = *(m+step_size*i+1); // v-coordinate
    int32_t c = *(m+step_size*i+3); // class
    
    // compute row and column of bin to which this observation belongs
    int32_t u_bin = min((int32_t)floor((float)u/(float)param.match_binsize),u_bin_num-1);
    int32_t v_bin = min((int32_t)floor((float)v/(float)param.match_binsize),v_bin_num-1);
    
    // save index
    k[(c*v_bin_num+v_bin)*u_bin_num+u_bin].push_back(i);
  }
}

inline void LoopMatcher::findMatch (int32_t* m1,const int32_t &i1,int32_t* m2,const int32_t &step_size,vector<int32_t> *k2,
                                const int32_t &u_bin_num,const int32_t &v_bin_num,const int32_t &stat_bin,
                                int32_t& min_ind,int32_t stage,bool flow,bool use_prior,double u_,double v_) {
  
  // init and load image coordinates + feature
  min_ind          = 0;
  double  min_cost = 10000000;
  int32_t u1       = *(m1+step_size*i1+0);
  int32_t v1       = *(m1+step_size*i1+1);
  int32_t c        = *(m1+step_size*i1+3);
  __m128i xmm1     = _mm_load_si128((__m128i*)(m1+step_size*i1+4));
  __m128i xmm2     = _mm_load_si128((__m128i*)(m1+step_size*i1+8));
  
  float u_min,u_max,v_min,v_max;
  
  // restrict search range with prior
  if (use_prior) {
    u_min = u1+ranges[stat_bin].u_min[stage];
    u_max = u1+ranges[stat_bin].u_max[stage];
    v_min = v1+ranges[stat_bin].v_min[stage];
    v_max = v1+ranges[stat_bin].v_max[stage];
    
  // otherwise: use full search space
  } else {
    u_min = u1-param.match_radius;
    u_max = u1+param.match_radius;
    v_min = v1-param.match_radius;
    v_max = v1+param.match_radius;
  }
  
  // if stereo search => constrain to 1d
  if (!flow) {
    v_min = v1-param.match_disp_tolerance;
    v_max = v1+param.match_disp_tolerance;
  }
  
  // bins of interest
  int32_t u_bin_min = min(max((int32_t)floor(u_min/(float)param.match_binsize),0),u_bin_num-1);
  int32_t u_bin_max = min(max((int32_t)floor(u_max/(float)param.match_binsize),0),u_bin_num-1);
  int32_t v_bin_min = min(max((int32_t)floor(v_min/(float)param.match_binsize),0),v_bin_num-1);
  int32_t v_bin_max = min(max((int32_t)floor(v_max/(float)param.match_binsize),0),v_bin_num-1);
  
  // for all bins of interest do
  for (int32_t u_bin=u_bin_min; u_bin<=u_bin_max; u_bin++) {
    for (int32_t v_bin=v_bin_min; v_bin<=v_bin_max; v_bin++) {
      int32_t k2_ind = (c*v_bin_num+v_bin)*u_bin_num+u_bin;
      for (vector<int32_t>::const_iterator i2_it=k2[k2_ind].begin(); i2_it!=k2[k2_ind].end(); i2_it++) {
        int32_t u2   = *(m2+step_size*(*i2_it)+0);
        int32_t v2   = *(m2+step_size*(*i2_it)+1);
        if (u2>=u_min && u2<=u_max && v2>=v_min && v2<=v_max) {
          __m128i xmm3 = _mm_load_si128((__m128i*)(m2+step_size*(*i2_it)+4));
          __m128i xmm4 = _mm_load_si128((__m128i*)(m2+step_size*(*i2_it)+8));                    
          xmm3 = _mm_sad_epu8 (xmm1,xmm3);
          xmm4 = _mm_sad_epu8 (xmm2,xmm4);
          xmm4 = _mm_add_epi16(xmm3,xmm4);
          double cost = (double)(_mm_extract_epi16(xmm4,0)+_mm_extract_epi16(xmm4,4));
          
          if (u_>=0 && v_>=0) {
            double du = (double)u2-u_;
            double dv = (double)v2-v_;
            double dist = sqrt(du*du+dv*dv);
            cost += 4*dist;
          }
          
          if (cost<min_cost) {
            min_ind  = *i2_it;
            min_cost = cost;
          }
        }
      }
    }
  }
}

void LoopMatcher::computePriorStatistics (vector<LoopMatcher::p_match> &p_matched) {
   
  // compute number of bins
  int32_t u_bin_num = (int32_t)ceil((float)dims_c[0]/(float)param.match_binsize);
  int32_t v_bin_num = (int32_t)ceil((float)dims_c[1]/(float)param.match_binsize);
  int32_t bin_num   = v_bin_num*u_bin_num;
  
  // number of matching stages
  int32_t num_stages = 2;
  
  // allocate bin accumulator memory
  vector<LoopMatcher::delta> *delta_accu = new vector<LoopMatcher::delta>[bin_num];
  
  // fill bin accumulator
  LoopMatcher::delta delta_curr;
  for (vector<LoopMatcher::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {

        delta_curr.val[0] = it->u1p - it->u1c;
        delta_curr.val[1] = it->v1p - it->v1c;
        delta_curr.val[2] = it->u1c - it->u1p;
        delta_curr.val[3] = it->v1c - it->v1p;
    
        // compute row and column of bin to which this observation belongs
        int32_t u_bin_min,u_bin_max,v_bin_min,v_bin_max;

        // flow + stereo: use current left image as reference
        u_bin_min = min(max((int32_t)floor(it->u1c/(float)param.match_binsize)-1,0),u_bin_num-1);
        u_bin_max = min(max((int32_t)floor(it->u1c/(float)param.match_binsize)+1,0),u_bin_num-1);
        v_bin_min = min(max((int32_t)floor(it->v1c/(float)param.match_binsize)-1,0),v_bin_num-1);
        v_bin_max = min(max((int32_t)floor(it->v1c/(float)param.match_binsize)+1,0),v_bin_num-1);
        
        // add to accumulator
        for (int32_t v_bin=v_bin_min; v_bin<=v_bin_max; v_bin++)
        for (int32_t u_bin=u_bin_min; u_bin<=u_bin_max; u_bin++)
            delta_accu[v_bin*u_bin_num+u_bin].push_back(delta_curr);
  }
  
  // clear ranges
  ranges.clear();
  
  // for all bins compute statistics
  for (int32_t v_bin=0; v_bin<v_bin_num; v_bin++) {
    for (int32_t u_bin=0; u_bin<u_bin_num; u_bin++) {
      
      // use full range in case there are no observations
      delta delta_min(-param.match_radius);
      delta delta_max(+param.match_radius);
      
      // otherwise determine delta min and delta max
      if (delta_accu[v_bin*u_bin_num+u_bin].size()>0) {
        
        // init displacements 'delta' to 'infinite'
        delta_min = delta(+1000000);
        delta_max = delta(-1000000);
        
        // find minimum and maximum displacements
        for (vector<LoopMatcher::delta>::iterator it=delta_accu[v_bin*u_bin_num+u_bin].begin();
             it!=delta_accu[v_bin*u_bin_num+u_bin].end(); it++) {
          for (int32_t i=0; i<num_stages*2; i++) {
            if (it->val[i]<delta_min.val[i]) delta_min.val[i] = it->val[i];
            if (it->val[i]>delta_max.val[i]) delta_max.val[i] = it->val[i];
          }
        }
      }
      
      // set search range for this bin
      range r;
      for (int32_t i=0; i<num_stages; i++) {
        
        // bound minimum search range to 20x20
        float delta_u = delta_max.val[i*2+0]-delta_min.val[i*2+0];
        if (delta_u<20) {
          delta_min.val[i*2+0] -= ceil((20-delta_u)/2);
          delta_max.val[i*2+0] += ceil((20-delta_u)/2);
        }
        float delta_v = delta_max.val[i*2+1]-delta_min.val[i*2+1];
        if (delta_v<20) {
          delta_min.val[i*2+1] -= ceil((20-delta_v)/2);
          delta_max.val[i*2+1] += ceil((20-delta_v)/2);
        }
        
        // set range for this bin
        r.u_min[i] = delta_min.val[i*2+0];
        r.u_max[i] = delta_max.val[i*2+0];
        r.v_min[i] = delta_min.val[i*2+1];
        r.v_max[i] = delta_max.val[i*2+1];
      }
      ranges.push_back(r);      
    }
  }
  
  // free bin accumulator memory
  delete []delta_accu;
}

bool LoopMatcher::parabolicFitting(const uint8_t* I1_du,const uint8_t* I1_dv,const int32_t* dims1,
                               const uint8_t* I2_du,const uint8_t* I2_dv,const int32_t* dims2,
                               const float &u1,const float &v1,
                               float       &u2,float       &v2,
                               slam2::Matrix At, slam2::Matrix AtA,
                               uint8_t* desc_buffer) {

  // check if parabolic fitting is feasible (descriptors are within margin)
  if (u2-3<margin || u2+3>dims2[0]-1-margin || v2-3<margin || v2+3>dims2[1]-1-margin)
    return false;
  
  // compute reference descriptor
  __m128i xmm1,xmm2;
  computeSmallDescriptor(I1_du,I1_dv,dims1[2],(int32_t)u1,(int32_t)v1,desc_buffer);
  xmm1 = _mm_load_si128((__m128i*)(desc_buffer));
  
  // compute cost matrix
  int32_t cost[49];
  for (int32_t dv=0; dv<7; dv++) {
    for (int32_t du=0; du<7; du++) {
      computeSmallDescriptor(I2_du,I2_dv,dims2[2],(int32_t)u2+du-3,(int32_t)v2+dv-3,desc_buffer);
      xmm2 = _mm_load_si128((__m128i*)(desc_buffer));
      xmm2 = _mm_sad_epu8(xmm1,xmm2);
      cost[dv*7+du] = _mm_extract_epi16(xmm2,0)+_mm_extract_epi16(xmm2,4);
    }
  }
  
  // compute minimum
  int32_t min_ind  = 0;
  int32_t min_cost = cost[0];
  for (int32_t i=1; i<49; i++) {
    if (cost[i]<min_cost) {
      min_ind   = i;
      min_cost  = cost[i];
    }
  }
  
  // get indices
  int32_t du = min_ind%7;
  int32_t dv = min_ind/7;
  
  // if minimum is at borders => remove this match
  if (du==0 || du==6 || dv==0 || dv==6)
    return false;
  
  // solve least squares system
  slam2::Matrix c(9,1);
  for (int32_t i=-1; i<=+1; i++) {
    for (int32_t j=-1; j<=+1; j++) {
      int32_t cost_curr = cost[(dv+i)*7+(du+j)];
      // if (i!=0 && j!=0 && cost_curr<=min_cost+150)
        // return false;
      c.val[(i+1)*3+(j+1)][0] = cost_curr;
    }
  }
  slam2::Matrix b = At*c;
  if (!b.solve(AtA))
    return false;
  
  // extract relative coordinates
  float divisor = (b.val[2][0]*b.val[2][0]-4.0*b.val[0][0]*b.val[1][0]);
  if (fabs(divisor)<1e-8 || fabs(b.val[2][0])<1e-8)
    return false;
  float ddv = (2.0*b.val[0][0]*b.val[4][0]-b.val[2][0]*b.val[3][0])/divisor;
  float ddu = -(b.val[4][0]+2.0*b.val[1][0]*ddv)/b.val[2][0];
  if (fabs(ddu)>=1.0 || fabs(ddv)>=1.0)
    return false;
  
  // update target
  u2 += (float)du-3.0+ddu;
  v2 += (float)dv-3.0+ddv;
  
  // return true on success
  return true;
}

void LoopMatcher::relocateMinimum(const uint8_t* I1_du,const uint8_t* I1_dv,const int32_t* dims1,
                              const uint8_t* I2_du,const uint8_t* I2_dv,const int32_t* dims2,
                              const float &u1,const float &v1,
                              float       &u2,float       &v2,
                              uint8_t* desc_buffer) {

  // check if parabolic fitting is feasible (descriptors are within margin)
  if (u2-2<margin || u2+2>dims2[0]-1-margin || v2-2<margin || v2+2>dims2[1]-1-margin)
    return;
  
  // compute reference descriptor
  __m128i xmm1,xmm2;
  computeSmallDescriptor(I1_du,I1_dv,dims1[2],(int32_t)u1,(int32_t)v1,desc_buffer);
  xmm1 = _mm_load_si128((__m128i*)(desc_buffer));
  
  // compute cost matrix
  int32_t cost[25];
  for (int32_t dv=0; dv<5; dv++) {
    for (int32_t du=0; du<5; du++) {
      computeSmallDescriptor(I2_du,I2_dv,dims2[2],(int32_t)u2+du-2,(int32_t)v2+dv-2,desc_buffer);
      xmm2 = _mm_load_si128((__m128i*)(desc_buffer));
      xmm2 = _mm_sad_epu8(xmm1,xmm2);
      cost[dv*5+du] = _mm_extract_epi16(xmm2,0)+_mm_extract_epi16(xmm2,4);
    }
  }
  
  // compute minimum
  int32_t min_ind  = 0;
  int32_t min_cost = cost[0];
  for (int32_t i=1; i<25; i++) {
    if (cost[i]<min_cost) {
      min_ind   = i;
      min_cost  = cost[i];
    }
  }
  
  // update target
  u2 += (float)(min_ind%5)-2.0;
  v2 += (float)(min_ind/5)-2.0;
}

inline void LoopMatcher::computeSmallDescriptor (const uint8_t* I_du,const uint8_t* I_dv,const int32_t &bpl,const int32_t &u,const int32_t &v,uint8_t *desc_addr) {
  
  // get address indices
  int32_t addr2 = getAddressOffsetImage(u,v,bpl);
  int32_t addr1 = addr2-bpl;
  int32_t addr0 = addr1-bpl;
  int32_t addr3 = addr2+bpl;
  int32_t addr4 = addr3+bpl;
  
  // compute ELAS-descriptor
  uint32_t k = 0;
  desc_addr[k++] = I_du[addr0];
  desc_addr[k++] = I_du[addr1-2];
  desc_addr[k++] = I_du[addr1];
  desc_addr[k++] = I_du[addr1+2];
  desc_addr[k++] = I_du[addr2-1];
  desc_addr[k++] = I_du[addr2];
  desc_addr[k++] = I_du[addr2];
  desc_addr[k++] = I_du[addr2+1];
  desc_addr[k++] = I_du[addr3-2];
  desc_addr[k++] = I_du[addr3];
  desc_addr[k++] = I_du[addr3+2];
  desc_addr[k++] = I_du[addr4];
  desc_addr[k++] = I_dv[addr1];
  desc_addr[k++] = I_dv[addr2-1];
  desc_addr[k++] = I_dv[addr2+1];
  desc_addr[k++] = I_dv[addr3];
}

void LoopMatcher::bucketFeatures(int32_t max_features,float bucket_width,float bucket_height) {

  // find max values
  float u_max = 0;
  float v_max = 0;
  for (vector<p_match>::iterator it = p_matched_2.begin(); it!=p_matched_2.end(); it++) {
    if (it->u1c>u_max) u_max=it->u1c;
    if (it->v1c>v_max) v_max=it->v1c;
  }

  // allocate number of buckets needed
  int32_t bucket_cols = (int32_t)floor(u_max/bucket_width)+1;
  int32_t bucket_rows = (int32_t)floor(v_max/bucket_height)+1;
  vector<p_match> *buckets = new vector<p_match>[bucket_cols*bucket_rows];

  // assign matches to their buckets
  for (vector<p_match>::iterator it=p_matched_2.begin(); it!=p_matched_2.end(); it++) {
    int32_t u = (int32_t)floor(it->u1c/bucket_width);
    int32_t v = (int32_t)floor(it->v1c/bucket_height);
    buckets[v*bucket_cols+u].push_back(*it);
  }
  
  // refill p_matched from buckets
  p_matched_2.clear();
  for (int32_t i=0; i<bucket_cols*bucket_rows; i++) {
    
    // shuffle bucket indices randomly
    std::random_shuffle(buckets[i].begin(),buckets[i].end());
    
    // add up to max_features features from this bucket to p_matched
    int32_t k=0;
    for (vector<p_match>::iterator it=buckets[i].begin(); it!=buckets[i].end(); it++) {
      p_matched_2.push_back(*it);
      k++;
      if (k>=max_features)
        break;
    }
  }

  // free buckets
  delete []buckets;
}


vector<double> LoopMatcher::estimateMotion (vector<LoopMatcher::p_match> p_matched) {

  // get number of matches
  int32_t N = p_matched.size();
  if (N<10)
    return vector<double>();
   
  // create calibration matrix
  double K_data[9] = {param.f,0,param.cu,0,param.f,param.cv,0,0,1};
  slam2::Matrix K(3,3,K_data);
    
  // normalize feature points and return on errors
  slam2::Matrix Tp,Tc;
  vector<LoopMatcher::p_match> p_matched_normalized = p_matched;
  if (!normalizeFeaturePoints(p_matched_normalized,Tp,Tc))
    return vector<double>();

  // initial RANSAC estimate of F
  slam2::Matrix E,F;
  inliers.clear();
  for (int32_t k=0;k<param.ransac_iters;k++) {

    // draw random sample set
    vector<int32_t> active = getRandomSample(N,8);

    // estimate fundamental matrix and get inliers
    fundamentalMatrix(p_matched_normalized,active,F);
    vector<int32_t> inliers_curr = getInlier(p_matched_normalized,F);

    // update model if we are better
    if (inliers_curr.size()>inliers.size())
      inliers = inliers_curr;
  }
  
  // are there enough inliers?
  if (inliers.size()<10)
    return vector<double>();
  
  // refine F using all inliers
  fundamentalMatrix(p_matched_normalized,inliers,F); 
  
  // denormalise and extract essential matrix
  F = ~Tc*F*Tp;
  E = ~K*F*K;
  
  // re-enforce rank 2 constraint on essential matrix
  slam2::Matrix U,W,V;
  E.svd(U,W,V);
  W.val[2][0] = 0;
  E = U*slam2::Matrix::diag(W)*~V;
  
  // compute 3d points X and R|t up to scale
  slam2::Matrix X,R,t;
  EtoRt(E,K,p_matched,X,R,t);
  
  // normalize 3d points and remove points behind image plane
  X = X/X.getMat(3,0,3,-1);
  vector<int32_t> pos_idx;
  for (int32_t i=0; i<X.n; i++)
    if (X.val[2][i]>0)
      pos_idx.push_back(i);
  slam2::Matrix X_plane = X.extractCols(pos_idx);
  
  // we need at least 10 points to proceed
  if (X_plane.n<10)
    return vector<double>();
  
  // get elements closer than median
  double median;
  smallerThanMedian(X_plane,median);
  
  // return error on large median (litte motion)
  if (median>param.motion_threshold)
    return vector<double>();
  
  // project features to 2d
  slam2::Matrix x_plane(2,X_plane.n);
  x_plane.setMat(X_plane.getMat(1,0,2,-1),0,0);
  
  slam2::Matrix n(2,1);
  n.val[0][0]       = cos(-param.pitch);
  n.val[1][0]       = sin(-param.pitch);
  slam2::Matrix   d        = ~n*x_plane;
  double   sigma    = median/50.0;
  double   weight   = 1.0/(2.0*sigma*sigma);
  double   best_sum = 0;
  int32_t  best_idx = 0;

  // find best plane
  for (int32_t i=0; i<x_plane.n; i++) {
    if (d.val[0][i]>median/param.motion_threshold) {
      double sum = 0;
      for (int32_t j=0; j<x_plane.n; j++) {
        double dist = d.val[0][j]-d.val[0][i];
        sum += exp(-dist*dist*weight);
      }
      if (sum>best_sum) {
        best_sum = sum;
        best_idx = i;
      }
    }
  }
  t = t*param.height/d.val[0][best_idx];
  
  // compute rotation angles
  double ry = asin(R.val[0][2]);
  double rx = asin(-R.val[1][2]/cos(ry));
  double rz = asin(-R.val[0][1]/cos(ry));
  
  // return parameter vector
  vector<double> tr_delta;
  tr_delta.resize(6);
  tr_delta[0] = rx;
  tr_delta[1] = ry;
  tr_delta[2] = rz;
  tr_delta[3] = t.val[0][0];
  tr_delta[4] = t.val[1][0];
  tr_delta[5] = t.val[2][0];
  return tr_delta;
}

slam2::Matrix LoopMatcher::smallerThanMedian (slam2::Matrix &X,double &median) {
  
  // set distance and index vector
  vector<double> dist;
  vector<int32_t> idx;
  for (int32_t i=0; i<X.n; i++) {
    dist.push_back(fabs(X.val[0][i])+fabs(X.val[1][i])+fabs(X.val[2][i]));
    idx.push_back(i);
  }
  
  // sort elements
  sort(idx.begin(),idx.end(),idx_cmp<vector<double>&>(dist));
  
  // get median
  int32_t num_elem_half = idx.size()/2;
  median = dist[idx[num_elem_half]];
  
  // create matrix containing elements closer than median
  slam2::Matrix X_small(4,num_elem_half+1);
  for (int32_t j=0; j<=num_elem_half; j++)
    for (int32_t i=0; i<4; i++)
      X_small.val[i][j] = X.val[i][idx[j]];
	return X_small;
}

bool LoopMatcher::normalizeFeaturePoints(vector<LoopMatcher::p_match> &p_matched,slam2::Matrix &Tp,slam2::Matrix &Tc) {
  
  // shift origins to centroids
  double cpu=0,cpv=0,ccu=0,ccv=0;
  for (vector<LoopMatcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    cpu += it->u1p;
    cpv += it->v1p;
    ccu += it->u1c;
    ccv += it->v1c;
  }
  cpu /= (double)p_matched.size();
  cpv /= (double)p_matched.size();
  ccu /= (double)p_matched.size();
  ccv /= (double)p_matched.size();
  for (vector<LoopMatcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    it->u1p -= cpu;
    it->v1p -= cpv;
    it->u1c -= ccu;
    it->v1c -= ccv;
  }
  
  // scale features such that mean distance from origin is sqrt(2)
  double sp=0,sc=0;
  for (vector<LoopMatcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    sp += sqrt(it->u1p*it->u1p+it->v1p*it->v1p);
    sc += sqrt(it->u1c*it->u1c+it->v1c*it->v1c);
  }
  if (fabs(sp)<1e-10 || fabs(sc)<1e-10)
    return false;
  sp = sqrt(2.0)*(double)p_matched.size()/sp;
  sc = sqrt(2.0)*(double)p_matched.size()/sc;
  for (vector<LoopMatcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    it->u1p *= sp;
    it->v1p *= sp;
    it->u1c *= sc;
    it->v1c *= sc;
  }
  
  // compute corresponding transformation matrices
  double Tp_data[9] = {sp,0,-sp*cpu,0,sp,-sp*cpv,0,0,1};
  double Tc_data[9] = {sc,0,-sc*ccu,0,sc,-sc*ccv,0,0,1};
  Tp = slam2::Matrix(3,3,Tp_data);
  Tc = slam2::Matrix(3,3,Tc_data);
  
  // return true on success
  return true;
}

void LoopMatcher::fundamentalMatrix (const vector<LoopMatcher::p_match> &p_matched,const vector<int32_t> &active,slam2::Matrix &F) {
  
  // number of active p_matched
  int32_t N = active.size();
  
  // create constraint matrix A
  slam2::Matrix A(N,9);
  for (int32_t i=0; i<N; i++) {
    LoopMatcher::p_match m = p_matched[active[i]];
    A.val[i][0] = m.u1c*m.u1p;
    A.val[i][1] = m.u1c*m.v1p;
    A.val[i][2] = m.u1c;
    A.val[i][3] = m.v1c*m.u1p;
    A.val[i][4] = m.v1c*m.v1p;
    A.val[i][5] = m.v1c;
    A.val[i][6] = m.u1p;
    A.val[i][7] = m.v1p;
    A.val[i][8] = 1;
  }
   
  // compute singular value decomposition of A
  slam2::Matrix U,W,V;
  A.svd(U,W,V);
   
  // extract fundamental matrix from the column of V corresponding to the smallest singular value
  F = slam2::Matrix::reshape(V.getMat(0,8,8,8),3,3);
  
  // enforce rank 2
  F.svd(U,W,V);
  W.val[2][0] = 0;
  F = U*slam2::Matrix::diag(W)*~V;
}

vector<int32_t> LoopMatcher::getInlier (vector<LoopMatcher::p_match> &p_matched,slam2::Matrix &F) {

  // extract fundamental matrix
  double f00 = F.val[0][0]; double f01 = F.val[0][1]; double f02 = F.val[0][2];
  double f10 = F.val[1][0]; double f11 = F.val[1][1]; double f12 = F.val[1][2];
  double f20 = F.val[2][0]; double f21 = F.val[2][1]; double f22 = F.val[2][2];
  
  // loop variables
  double u1,v1,u2,v2;
  double x2tFx1;
  double Fx1u,Fx1v,Fx1w;
  double Ftx2u,Ftx2v;
  
  // vector with inliers
  vector<int32_t> inliers;
  
  // for all matches do
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++) {

    // extract matches
    u1 = p_matched[i].u1p;
    v1 = p_matched[i].v1p;
    u2 = p_matched[i].u1c;
    v2 = p_matched[i].v1c;
    
    // F*x1
    Fx1u = f00*u1+f01*v1+f02;
    Fx1v = f10*u1+f11*v1+f12;
    Fx1w = f20*u1+f21*v1+f22;
    
    // F'*x2
    Ftx2u = f00*u2+f10*v2+f20;
    Ftx2v = f01*u2+f11*v2+f21;
    
    // x2'*F*x1
    x2tFx1 = u2*Fx1u+v2*Fx1v+Fx1w;
    
    // sampson distance
    double d = x2tFx1*x2tFx1 / (Fx1u*Fx1u+Fx1v*Fx1v+Ftx2u*Ftx2u+Ftx2v*Ftx2v);
    
    // check threshold
    if (fabs(d)<param.inlier_threshold)
      inliers.push_back(i);
  }

  // return set of all inliers
  return inliers;
}

void LoopMatcher::EtoRt(slam2::Matrix &E,slam2::Matrix &K,vector<LoopMatcher::p_match> &p_matched,slam2::Matrix &X,slam2::Matrix &R,slam2::Matrix &t) {

  // hartley matrices
  double W_data[9] = {0,-1,0,+1,0,0,0,0,1};
  double Z_data[9] = {0,+1,0,-1,0,0,0,0,0};
  slam2::Matrix W(3,3,W_data);
  slam2::Matrix Z(3,3,Z_data); 
  
  // extract T,R1,R2 (8 solutions)
  slam2::Matrix U,S,V;
  E.svd(U,S,V);
  slam2::Matrix T  = U*Z*~U;
  slam2::Matrix Ra = U*W*(~V);
  slam2::Matrix Rb = U*(~W)*(~V);
  
  // convert T to t
  t = slam2::Matrix(3,1);
  t.val[0][0] = T.val[2][1];
  t.val[1][0] = T.val[0][2];
  t.val[2][0] = T.val[1][0];
  
  // assure determinant to be positive
  if (Ra.det()<0) Ra = -Ra;
  if (Rb.det()<0) Rb = -Rb;
  
  // create vector containing all 4 solutions
  vector<slam2::Matrix> R_vec;
  vector<slam2::Matrix> t_vec;
  R_vec.push_back(Ra); t_vec.push_back( t);
  R_vec.push_back(Ra); t_vec.push_back(-t);
  R_vec.push_back(Rb); t_vec.push_back( t);
  R_vec.push_back(Rb); t_vec.push_back(-t);
  
  // try all 4 solutions
  slam2::Matrix X_curr;
  int32_t max_inliers = 0;
  for (int32_t i=0; i<4; i++) {
    int32_t num_inliers = triangulateChieral(p_matched,K,R_vec[i],t_vec[i],X_curr);
    if (num_inliers>max_inliers) {
      max_inliers = num_inliers;
      X = X_curr;
      R = R_vec[i];
      t = t_vec[i];
    }
  }
}

int32_t LoopMatcher::triangulateChieral (vector<LoopMatcher::p_match> &p_matched,slam2::Matrix &K,slam2::Matrix &R,slam2::Matrix &t,slam2::Matrix &X) {
  
  // init 3d point matrix
  X = slam2::Matrix(4,p_matched.size());
  
  // projection matrices
  slam2::Matrix P1(3,4);
  slam2::Matrix P2(3,4);
  P1.setMat(K,0,0);
  P2.setMat(R,0,0);
  P2.setMat(t,0,3);
  P2 = K*P2;
  
  // triangulation via orthogonal regression
  slam2::Matrix J(4,4);
  slam2::Matrix U,S,V;
  for (int32_t i=0; i<(int)p_matched.size(); i++) {
    for (int32_t j=0; j<4; j++) {
      J.val[0][j] = P1.val[2][j]*p_matched[i].u1p - P1.val[0][j];
      J.val[1][j] = P1.val[2][j]*p_matched[i].v1p - P1.val[1][j];
      J.val[2][j] = P2.val[2][j]*p_matched[i].u1c - P2.val[0][j];
      J.val[3][j] = P2.val[2][j]*p_matched[i].v1c - P2.val[1][j];
    }
    J.svd(U,S,V);
    X.setMat(V.getMat(0,3,3,3),0,i);
  }
  
  // compute inliers
  slam2::Matrix  AX1 = P1*X;
  slam2::Matrix  BX1 = P2*X;
  int32_t num = 0;
  for (int32_t i=0; i<X.n; i++)
    if (AX1.val[2][i]*X.val[3][i]>0 && BX1.val[2][i]*X.val[3][i]>0)
      num++;
  
  // return number of inliers
  return num;
}

slam2::Matrix LoopMatcher::transformationVectorToMatrix (vector<double> tr) {

  // extract parameters
  double rx = tr[0];
  double ry = tr[1];
  double rz = tr[2];
  double tx = tr[3];
  double ty = tr[4];
  double tz = tr[5];

  // precompute sine/cosine
  double sx = sin(rx);
  double cx = cos(rx);
  double sy = sin(ry);
  double cy = cos(ry);
  double sz = sin(rz);
  double cz = cos(rz);

  // compute transformation
  slam2::Matrix Tr(4,4);
  Tr.val[0][0] = +cy*cz;          Tr.val[0][1] = -cy*sz;          Tr.val[0][2] = +sy;    Tr.val[0][3] = tx;
  Tr.val[1][0] = +sx*sy*cz+cx*sz; Tr.val[1][1] = -sx*sy*sz+cx*cz; Tr.val[1][2] = -sx*cy; Tr.val[1][3] = ty;
  Tr.val[2][0] = -cx*sy*cz+sx*sz; Tr.val[2][1] = +cx*sy*sz+sx*cz; Tr.val[2][2] = +cx*cy; Tr.val[2][3] = tz;
  Tr.val[3][0] = 0;               Tr.val[3][1] = 0;               Tr.val[3][2] = 0;      Tr.val[3][3] = 1;
  return Tr;
}

vector<int32_t> LoopMatcher::getRandomSample(int32_t N,int32_t num) {

  // init sample and totalset
  vector<int32_t> sample;
  vector<int32_t> totalset;
  
  // create vector containing all indices
  for (int32_t i=0; i<N; i++)
    totalset.push_back(i);

  // add num indices to current sample
  sample.clear();
  for (int32_t i=0; i<num; i++) {
    int32_t j = rand()%totalset.size();
    sample.push_back(totalset[j]);
    totalset.erase(totalset.begin()+j);
  }
  
  // return sample
  return sample;
}


int LoopMatcher::getNumberOfMatches()
{
  return (int) p_matched_2.size();
}


int LoopMatcher::getNumberOfInliers()
{
  return (int) inliers.size();
}

