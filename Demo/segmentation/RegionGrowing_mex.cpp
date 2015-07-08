// ========================================================================
// ***
// *** RegionGrowing_mex.cpp
// ***
// *** A fast 2D/3D region growing algorithm for Matlab. See the Matlab
// *** wrapper file 'RegionGrowing.m' for details on its usage.
// ***
// *** Compile this file by making the directiory containing this file
// *** your current Matlab working directory and typing 
// ***
// *** >> mex RegionGrowing_mex.cpp
// ***
// *** in the Matlab console.
// ***
// *** Copyright 2013 Christian Wuerslin, University of Tuebingen and
// *** University of Stuttgart, Germany.
// *** Contact: christian.wuerslin@med.uni-tuebingen.de
// ***
// ========================================================================

#include "mex.h"
#include <queue>
#include <cmath>

#define UPDATECYCLE3D     50000
#define UPDATECYCLE2D      1000
#define NQUEUES             100	// number of FIFO queues

using namespace std;

long             lNZ, lNX, lNY;     // The image dimensions;
double           dMaxDif, dRegMean;
double          *pdImg;             // pointer to the image
queue<long>     *aqQueue;			// vector of FIFO queues

// ========================================================================
// Inline function to determin minimum of two numbers
inline double ifMin(double a, double b)
{
    return a < b ? a : b;
}
// ========================================================================



// ========================================================================
// Inline function to determin maximum of two numbers
inline double ifMax(double a, double b)
{
    return a > b ? a : b;
}
// ========================================================================



// ========================================================================
// ***
// *** FUNCTION fPop
// ***
// *** Function that pops a voxel location from the highest priority
// *** non-empty queue which fulfills the region growing criterion.
// ***
// ========================================================================
long fPop() {
	long lInd; // Index of the voxel
    
    // --------------------------------------------------------------------
    // Loop over the queues, start with highest priority (0)
    for (int iQueueInd = 0; iQueueInd < NQUEUES; iQueueInd++) {
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // While there are still entries in the queue, pop and determine
        // whether it fullfills the region growing criterion.
        while (!aqQueue[iQueueInd].empty()) {
            lInd = aqQueue[iQueueInd].front();aqQueue[iQueueInd].pop();
            if (fabs(dRegMean - pdImg[lInd]) < dMaxDif) return lInd;// Return if valid entry found
        }
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    }
    // --------------------------------------------------------------------
    
	return -1; // if all queues are empty
}
// ========================================================================
// *** END OF FUNCTION fPop
// ========================================================================



// ========================================================================
// ***
// *** FUNCTION fGetNHood
// ***
// *** Get the 4-/6-neighbourhood of voxel lLinInd
// ***
// ========================================================================
void fGetNHood(const long lLinInd, long& lNHoodSize, long* lNHood) {
	long lX, lY, lZ, lTemp;
    
    lNHoodSize = 0;
    
    lY = lLinInd % lNY; // get y coordinate, add +/-1 if in image range
	if (lY >       0) lNHood[lNHoodSize++] = lLinInd - 1;
	if (lY < lNY - 1) lNHood[lNHoodSize++] = lLinInd + 1;
    
	lTemp = lLinInd/lNY; // That is a floor() operation in c
	lX = lTemp % lNX; // get x coordinate, add +/-1 if in image range. X increment is lNY
	if (lX >       0) lNHood[lNHoodSize++] = lLinInd - lNY;
	if (lX < lNX - 1) lNHood[lNHoodSize++] = lLinInd + lNY;
    
    if (lNZ > 1) { // 3D case
        lZ = lTemp/lNX; // z coordinate, add +/-1 if in image range. Z increment is lNX*lNY
        if (lZ >       0) lNHood[lNHoodSize++] = lLinInd - lNX*lNY;
        if (lZ < lNZ - 1) lNHood[lNHoodSize++] = lLinInd + lNX*lNY;
    }
}
// ========================================================================
// *** END OF FUNCTION fGetNHood
// ========================================================================



// ========================================================================
// ***
// *** FUNCTION fGetMinMax
// ***
// *** Get the minimum and maximum value of an array
// ***
// ========================================================================
void fGetMinMax(double *pdArray, long lLength, double &dMin, double &dMax)
{
    dMax   = 0.0;
    dMin   = double(1e15);

    for (long lI = 0; lI < lLength; lI++) {
        dMin = ifMin(dMin, pdArray[lI]);
        dMax = ifMax(dMax, pdArray[lI]);
    }
}
// ========================================================================
// *** END OF FUNCTION fFindMinDist
// ========================================================================



// ========================================================================
// ***
// *** MAIN MEX FUNCTION RegionGrowing_mex
// ***
// *** See m-file for description
// ***
// ========================================================================
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
    // --------------------------------------------------------------------
    // Check the number of the input and output arguments.
    if(nrhs < 3)  mexErrMsgTxt("At least 3 input arguments required.");
    if(nlhs != 1) mexErrMsgTxt("Exactly one ouput argument required.");
    // --------------------------------------------------------------------
    
    // --------------------------------------------------------------------
    // Get pointer/values to/of the input and outputs objects
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    // 1st input: Image (get dimensions as well)
    if (!mxIsDouble(prhs[0])) mexErrMsgTxt("First input argument must be of type double.");
    pdImg = (double*) mxGetData(prhs[0]);
    const int* pSize = mxGetDimensions(prhs[0]);
    long lNDims = mxGetNumberOfDimensions(prhs[0]);
    lNY = long(pSize[0]);
	lNX = long(pSize[1]);
    long lUpdateCycle;
    if (lNDims == 3) {
        lNZ = long(pSize[2]);
        lUpdateCycle = UPDATECYCLE3D;
    } else {
        lNZ = 1;
        lUpdateCycle = UPDATECYCLE2D;
    }
    
    long    lImSize = lNX*lNY*lNZ;
    
    double  dMax, dMin;
    fGetMinMax(pdImg, lImSize, dMin, dMax);
    double  dDynamicRange = dMax - dMin;
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    // 2nd input: Seed point coordinates.
    short *pSeed = (short*) mxGetPr(prhs[1]);
    long lLinInd;
    if (lNZ > 1)
        lLinInd = long(pSeed[0]) - 1 + (long(pSeed[1]) - 1)*lNY + (long(pSeed[2]) - 1)*lNX*lNY;
    else
        lLinInd = long(pSeed[0]) - 1 + (long(pSeed[1]) - 1)*lNY;
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    // 3rd input: RG stoping difference.
    dMaxDif = double(*mxGetPr(prhs[2]));
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    // Get pointer to output arguments and allocate memory for the corresponding objects
    plhs[0] = mxCreateNumericArray(lNDims, pSize, mxLOGICAL_CLASS, mxREAL);	// create output array
    bool *pbMask = (bool*) mxGetData(plhs[0]);						// get data pointer to mask
    bool *pbCandidate = (bool*) mxGetData(mxCreateNumericArray(lNDims, pSize, mxLOGICAL_CLASS, mxREAL));	// create output array
        
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    // Create a parameter array for the call of the drawing function
    mxArray *pParams[2];
    bool bDraw = false;
    if (nrhs > 3) {
        pParams[0] = const_cast<mxArray *>(prhs[3]);
        pParams[1] = const_cast<mxArray *>(plhs[0]);
        bDraw = true;
    }

    // --------------------------------------------------------------------
    // Start of the real functionality
    
    aqQueue = new queue<long>[NQUEUES];
    
    long    lNHoodSize;
    long    alNHood[6];
    long    lInd;
    long    lRegSize = 1;
    long    lIterations = 0;
    long    lQueueInd;
    
    double  dDist;
    double  dLastDif = 0.0;
    
    dRegMean = pdImg[lLinInd];
    
    pbMask[lLinInd] = true;
    pbCandidate[lLinInd] = true;
    
    // --------------------------------------------------------------------
    while ((lRegSize < lImSize) && (dLastDif < dMaxDif)){

        fGetNHood(lLinInd, lNHoodSize, alNHood);
        for (int iI = 0; iI < lNHoodSize; iI++) {
            lLinInd = alNHood[iI];
            if (pbCandidate[lLinInd]) continue;
            
            pbCandidate[lLinInd] = true;
            lQueueInd = long(fabs(pdImg[lLinInd] - dRegMean) / dDynamicRange * NQUEUES * 2);
            if (lQueueInd > NQUEUES - 1) lQueueInd = NQUEUES - 1;
            aqQueue[lQueueInd].push(lLinInd);
        }
        
        lLinInd = fPop();
        if (lLinInd < 0) return;
        
        pbMask[lLinInd] = true;
        dRegMean = (dRegMean*double(lRegSize) + pdImg[lLinInd]);
        lRegSize++;
        dRegMean = dRegMean/double(lRegSize);
        
        if (!(lIterations % lUpdateCycle) && bDraw) {
            mexCallMATLAB(0, 0, 2, pParams, "feval");
        }
        lIterations++;
    }
    // End of while loop
    // --------------------------------------------------------------------
}
// ========================================================================
// *** END OF MAIN MEX FUNCTION RegionGrowing_mex
// ========================================================================