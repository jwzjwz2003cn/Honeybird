#include "mex.h"
#include "math.h"
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
   
//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context g_Context;
xn::DepthGenerator g_DepthGenerator;

/* The matlab mex function */
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
    double *Iout;
    XnUInt64 *MXadress;

    MXadress = (XnUInt64*)mxGetData(prhs[0]);
    if(MXadress[0]>0){ g_Context = ((xn::Context*) MXadress[0])[0]; }
    if(MXadress[2]>0)
	{ 
		g_DepthGenerator = ((xn::DepthGenerator*) MXadress[2])[0]; 
	}
	else
	{
		mexErrMsgTxt("No Depth Node in Kinect Context"); 
	}
        
    printf("XProj %d, Yproj %d, Zproj %d",(int)mxGetScalar(prhs[1]),(int)mxGetScalar(prhs[2]),(int)mxGetScalar(prhs[3]));
    
	XnPoint3D pt_proj = {(int)mxGetScalar(prhs[1]), (int)mxGetScalar(prhs[2]), (int)mxGetScalar(prhs[3])};
    XnPoint3D pt_world;

    g_DepthGenerator.ConvertProjectiveToRealWorld ( 1,  &pt_proj, &pt_world);
    
    plhs[0] = mxCreateDoubleMatrix(1,3,mxREAL);
    Iout = mxGetPr(plhs[0]);

    Iout[0]=pt_world.X;
    Iout[1]=pt_world.Y;
    Iout[2]=pt_world.Z;

}
