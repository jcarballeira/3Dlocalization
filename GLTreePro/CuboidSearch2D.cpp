#include "mex.h"




#include "GLTree2D.cpp"


//#include "FunctionsLib.h"


// In Matlab la funzione deve essere idc=NNSearch(x,y,pk,ptrtree)

/* the gateway function */
//In Matlab deve essere idc=CuboidSearch(p,cuboid,ptrtree)

void mexFunction( int nlhs, mxArray *plhs[1], int nrhs,  const mxArray *prhs[3]) {
    
    double       *p;
    double  *ptrtree, *idcdouble, *Cuboid;//pointe to tree, index found, cuoboid
    int N, i;//number of points and counter
    int npts;//number of points found
    int rows, columns;
    
    
    
    
    // Errors check
    
    if(nrhs!=3)
    { mexErrMsgTxt("3 inputs required.");}
    
    if(nlhs>1)
    { mexErrMsgTxt("Only one output supported.");}
    
    
    rows=mxGetM(prhs[0]);
    
    if (rows!=2) { mexErrMsgTxt("Only 2D points supported check input number 1");}
    
    
    rows=mxGetM(prhs[1]);
    columns=mxGetN(prhs[1]);
    if ( rows!=4){mexErrMsgTxt("Input number 2: Cuboid must be 4x1 elements");}
    
    if( !mxIsDouble(prhs[0]) || !mxIsDouble(prhs[1]) || !mxIsDouble(prhs[2] ))
    { mexErrMsgTxt("Inputs must be double array ");}
    
    
    N=mxGetN(prhs[0]);//number of points
    p = mxGetPr(prhs[0]);//puntatore all'array dei punti
    
    
    
    
    Cuboid= mxGetPr(prhs[1]);//Radius
    ptrtree = mxGetPr(prhs[2]);//puntatore all'albero precedentemente fornito
    
    if( ptrtree == NULL )
    { mexErrMsgTxt("ptrtree must be a valid pointer");}
    
    
     if( Cuboid[1]<=Cuboid[0] || Cuboid[3]<=Cuboid[2])
    { mexErrMsgTxt("Invalid Cuboid");}
    
    
    
    
    
    
    GLTREE2D *Tree;//dichiaro il puntatore l'oggetto
    
    Tree=(GLTREE2D*)((long)(ptrtree[0]));//ritrasformo il puntatore passato
    
//     mexPrintf("puntatore= %4.4x\n",Tree);
    
    
    //x e y sono i vettori dei reference points passati
    //pk � un array [2x1] con x e y del query point
    //idc double � l'id del pi� vicino, sarebbe un integer ma per ridare il puntatore a double
    
    
    
    
    //  mexPrintf("Copy data structure\n");
    Coord2D* pstruct=(Coord2D*) p;
    
    
    
    
   // mexPrintf("Function Call\n");
    
    Tree->SearchCuboid(pstruct, Cuboid, &npts);
    
   //  mexPrintf("Function exit\n");
    
    plhs[0] = mxCreateDoubleMatrix(npts, 1, mxREAL);//costruisce l'output array
    idcdouble = mxGetPr(plhs[0]);//appicicaci il puntatore
    
    for (i=0;i<npts;i++)//copy the results in the output array
    {
        idcdouble[i]=Tree->idStore[i]+1;
    }
}




