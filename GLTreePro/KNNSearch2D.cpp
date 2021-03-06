#include "mex.h"




#include "GLTree2D.cpp"
//#include "FunctionsLib.h"




/* the gateway function */
// In matlab la funzione deve essere
//[NNG,dist]=KNNSearch(p,qp,ptrtree,k)

void mexFunction( int nlhs, mxArray *plhs[2],
int nrhs,  const mxArray *prhs[4])
{
    
    double       *p;

    double       *qp;

    
    double  *ptrtree;
    double * idcdouble;
    int N,Nq,i,j;
    double* outdistances;
    
    
    
    // Errors check
    
    if(nrhs!=4)
        mexErrMsgTxt("Four inputs required.");
    
      if(nlhs>2)
    {  mexErrMsgTxt("Maximum two outputs supported");}
    
    
    
  
   
 N=mxGetM(prhs[0]);
 Nq=mxGetM(prhs[1]);//non � il numero dei query solo un uso temporaneo
 
 
//  mexPrintf("N= %4.4f Nq=%4.4f\n",N,Nq);
 if(N!=2 || Nq!=2)
  { mexErrMsgTxt("Only 2D points supported ");
         } 
  
  if( !mxIsDouble(prhs[0]) || !mxIsDouble(prhs[1]))
 { mexErrMsgTxt("Inputs must be double vectors ");
         } 
    
    
    
    p = mxGetPr(prhs[0]);//puntatore all'array dei punti
   
    qp = mxGetPr(prhs[1]);//puntatore all'array dei punti query
   
    ptrtree = mxGetPr(prhs[2]);//puntatore all'albero precedentemente fornito
    double *k=mxGetPr(prhs[3]);
    
 
    
    
    
     int kint=*k;//numberof neighbours
     
     N=mxGetN(prhs[0]);//dimensione reference
     Nq=mxGetN(prhs[1]);//numero dei query 
     
    //  mexPrintf("N= %4.4f Nq=%4.4f\n",N,Nq); 
         if (*k>N)
  {
      mexErrMsgTxt("Can not run search, reference points are less than k");
         }
    
    plhs[0] = mxCreateDoubleMatrix(Nq, kint,mxREAL);//costruisce l'output array
    idcdouble = mxGetPr(plhs[0]);//appicicaci il puntatore 
    
    
    GLTREE2D *Tree;//dichiaro il puntatore l'oggetto
    
    Tree=(GLTREE2D*)((long)(ptrtree[0]));//ritrasformo il puntatore passato
    
 //  mexPrintf("puntatore= %4.4x\n",Tree);
    
    
    int* idc=new int[kint];
    double* distances=new double[kint];
   
    
    //converto la struttura dati
     Coord2D* pstruct=(Coord2D*) p; 
     Coord2D* qpstruct=(Coord2D*) qp;  

        plhs[1] = mxCreateDoubleMatrix(Nq, kint,mxREAL);//costruisce l'output array
        outdistances = mxGetPr(plhs[1]);//appicicaci il puntatore
        for (i=0;i<Nq;i++)
        {
//             mexPrintf("Prima della ricerca\n");          
            Tree->SearchKClosest(pstruct,&qpstruct[i],idc,distances,kint);//lancio la routine per la ricerca del pi� vicino
//            mexPrintf("Dopo la ricerca\n");
            for (j=0;j<kint;j++)
            {
                idcdouble[Nq*(j)+i]=idc[j]+1;//convert to matlab notation
                outdistances[Nq*(j)+i]=distances[j];
            }
        }
   
    //Free aloocated memory
    delete [] idc;
    delete [] distances;

}





