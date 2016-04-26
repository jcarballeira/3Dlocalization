function Resultado=experiments
%%Function to carry out experiments of global 3D localization, results of
%%the localization algorithm launched X times (Y iterations each) will be
%%saved into an array and showed on a table.Previously we must specify the
%%laser error, contamination error (in case of testing uniform noise response),
%%real robot's position, number of iterations, population number, type of
%%DE algorithm used, and type of cost function.
%--------------------------------------------------------------------------
clc
%The environment map is loaded
load '3DMAP_open.mat';
addpath('GLTreePro');
[m,n,o]=size(Mapa_3D);
mapmax=[m,n,o,360];
mapmin=[1,1,1,1];
divergencias=[3,4,9,10];%%divergencias usadas en cada experimento
alpha=1;

% Variables introduced via keyboard
[posicion, iter_max]=initialization(mapmin,mapmax,Mapa_3D);
%   Initialization of population size (NP). 2 options according to NP_opt: 
%                       1: Initialized by function init_NP
%                       Else: Fixed size given by code.
NP_opt=2;
if NP_opt==1
    NP=init_NP(dist_real_3d,incr_theta,num_medidas,num_barridos,err_dis);
else
    NP=450;
end
NP=round(NP)
version_de=1
% %--------------------------------------------------------------------------
% % Different options for the GL algorithm can be selected via keyboard:
% %  - DE Core Options:
% %    1) Random Mutation, with Thresholding and Discarding (Default). 
% %    2) Basic version, Random Mutation, without Thresholding, Discarding.
% %    3) Mutation from Best candidate, with Thresholding and Discarding.
% %    4) Random Mutation, with Thresholding and Discarding, NP is
% %    drastically reduced (tracking) after convergence.
% version_de=input('\ \n Introduce the DE version that you want to apply: \n 1) Random Mutation, with Thresholding and Discarding. \n 2) Basic version, Random Mutation, without Thresholding, Discarding. \n 3) Mutation from Best candidate, with Thresholding and Discarding. \n 4) Random Mutation, with Thresholding and Discarding, NP reduced (tracking) after convergence. \n');
% if isempty(version_de),
%     version_de=1;   
%     fprintf(1,'\n \t Option 1 by default. \n');
% end
%  - Fitness Function Options:
%    1) L2 Norm. Sum of the squared errors (Default).
%    2) L1 Norm. Sum of the absolute values of the error (Mahalanobis).
%    3) Kullback-Leibler Divergence based.
%    4) Density Power Divergence based.
%    5) Hellinger Distance based.
%    6) L2 Norm from Probability Distributions
%    7) L(Variable Exponent) Norm from Probability Distributions
%    8) Generalized Kullback-Leibler Divergence based.
%    9) Itakura-Saito Divergence based.
%    10) Jensen-Shannon Divergence based.
%    11) Havrda-Charvat Divergence based.
%    12) Rényi Divergence based.
% version_fitness=input('\ \n Introduce the Fitness Function that you want to apply: \n 1) L2 Norm. Sum of the squared errors (Default). \n 2) L1 Norm. Sum of the absolute values of the error. \n 3) Kullback-Leibler Divergence based. \n 4) Density Power Divergence based. \n 5) Hellinger Distance based. \n 6) L2 Norm from Probability Distributions \n 7) L(Variable Exponent) Norm from Probability Distributions. \n 8) Generalized Kullback-Leibler Divergence based. \n 9) Itakura-Saito Divergence based. \n 10) Jensen-Shannon Divergence based. \n 11) Havrda-Charvat Divergence based \n 12)Rényi Divergence based \n');
% if isempty(version_fitness),
%     version_fitness=1;   
%     fprintf(1,'\n \t Option 1 by default. \n');
% end
% 
% alpha=0;
% if ((version_fitness==4)||(version_fitness==7)||(version_fitness==11)||(version_fitness==12))
% alpha=input('\ \n Introduce variable parameter apply (0-1): \n ');
% if isempty(alpha),
%    alpha=0.1;   
%     fprintf(1,'\n \t Alpha=%f by default. \n',alpha);
% end
% end

numero_ensayos=input('\ \n Introduce numero de veces a lanzar el algoritmo para realizar el experimento \n');
if isempty(numero_ensayos),
    numero_ensayos=5;   
    fprintf(1,'\n \t Por defecto se realizan 5 pruebas \n');
end

fnam='A1';
for error_dis=1:10;
    
results=zeros(numero_ensayos,4);
div=3;

if div==3,
    div_string='KL';
elseif div==4,
    div_string='DPD';
elseif div==9;
    div_string='IS';
elseif div==10;
    div_string='JS';
end
    
 

for num_ensayo=1:numero_ensayos,
    fprintf(1,'\n \t %s, Error: %d. Ensayo número: %d.  \n',div_string,error_dis,num_ensayo);
    err_dis=error_dis/100;
    [pos,pos_err,ori_err]=local_3D_real(posicion, err_dis, iter_max,mapmin,mapmax,Mapa_3D,NP,version_de,div,alpha);
    results(num_ensayo,1)=error_dis;
    results(num_ensayo,2)=num_ensayo;
    results(num_ensayo,3)=pos_err;
    results(num_ensayo,4)=ori_err;
    
end

%exportamos tabla de resultados a una hoja excel
filename='experimentos_ruido_sensor_variable.xlsx';
sheet=div; %hoja de excel 
T=num2cell(results);
fext='A';
fnam=sprintf('%s%d',fext,numero_ensayos*error_dis);
xlswrite(filename,T,sheet,fnam); %escribimos la tabla

if error_dis==10,
    send_email
 end    
end
end




