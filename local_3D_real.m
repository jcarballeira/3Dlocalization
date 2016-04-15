function [pos,pos_err,ori_err]=local_3D_real(posicion, err_dis, iter_max,mapmin,mapmax,Mapa_3D,NP,version_de,version_fitness,alpha)
%--------------------------------------------------------------------------
%   Main Program: local_3D_real
%   Author: Fernando Martin Monar.
%   Date: December, 2010
%--------------------------------------------------------------------------
% -> Description: Global Localization filter based on the Differential
% Evolution filter. The objective is to estimate the robot's location given
% a laser scan and the known map. Robot's motion is possible after
% convergence.
%   The robot's true pose is given via keyboard (initialization.m) and the
%   main programs calls alg_genet_3D, which contains the DE-based GL
%   filter. After convergence, the robot's pose estimate is returned in
%   bestmem.
%--------------------------------------------------------------------------
% -> Inputs (all optional): no inputs are required.
%
% -> Outputs: 
%       -Solution:
%           -pose_estimate: Vector of D elements containing the solution of
%           the global localization filter (robot's location). The first
%           three coordinates are the cartesian coordinates, in cells, and
%           the fourth one is the orientation.
%           -error: Cost value of the solution.
%--------------------------------------------------------------------------
% -> Usage: Solution=local_3D_real
%--------------------------------------------------------------------------
% Different options for the GL algorithm can be selected via keyboard:
%  - DE Core Options:
%    1) Random Mutation, with Thresholding and Discarding (Default). 
%    2) Basic version, Random Mutation, without Thresholding, Discarding.
%    3) Mutation from Best candidate, with Thresholding and Discarding.
%    4) Random Mutation, with Thresholding and Discarding, NP is
%    drastically reduced (tracking) after convergence.
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
%--------------------------------------------------------------------------
% -> File requirements: 'GLTreePro'
%--------------------------------------------------------------------------
% -> Data requirements: '3DMAP_open.mat'
%--------------------------------------------------------------------------
% -> See also: alg_genet_3D initialization dist_est_3d init_NP plot_results
%              fitness_3D
%--------------------------------------------------------------------------
%Initialization parameters:
num_medidas=61;         % number of horizontal measurements
incr_theta=3;           % laser horizontal resolution (degrees)
num_barridos=9;         % number of vertical measurements
incr_phi=10;            % laser vertical resolution (degrees)
D=4;                    % número de cromosomas del algoritmo
F=0.85;                 % factor de variacion diferencial (mutacion) (0-2)
CR=0.75;                % Crossover constant

%--------------------------------------------------------------------------
% The laser scan is generated according to the robot's true pose and the
% known map.
[dist_real_3d,cart_real_3d]=dist_est_rob_3d(posicion,Mapa_3D,mapmax,mapmin,err_dis,num_medidas,num_barridos,incr_theta);
% The next option can be applied to use a real laser scan (it has to be
% tested):
%   load datoslaser;
%   num_medidas_laser=laser_real_3d;
%   dist_real_3d=dist_real_3d(datoslaser,num_medidas,num_barridos,incr_theta);
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
poblacion=inicio_pob(NP,D,mapmin,mapmax); 
% The initial population is randomly generated to cover the whole map.
%--------------------------------------------------------------------------
        
        % EJECUCION DEL ALGORITMO GENETICO
        fprintf(1,'\n Posicion real del robot(x, y, z, theta): [%f, %f, %f, %f] \n',posicion(1),posicion(2),posicion(3),posicion(4));
        tic
        [bestmem,error,poblacion,F,NP]=alg_genet_3D(dist_real_3d,cart_real_3d,Mapa_3D,poblacion,mapmax,mapmin,err_dis,num_medidas,num_barridos,incr_theta,NP,D,iter_max,F,CR,version_de,version_fitness,alpha);
        toc
        %actualizo la posicion con la estimacion (posicion_e=bestmem)
        fprintf(1,'\n Posicion real del robot(x, y, z, theta): [%f, %f, %f, %f]\n',posicion(1),posicion(2),posicion(3),posicion(4));
        fprintf(1,'\n Posicion estimada tras ejecución:[%f, %f, %f, %f]\n',bestmem(2),bestmem(3),bestmem(4),bestmem(5));
        
        poserror=12.1*sqrt((posicion(1)-bestmem(2))^2+(posicion(2)-bestmem(3))^2+(posicion(3)-bestmem(4))^2);
        orierror=abs(posicion(4)-bestmem(5));
        fprintf(1,'\n El error de posicion es : %f cm y  el de orientacion: %f grados\n',poserror,orierror);
        
        %----------------------------------------------------------------
        
        
        %NP variable, se incrementa o disminuye en funcion del error
        %Funciona, pero cuando aumenta el numero de elementos lo hace
        %aleatoriamente, y como los demas ya llevan un tiempo evolucionando
        %suelen presentar mejores valores. Luego sería necesario repartir
        %los nuevos entre los mejores valores, aunque así se perdería la
        %diversidad q estamos buscando al incluir nuevos elementos.
        if version==6   
            if error<2000
                NP=int8(NP*0.90);
            else
                NPaux=int8(NP*1.1);
                restopob=completa_pob(NP,NPaux,D,mapmin,mapmax,Mapa_3D,err_dis,num_medidas,num_barridos,incr_theta,incr_phi,version,dist_real_3d);
                poblacion=[poblacion; restopob];
                NP=NPaux;
            end
        end
        
        %En la siguiente opcion, cuando el robot está localizado,
        %disminuimos drástcamente el número de elementos de la población,
        %ya que ahora el problema es de tracking.
        if version==7
            if error<50
                NP=10;
            end
        end        
        

pos=bestmem(2:(D+1));
pos_err=poserror;
ori_err=orierror;

%plot_results(Mapa_3D,poblacion,posicion,bestmem,dist_real_3d,num_medidas,num_barridos,incr_theta,incr_phi);
       
        


