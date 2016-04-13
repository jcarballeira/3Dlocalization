function error=fitness_3d(dist_real_3d, cart_real_3d, dist_est_3d, cart_est_3d,version_fitness,err_dis,alpha,num_medidas,num_barridos)
%--------------------------------------------------------------------------
%   Main Program: fitness_3d
%   Author: Fernando Martin Monar.
%   Date: December, 2010
%--------------------------------------------------------------------------
% -> Description: fitness function that is optimized by the DE-based Global
% localization filter. Laser measurements from the true location are
% compared to laser measurements from an estimate to compute a cost value
% for an estimate. This cost value will be minimized to obtain the solution
% of the GL problem.
%   Polar matching between laser orientations applied. Closest points could
%   be also computed (implemented at the end of the script).
%--------------------------------------------------------------------------
% -> Inputs:
%       -dist_real_3d: Matrix with dimensions num_barridos*num_medidas
%       containing the distances of the laser measurements when the robot
%       is situated in the true pose.
%       -cart_real_3d: Matrix with dimensions (num_barridos*num_medidas)*3
%       containing the 3D Cartesian coordinates of the laser measurements
%       when the robot is situated in the true pose.
%       -dist_est_3d: Matrix with dimensions num_barridos*num_medidas
%       containing the distances of the laser measurements when the robot
%       is assumed to be situated in an estimate.
%       -cart_est_3d: Matrix with dimensions (num_barridos*num_medidas)*3
%       containing the 3D Cartesian coordinates of the laser measurements
%       when the robot is assumed to be situated in an estimate.
%       -version: Option of the DE algorithm selected via keyboard in
%       local_3D_real.m
%       -err_dis: Sensor noise, standard deviation, in percentage over the
%       distance weighted.
% -> Output: 
%       -error: fitness value.
%--------------------------------------------------------------------------
% -> See also: local_3D_real alg_genet_3D
%--------------------------------------------------------------------------

% OJO, HAY QUE CAMBIAR EL VALOR, ESTA PA QUE EL ERROR ESA LA SUMA DE ERRORES
% INDIVIDUALES

cell_size=0.121; % tamaño de la celda en metros 
s_range=15/cell_size;


% for i=1:num_barridos
%     for j=1:num_medidas
%         if dist_real_3d(i,j) > s_range, dist_real_3d(i,j)=s_range; end
%         if dist_est_3d(i,j) > s_range, dist_est_3d(i,j)=s_range; end
%     end
% end

%En el caso de considerar la norma L1, el en cada medida la aportacion al
%error es la diferencia entre la medida estimada y la real, en valor
%absoluto, dividido por la desviación típica, para poder aplicar los
%criterios de convergencia desarrollados, siendo esto siempre una estimacion estadistica. 
if (version_fitness==2)
    error=sum(sum(abs(dist_real_3d - dist_est_3d)./(1+err_dis*dist_real_3d)));
%     for i=1:num_barridos
%         for j=1:num_medidas
%             error=error+abs((dist_real_3d(i,j)- dist_est_3d(i,j))/(err_dis*(dist_real_3d(i,j))));
%         end
%     end
end

%En el caso de considerar la norma L2, el en cada medida la aportacion al
%error es el cuadrado de la diferencia entre la medida estimada y la real, 
%dividido por la varianza multiplicada por 2, para poder aplicar los
%criterios de convergencia desarrollados, siendo esto siempre una
%estimacion estadistica. 
if (version_fitness==1)
    error=sum(sum(((dist_real_3d - dist_est_3d).^2)./(1+2*(err_dis*dist_real_3d).^2)));
%     for i=1:num_barridos
%         for j=1:num_medidas
%             error=error+((dist_real_3d(i,j)- dist_est_3d(i,j))^2/(1+2*(err_dis*dist_real_3d(i,j))^2));
%         end
%     end
    %error=sqrt(error); 
end
%error=sqrt(error)/(num_barridos*num_medidas);
%FALTA INCLUIR EL ERROR DE POSICION UNA VEZ HA LLEGADO A LA CONVERGENCIA.

%ptrtree=BuildGLTree3D(cart_real_3d');
%[kNNG,dist]=KNNSearch3D(cart_real_3d',cart_est_3d',ptrtree,1);
%error=sqrt(dist'*dist);
%error=sqrt((dist'*dist)/size(cart_real_3d,1));
if (version_fitness==3)
    KLD_T=0;
    OCL_T=0;
    for j=1:num_barridos
    for i=1:num_medidas
      
        KLD=0;
        if floor(dist_real_3d(j,i)+8) < floor(dist_est_3d(j,i)),
            %      posible oclusion
            %      medida       ______^_____
            %      estimacion   ___________^
            KLD=KLD+ floor(dist_real_3d(j,i))*log(0.1/0.05)*0.1  ; % antes 0.1
            KLD=KLD+ log(0.9/0.05)*0.9  ;
            KLD=KLD+ (floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*log(0.15/0.05)*0.15  ; %antes log(0.15/0.05)*0.15
            KLD=KLD+ log(0.15/0.95)*0.15  ; 
        elseif floor(dist_real_3d(j,i)+2)<floor(dist_est_3d(j,i)),
            %      medida       _________^--
            %      estimacion   ___________^
            KLD=KLD+ floor(dist_real_3d(j,i))*log(0.1/0.05)*0.1 ; % antes 0.1
            KLD=KLD+ log(0.9/0.05)*0.9 ;
            KLD=KLD+ (floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*log(0.5/0.05)*0.5  ;
            KLD=KLD+ log(0.5/0.95)*0.5 ;
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)+6),  
            % imposible
            KLD=KLD+floor(dist_real_3d(j,i))*log(0.95/0.05)*0.95 ;        
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)),    
            %      medida       ___________^
            %      estimacion   ________^---
            KLD=KLD+  floor(dist_est_3d(j,i))*log(0.1/0.05)*0.1  ;      
            KLD=KLD+  log(0.9/0.05)*0.9   ;
            KLD=KLD+  (floor(dist_real_3d(j,i))-ceil(dist_est_3d(j,i)))*log(0.99/0.05)*0.99 ;  
            KLD=KLD+  log(0.5/0.95)*0.5 ;
        elseif floor(dist_real_3d(j,i))<=floor(dist_est_3d(j,i)),   
            %      medida       ___________^
            %      estimacion   ___________^
            KLD=KLD+  floor(dist_real_3d(j,i))*log(0.1/0.05)*0.1 ;
            KLD=KLD+  log(0.9/0.95)*0.9 ;   
        end
        
        if (dist_real_3d(j,i)+8 <= dist_est_3d(j,i)),
            % oclusion
            OCL_T=OCL_T+1;
        end
        KLD_T=KLD_T+KLD;
    end
    end
    
    error=KLD_T*(exp(OCL_T/num_medidas*num_barridos));
end

if version_fitness==4
DPD=0;
    %OCL_T=0;
    for j=1:num_barridos
    for i=1:num_medidas

        if floor(dist_real_3d(j,i)+8) < floor(dist_est_3d(j,i)),
            %      posible oclusion
            %      medida       ______^_____
            %      estimacion   ___________^
            DPD=DPD+floor(dist_real_3d(j,i))*((0.05^alpha)-(1+1/alpha)*0.1*0.05^alpha+(1/alpha)*0.1^(1+alpha));
            DPD=DPD+((0.05^alpha)-(1+1/alpha)*0.9*0.05^alpha+(1/alpha)*0.9^(1+alpha));
            DPD=DPD+(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*((0.05^alpha)-(1+1/alpha)*0.15*0.05^alpha+(1/alpha)*0.15^(1+alpha));
            DPD=DPD+((0.95^alpha)-(1+1/alpha)*0.15*0.95^alpha+(1/alpha)*0.15^(1+alpha));
            %OCL_T=OCL_T+1;
        elseif floor(dist_real_3d(j,i)+2)<floor(dist_est_3d(j,i)),
            %      medida       _________^--
            %      estimacion   ___________^
            DPD=DPD+floor(dist_real_3d(j,i))*((0.05^alpha)-(1+1/alpha)*0.1*0.05^alpha+(1/alpha)*0.1^(1+alpha));
            DPD=DPD+((0.05^alpha)-(1+1/alpha)*0.9*0.05^alpha+(1/alpha)*0.9^(1+alpha));
            DPD=DPD+(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*((0.05^alpha)-(1+1/alpha)*0.5*0.05^alpha+(1/alpha)*0.5^(1+alpha));
            DPD=DPD+((0.95^alpha)-(1+1/alpha)*0.5*0.95^alpha+(1/alpha)*0.5^(1+alpha));
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)+6),  
            % imposible  
            DPD=DPD+floor(dist_real_3d(j,i))*((0.95^alpha)-(1+1/alpha)*0.05*0.95^alpha+(1/alpha)*0.05^(1+alpha));
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)),    
            %      medida       ___________^
            %      estimacion   ________^---
            DPD=DPD+floor(dist_est_3d(j,i))*((0.05^alpha)-(1+1/alpha)*0.1*0.05^alpha+(1/alpha)*0.1^(1+alpha));
            DPD=DPD+((0.95^alpha)-(1+1/alpha)*0.1*0.95^alpha+(1/alpha)*0.1^(1+alpha));
            DPD=DPD+(floor(dist_real_3d(j,i))-ceil(dist_est_3d(j,i)))*((0.5^alpha)-(1+1/alpha)*0.1*0.5^alpha+(1/alpha)*0.1^(1+alpha));
            DPD=DPD+((0.5^alpha)-(1+1/alpha)*0.9*0.5^alpha+(1/alpha)*0.9^(1+alpha));
        elseif floor(dist_real_3d(j,i))<=floor(dist_est_3d(j,i)),   
            %      medida       ___________^
            %      estimacion   ___________^
            DPD=DPD+floor(dist_real_3d(j,i))*((0.05^alpha)-(1+1/alpha)*0.1*0.05^alpha+(1/alpha)*0.1^(1+alpha));
            DPD=DPD+((0.95^alpha)-(1+1/alpha)*0.9*0.95^alpha+(1/alpha)*0.9^(1+alpha)); 
        end
    end
    end
    error=DPD/num_medidas*num_barridos;
    %error=DPD*(exp(OCL_T/num_medidas));
end 

if (version_fitness==5)
    
    HD_Total=0;
    %OCL_T=0;
    for j=1:num_barridos
    for i=1:num_medidas
        
        HD=0;
        if floor(dist_real_3d(j,i)+8) < floor(dist_est_3d(j,i)),
            %      posible oclusion
            %      medida       ______^_____
            %      estimacion   ___________^
            HD=HD+floor(dist_real_3d(j,i))*(sqrt(0.1)-sqrt(0.05))^2;
            HD=HD+(sqrt(0.9)-sqrt(0.05))^2;
            HD=HD+(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(sqrt(0.15)-sqrt(0.05))^2;
            HD=HD+(sqrt(0.15)-sqrt(0.95))^2;
            %OCL_T=OCL_T+1;
        elseif floor(dist_real_3d(j,i)+2)<floor(dist_est_3d(j,i)),
            %      medida       _________^--
            %      estimacion   ___________^
            HD=HD+floor(dist_real_3d(j,i))*(sqrt(0.1)-sqrt(0.05))^2;
            HD=HD+(sqrt(0.9)-sqrt(0.05))^2;
            HD=HD+(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(sqrt(0.5)-sqrt(0.05))^2;
            HD=HD+(sqrt(0.5)-sqrt(0.95))^2;

        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)+6),  
            % imposible
            HD=HD+floor(dist_real_3d(j,i))*(sqrt(0.95)-sqrt(0.05))^2;   
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)),    
            %      medida       ___________^
            %      estimacion   ________^---
            HD=HD+floor(dist_est_3d(j,i))*(sqrt(0.1)-sqrt(0.05))^2;
            HD=HD+(sqrt(0.9)-sqrt(0.05))^2;
            HD=HD+(floor(dist_real_3d(j,i))-ceil(dist_est_3d(j,i)))*(sqrt(0.99)-sqrt(0.05))^2;
            HD=HD+(sqrt(0.5)-sqrt(0.95))^2;
        elseif floor(dist_real_3d(j,i))<=floor(dist_est_3d(j,i)),   
            %      medida       ___________^
            %      estimacion   ___________^            
            HD=HD+floor(dist_real_3d(j,i))*(sqrt(0.1)-sqrt(0.05))^2;
            HD=HD+(sqrt(0.9)-sqrt(0.95))^2;  
        end
        
        HD_Total=HD_Total+sqrt(HD);
    end
    end
    error=1/sqrt(2)*HD_Total/num_medidas*num_barridos;
    %error=HD_Total*(exp(OCL_T/NUM_MEASUREMENTS));
end

% L2 Norm from Probability Distributions
if (version_fitness==6)
    
    L2_Total=0;
    %OCL_T=0;
    for j=1:num_barridos
    for i=1:num_medidas
        
        L2=0;
        if floor(dist_real_3d(j,i)+8) < floor(dist_est_3d(j,i)),
            %      posible oclusion
            %      medida       ______^_____
            %      estimacion   ___________^
            L2=L2+floor(dist_real_3d(j,i))*(0.1-0.05)^2;
            L2=L2+(0.9-0.05)^2;
            L2=L2+(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(0.15-0.05)^2;
            L2=L2+(0.15-0.95)^2;
            %OCL_T=OCL_T+1;
        elseif floor(dist_real_3d(j,i)+2)<floor(dist_est_3d(j,i)),
            %      medida       _________^--
            %      estimacion   ___________^
            L2=L2+floor(dist_real_3d(j,i))*(0.1-0.05)^2;
            L2=L2+(0.9-0.05)^2;
            L2=L2+(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(0.5-0.05)^2;
            L2=L2+(0.5-0.95)^2;
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)+6),  
            % imposible
            L2=L2+floor(dist_real_3d(j,i))*(0.95-0.05)^2;   
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)),    
            %      medida       ___________^
            %      estimacion   ________^---
            L2=L2+floor(dist_est_3d(j,i))*(0.1-0.05)^2;
            L2=L2+(0.9-0.05)^2;
            L2=L2+(floor(dist_real_3d(j,i))-ceil(dist_est_3d(j,i)))*(0.99-0.05)^2;
            L2=L2+(0.5-0.95)^2;
        elseif floor(dist_real_3d(j,i))<=floor(dist_est_3d(j,i)),   
            %      medida       ___________^
            %      estimacion   ___________^            
            L2=L2+floor(dist_real_3d(j,i))*(0.1-0.05)^2;
            L2=L2+(0.9-0.95)^2;  
        end
        
        L2_Total=L2_Total+sqrt(L2);
    end
    end
    error=L2_Total/num_medidas*num_barridos;
    %error=L2_Total*(exp(OCL_T/num_medidas));
end

% L(Variable exponent) Norm from Probability Distributions
if (version_fitness==7)
    
    LV_Total=0;
    %OCL_T=0;
    for j=1:num_barridos
    for i=1:num_medidas
        
        LV=0;
        if floor(dist_real_3d(j,i)+8) < floor(dist_est_3d(j,i)),
            %      posible oclusion
            %      medida       ______^_____
            %      estimacion   ___________^
            LV=LV+floor(dist_real_3d(j,i))*(0.1-0.05)^alpha;
            LV=LV+(0.9-0.05)^alpha;
            LV=LV+(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(0.15-0.05)^alpha;
            LV=LV+(0.15-0.95)^alpha;
            %OCL_T=OCL_T+1;
        elseif floor(dist_real_3d(j,i)+2)<floor(dist_est_3d(j,i)),
            %      medida       _________^--
            %      estimacion   ___________^
            LV=LV+floor(dist_real_3d(j,i))*(0.1-0.05)^alpha;
            LV=LV+(0.9-0.05)^alpha;
            LV=LV+(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(0.5-0.05)^alpha;
            LV=LV+(0.5-0.95)^alpha;
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)+6),  
            % imposible
            LV=LV+floor(dist_real_3d(j,i))*(0.95-0.05)^alpha;   
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)),    
            %      medida       ___________^
            %      estimacion   ________^---
            LV=LV+floor(dist_est_3d(j,i))*(0.1-0.05)^alpha;
            LV=LV+(0.9-0.05)^alpha;
            LV=LV+(floor(dist_real_3d(j,i))-ceil(dist_est_3d(j,i)))*(0.99-0.05)^alpha;
            LV=LV+(0.5-0.95)^alpha;
        elseif floor(dist_real_3d(j,i))<=floor(dist_est_3d(j,i)),   
            %      medida       ___________^
            %      estimacion   ___________^            
            LV=LV+floor(dist_real_3d(j,i))*(0.1-0.05)^alpha;
            LV=LV+(0.9-0.95)^alpha;  
        end
        
        LV_Total=LV_Total+LV^(1/alpha);
    end
    end
    error=LV_Total/num_medidas*num_barridos;
    %error=L2_Total*(exp(OCL_T/num_medidas));
end

% Generalized KL Divergence
if (version_fitness==8)
    
    KLD_T=0;
    OCL_T=0;
    for j=1:num_barridos
    for i=1:num_medidas
      
        KLD=0;
        if floor(dist_real_3d(j,i)+8) < floor(dist_est_3d(j,i)),
            %      posible oclusion
            %      medida       ______^_____
            %      estimacion   ___________^
            KLD=KLD+ floor(dist_real_3d(j,i))*(log(0.1/0.05)*0.1-0.1+0.05); % antes 0.1
            KLD=KLD+ log(0.9/0.05)*0.9-0.9+0.05  ;
            KLD=KLD+ (floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(log(0.15/0.05)*0.15-0.15+0.05)  ; %antes log(0.15/0.05)*0.15
            KLD=KLD+ log(0.15/0.95)*0.15-0.15+0.95  ; 
        elseif floor(dist_real_3d(j,i)+2)<floor(dist_est_3d(j,i)),
            %      medida       _________^--
            %      estimacion   ___________^
            KLD=KLD+ floor(dist_real_3d(j,i))*(log(0.1/0.05)*0.1-0.1+0.05) ; % antes 0.1
            KLD=KLD+ log(0.9/0.05)*0.9-0.9+0.05 ;
            KLD=KLD+ (floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(log(0.5/0.05)*0.5-0.5+0.05)  ;
            KLD=KLD+ log(0.5/0.95)*0.5-0.5+0.95 ;
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)+6),  
            % imposible
            KLD=KLD+floor(dist_real_3d(j,i))*(log(0.95/0.05)*0.95-0.95+0.05);        
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)),    
            %      medida       ___________^
            %      estimacion   ________^---
            KLD=KLD+  floor(dist_est_3d(j,i))*(log(0.1/0.05)*0.1-0.1+0.05)  ;      
            KLD=KLD+  log(0.9/0.05)*0.9-0.9+0-05   ;
            KLD=KLD+  (floor(dist_real_3d(j,i))-ceil(dist_est_3d(j,i)))*(log(0.99/0.05)*0.99-0.99+0.05) ;  
            KLD=KLD+  log(0.5/0.95)*0.5-0.5+0.95 ;
        elseif floor(dist_real_3d(j,i))<=floor(dist_est_3d(j,i)),   
            %      medida       ___________^
            %      estimacion   ___________^
            KLD=KLD+  floor(dist_real_3d(j,i))*(log(0.1/0.05)*0.1-0.1+0.05);
            KLD=KLD+  log(0.9/0.95)*0.9-0.9+0.95 ;   
        end
        
        if (dist_real_3d(j,i)+8 <= dist_est_3d(j,i)),
            % oclusion
            OCL_T=OCL_T+1;
        end
        KLD_T=KLD_T+KLD;
    end
    end
    
    error=KLD_T*(exp(OCL_T/num_medidas*num_barridos));

end

% Itakura-Saito
if (version_fitness==9)
    
    IS=0;
    %OCL_T=0;
    for j=1:num_barridos
    for i=1:num_medidas

        if floor(dist_real_3d(j,i)+8) < floor(dist_est_3d(j,i)),
            %      posible oclusion
            %      medida       ______^_____
            %      estimacion   ___________^          
            IS=IS+ floor(dist_real_3d(j,i))*(0.1/0.05-log(0.1/0.05)-1); % antes 0.1
            IS=IS+ 0.9/0.05-log(0.9/0.05)-1;
            IS=IS+ (floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(0.15/0.05-log(0.15/0.05)-1)  ; %antes log(0.15/0.05)*0.15
            IS=IS+ 0.15/0.95-log(0.15/0.95)-1  ; 
            %OCL_T=OCL_T+1;
        elseif floor(dist_real_3d(j,i)+2)<floor(dist_est_3d(j,i)),
            %      medida       _________^--
            %      estimacion   ___________^
            IS=IS+ floor(dist_real_3d(j,i))*(0.1/0.05-log(0.1/0.05)-1) ; % antes 0.1
            IS=IS+ 0.9/0.05-log(0.9/0.05)-1 ;
            IS=IS+ (floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(0.5/0.05-log(0.5/0.05)-1)  ;
            IS=IS+ 0.5/0.95-log(0.5/0.95)-1 ;
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)+6),  
            % imposible
            IS=IS+floor(dist_real_3d(j,i))*(0.95/0.05-log(0.95/0.05)-1);        
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)),    
            %      medida       ___________^
            %      estimacion   ________^---
            IS=IS+  floor(dist_est_3d(j,i))*(0.1/0.05-log(0.1/0.05)-1)  ;      
            IS=IS+  0.9/0.05-log(0.9/0.05)-1  ;
            IS=IS+  (floor(dist_real_3d(j,i))-ceil(dist_est_3d(j,i)))*(0.99/0.05-log(0.99/0.05)-1) ;  
            IS=IS+  0.5/0.95-log(0.5/0.95)-1 ;
        elseif floor(dist_real_3d(j,i))<=floor(dist_est_3d(j,i)),   
            %      medida       ___________^
            %      estimacion   ___________^
            IS=IS+  floor(dist_real_3d(j,i))*(0.1/0.05-log(0.1/0.05)-1);
            IS=IS+  0.9/0.95-log(0.9/0.95)-1;   
        end
    end
    end
    error=IS/num_medidas*num_barridos;
    %error=IS*(exp(OCL_T/num_medidas));
end

% Jensen-Shannon Divergence
if (version_fitness==10)
    
    JS=0;
    %OCL_T=0;
    for j=1:num_barridos
    for i=1:num_medidas
      
        if floor(dist_real_3d(j,i)+8) < floor(dist_est_3d(j,i)),
            %      posible oclusion
            %      medida       ______^_____
            %      estimacion   ___________^
            JS=JS+ 0.5*floor(dist_real_3d(j,i))*log(0.1/((0.05+0.1)/2))*0.1 + 0.5*floor(dist_real_3d(j,i))*log(0.05/((0.05+0.1)/2))*0.05  ; % antes 0.1
            JS=JS+ 0.5*log(0.9/((0.05+0.9)/2))*0.9+0.5*log(0.05/((0.05+0.9)/2))*0.05   ;
            JS=JS+ 0.5*(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*log(0.15/((0.05+0.15)/2))*0.15 +0.5*(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*log(0.05/((0.05+0.15)/2))*0.05  ; %antes log(0.15/0.05)*0.15
            JS=JS+ 0.5*log(0.15/((0.95+0.15)/2))*0.15+ 0.5*log(0.95/((0.95+0.15)/2))*0.95  ; 
            %OCL_T=OCL_T+1;
        elseif floor(dist_real_3d(j,i)+2)<floor(dist_est_3d(j,i)),
            %      medida       _________^--
            %      estimacion   ___________^
            JS=JS+ 0.5* floor(dist_real_3d(j,i))*log(0.1/((0.05+0.1)/2))*0.1 +0.5* floor(dist_real_3d(j,i))*log(0.05/((0.05+0.1)/2))*0.05 ; % antes 0.1
            JS=JS+ 0.5*log(0.9/((0.05+0.9)/2))*0.9 +0.5*log(0.05/((0.05+0.9)/2))*0.05 ;
            JS=JS+ 0.5*(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*log(0.5/((0.05+0.5)/2))*0.5+ 0.5*(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*log(0.05/((0.05+0.5)/2))*0.05  ;
            JS=JS+ 0.5*log(0.5/((0.95+0.5)/2))*0.5 +0.5*log(0.95/((0.95+0.5)/2))*0.95;
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)+6),  
            % imposible
            JS=JS+0.5*floor(dist_real_3d(j,i))*log(0.95/((0.05+0.95)/2))*0.95 + +0.5*floor(dist_real_3d(j,i))*log(0.05/((0.05+0.95)/2))*0.05  ;        
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)),    
            %      medida       ___________^
            %      estimacion   ________^---
            JS=JS+  0.5*floor(dist_est_3d(j,i))*log(0.1/((0.05+0.1)/2))*0.1 + 0.5*floor(dist_est_3d(j,i))*log(0.05/((0.05+0.1)/2))*0.05  ;      
            JS=JS+  0.5* log(0.9/((0.05+0.9)/2))*0.9 + 0.5* log(0.05/((0.05+0.9)/2))*0.05   ;
            JS=JS+  0.5*(floor(dist_real_3d(j,i))-ceil(dist_est_3d(j,i)))*log(0.99/((0.05+0.99)/2))*0.99+ 0.5*(floor(dist_real_3d(j,i))-ceil(dist_est_3d(j,i)))*log(0.05/((0.05+0.99)/2))*0.05 ;  
            JS=JS+  0.5*log(0.5/((0.5+0.95)/2))*0.5 +0.5*log(0.95/((0.95+0.5)/2))*0.95 ;
        elseif floor(dist_real_3d(j,i))<=floor(dist_est_3d(j,i)),   
            %      medida       ___________^
            %      estimacion   ___________^
            JS=JS+  0.5*floor(dist_real_3d(j,i))*log(0.1/((0.05+0.1)/2))*0.1+ 0.5*floor(dist_real_3d(j,i))*log(0.05/((0.05+0.1)/2))*0.05  ;
            JS=JS+  0.5*log(0.9/((0.95+0.9)/2))*0.9+ 0.5*log(0.95/((0.95+0.9)/2))*0.95 ;   
        end
       
    end
    end
    error=JS/num_medidas*num_barridos;
    %error=JS*(exp(OCL_T/num_medidas));
end
% Havrda-Charvat
if (version_fitness==11)

    HC=0; 
    for j=1:num_barridos
    for i=1:num_medidas
        if floor(dist_real_3d(j,i)+8) < floor(dist_est_3d(j,i)),
            %      posible oclusion
            %      medida       ______^_____
            %      estimacion   ___________^
           HC=HC+floor(dist_real_3d(j,i))*(1/alpha/(1-alpha))*((0.1^alpha)*(0.05^(1-alpha))-1);
           HC=HC+(1/alpha/(1-alpha))*((0.9^alpha)*(0.05^(1-alpha))-1);
           HC=HC+(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(1/alpha/(1-alpha))*((0.15^alpha)*(0.05^(1-alpha))-1);
           HC=HC+(1/alpha/(1-alpha))*((0.15^alpha)*(0.95^(1-alpha))-1);
        elseif floor(dist_real_3d(j,i)+2)<floor(dist_est_3d(j,i)),
            %      medida       _________^--
            %      estimacion   ___________^ 
           HC=HC+floor(dist_real_3d(j,i))*(1/alpha/(1-alpha))*((0.1^alpha)*(0.05^(1-alpha))-1);
           HC=HC+(1/alpha/(1-alpha))*((0.9^alpha)*(0.05^(1-alpha))-1);
           HC=HC+(floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(1/alpha/(1-alpha))*((0.5^alpha)*(0.05^(1-alpha))-1);
           HC=HC+(1/alpha/(1-alpha))*((0.5^alpha)*(0.95^(1-alpha))-1);
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)+6),  
            % imposible  
            HC=HC+floor(dist_real_3d(j,i))*(1/alpha/(1-alpha))*((0.95^alpha)*(0.05^(1-alpha))-1);
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)),    
            %      medida       ___________^
            %      estimacion   ________^--- 
           HC=HC+floor(dist_est_3d(j,i))*(1/alpha/(1-alpha))*((0.1^alpha)*(0.05^(1-alpha))-1);
           HC=HC+(1/alpha/(1-alpha))*((0.1^alpha)*(0.95^(1-alpha))-1);
           HC=HC+(floor(dist_real_3d(j,i))-ceil(dist_est_3d(j,i)))*(1/alpha/(1-alpha))*((0.1^alpha)*(0.5^(1-alpha))-1);
           HC=HC+(1/alpha/(1-alpha))*((0.9^alpha)*(0.5^(1-alpha))-1);
        elseif floor(dist_real_3d(j,i))<=floor(dist_est_3d(j,i)),   
            %      medida       ___________^
            %      estimacion   ___________^
           HC=HC+floor(dist_real_3d(j,i))*(1/alpha/(1-alpha))*((0.1^alpha)*(0.05^(1-alpha))-1);
           HC=HC+(1/alpha/(1-alpha))*((0.9^alpha)*(0.95^(1-alpha))-1);
        end
    end
    end
    error=abs(HC);
    
end    
% Rényi divergence    
if (version_fitness==12),
    RY_T=0;
    OCL_T=0;
    for j=1:num_barridos
    for i=1:num_medidas
      
        RY=0;
        if floor(dist_real_3d(j,i)+8) < floor(dist_est_3d(j,i)),
            %      posible oclusion
            %      medida       ______^_____
            %      estimacion   ___________^
            RY=RY+ floor(dist_real_3d(j,i))*(0.1^alpha)/0.05^(alpha-1);
            RY=RY+ (0.9^alpha)/(0.05^(alpha-1))  ;
            RY=RY+ (floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*((0.15^alpha)/(0.05^(alpha-1)));
            RY=RY+ 0.15^alpha/0.95^(alpha-1); 
        elseif floor(dist_real_3d(j,i)+2)<floor(dist_est_3d(j,i)),
            %      medida       _________^--
            %      estimacion   ___________^
            RY=RY+ floor(dist_real_3d(j,i))*(0.1^alpha/0.05^(alpha-1));
            RY=RY+ 0.9^alpha/0.05^(alpha-1);
            RY=RY+ (floor(dist_est_3d(j,i))-ceil(dist_real_3d(j,i)))*(0.5^alpha/0.05^(alpha-1));
            RY=RY+ 0.5^alpha/0.95^(alpha-1);
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)+6),  
            % imposible
            RY=RY+floor(dist_real_3d(j,i))*(0.95^alpha/0.05^(alpha-1));        
        elseif floor(dist_real_3d(j,i))>floor(dist_est_3d(j,i)),    
            %      medida       ___________^
            %      estimacion   ________^---
            RY=RY+  floor(dist_est_3d(j,i))*(0.1^alpha/0.05^(alpha-1));      
            RY=RY+  0.9^alpha/0.05^(alpha-1);
            RY=RY+  (floor(dist_real_3d(j,i))-ceil(dist_est_3d(j,i)))*(0.99^alpha/0.05^(alpha-1));  
            RY=RY+  0.5^alpha/0.95^(alpha-1);
        elseif floor(dist_real_3d(j,i))<=floor(dist_est_3d(j,i)),   
            %      medida       ___________^
            %      estimacion   ___________^
            RY=RY+  floor(dist_real_3d(j,i))*(0.1^alpha/0.05^(alpha-1));
            RY=RY+  0.9^alpha/0.95^(alpha-1);   
        end
        
        if (dist_real_3d(j,i)+8 <= dist_est_3d(j,i)),
            % oclusion
            OCL_T=OCL_T+1;
        end
        RY_T=RY_T+RY;
        RY_T=1/(alpha-1)*log(RY_T);
    end
    end
    
    error=RY_T*(exp(OCL_T/num_medidas*num_barridos));
end    
    
end


