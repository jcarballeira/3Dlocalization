function[NP]=init_NP(dist_real_3d,incr_theta,num_medidas,num_barridos,err_dis)
%--------------------------------------------------------------------------
%   Main Function: init_NP
%   Author: Fernando Martin Monar.
%   Date: February, 2012
%--------------------------------------------------------------------------
% -> Description: The population size is computed according to the
% information contained in a laser scan. The algorithm considers size,
% overlapping, symmetries... to estimate an optimum population size.
%--------------------------------------------------------------------------
% -> Inputs:
%       -dist_real_3d: matrix with dimensions num_barridos*num_medidas
%       containing the distances of the laser measurements.
%       -incr_theta: laser horizontal resolution.
%       -num_medidas: number of horizontal measurements.
%       -num_barridos: number of vertical scans.
%       -err_dis: sensor noise, standard deviation, in percentage over the
%       distance weighted.
% -> Output: 
%       -NP: population size.
%--------------------------------------------------------------------------
% -> See also: local_3D_real
%--------------------------------------------------------------------------
cell_size=0.121;
s_range=10/cell_size;
%Empiezo midiendo las celdillas que cubre un barrido, todas las medidas en
%celdillas.
%I=sum(sum(dist_real_3d))/(500*119*25);
%--------------------------------------------------------------------------
%Overlapping
l=1/tan(incr_theta*pi/180);
Aef=zeros(num_barridos,num_medidas); % Se inicializan las medidas
Aef(1,:)=dist_real_3d(1,:);
nd=0; %inicializo las dicontinuidades
%--------------------------------------------------------------------------
for i=1:num_barridos
    for j=2:num_medidas
        
        if (dist_real_3d(i,j)>l) && (dist_real_3d(i,j-1)>l)
            Aef(i,j)=cell_size*(dist_real_3d(i,j)-(l/2));
        else
            Aef(i,j)=cell_size*(dist_real_3d(i,j)-(l/2)+(l-min(dist_real_3d(i,j),dist_real_3d(i,j-1)))^2/(2*l));
        end
        
        %Discontinuidades
        dif=dist_real_3d(i,j)-dist_real_3d(i,j-1);
        dif_percent=(dist_real_3d(i,j)-dist_real_3d(i,j-1))/dist_real_3d(i,j)*100;
        if (dif>50)||(dif_percent>60)
            nd=nd+1;
        end
    end
end
NP=500*119*25/(sum(sum(Aef)));
%if nd>2
%   NP=(nd-1)*NP;
%end
NP=(1+sqrt(err_dis))*NP;
%Noise
end
