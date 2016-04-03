function[posicion, err_dis, iter_max]=initialization(mapmin,mapmax,Mapa_3D)
%--------------------------------------------------------------------------
%   Function: initialization
%   Author: Fernando Martin Monar.
%   Date: December, 2010
%--------------------------------------------------------------------------
% -> Description: The initial configuration is introduced via keyboard
%--------------------------------------------------------------------------
% -> Inputs:
%       -mapmin: Minimum index in the map. Typically =[1,1,1,1].
%       -mapmax: Vector of 4 elements that corresponds to the map size. The
%       first three corrdinates are the map dimensions, in cartesian
%       coordinates, and the fourth one is the orientation, typically 360
%       degrees.
%       -Mapa_3D: matrix that contains the 3D map of the environment.
%--------------------------------------------------------------------------
% -> Outputs: 
%       -posicion: robot's true pose. Vector coordinates are given in
%       Cartesian coordinates and plane orientation (4 dof).
%       -err_dis: sensor noise, standard deviation, in percentage over the
%       distance weighted.
%       -iter_max: maximum number of iterations.
%--------------------------------------------------------------------------
% -> File requirements: this function is called by local_3D_real.m
%--------------------------------------------------------------------------
% -> See also: local_3D_real
%--------------------------------------------------------------------------

% The robot initial orientation is introduced.
th0=input('\n Orientacion del robot (en grados):');
if th0>=360,th0=th0-360;end
if th0<0,th0=th0+360;end 
if isempty(th0),    
    th0=0;  
    fprintf(1,'\n Por defecto la orientacion del robot es %.2fº \n',th0);
end
%--------------------------------------------------------------------------
posicion=[1,1,1,th0]; % th0 in degrees
%--------------------------------------------------------------------------
% The robot cartesian coordinates, limited by map borders, are introduced.
fprintf(1,'\n Introduzca las coordenadas del robot:')
while(Mapa_3D(round(posicion(1)),round(posicion(2)),round(posicion(3)))==0),
     posicion(1)=input('\n Introduzca la coordenada x:');
     if(posicion(1)<mapmin(1)||posicion(1)>mapmax(1)),
         fprintf(1,'\n Limites en x del mapa son [%.2f,%.2f]:',mapmin(1),mapmax(1));
         posicion(1)=input('\n Por favor, introduzca la coordenada x:');
     end
     posicion(2)=input('\n Introduzca la coordenada y: ');
     if(posicion(2)<mapmin(2)||posicion(2)>mapmax(2)),
        fprintf(1,'\n Limites de y en el mapa [%.2f,%.2f] ',mapmin(2),mapmax(2));
         posicion(2)=input('\n \ Por favor, introduzca la coordenada y:');
     end
     posicion(3)=input('\n Introduzca la coordenada z: ');
     if(posicion(3)<mapmin(3)||posicion(3)>mapmax(3)),
        fprintf(1,'\n Limites de z en el mapa [%.2f,%.2f] ',mapmin(3),mapmax(3));
         posicion(3)=input('\n \ Por favor, introduzca la coordenada z:');
     end
     if(Mapa_3D(round(posicion(1)),round(posicion(2)),round(posicion(3)))==0),
         fprintf(1,'\n El punto elegido es un obstaculo. \n Debe introducir otro punto.\n');
     end
end
%--------------------------------------------------------------------------
%The simulated laser error is computed
err_dis=input('\n Introduzca el error del sensor a introducir en el calculo de las distancias (en %): \ \');
err_dis=err_dis/100;
if isempty(err_dis),
    err_dis=0.1;    % Error en las distancias err_dis=10%
    fprintf(1,'\n \t Por defecto el error cometido en las lecturas de los sensores es del %.2f%%  \n',err_dis*100);
end
%--------------------------------------------------------------------------
% Genetic algorithm population size
% NP=100;
% NP=input('\ \n Introduzca el numero de individuos de la poblacion: \ \');
% NP=round(NP);
% if isempty(NP),
%     NP=ceil(sqrt(mapmax(1)*mapmax(2)));    
%     fprintf(1,'\n \t Por defecto el numero de individuos es %d \n',NP);
% end
%--------------------------------------------------------------------------
% Genetic algorithm upper iteration limit
iter_max=input('\ \n Introduzca el numero de iteraciones: \ \');
iter_max=round(iter_max);
if isempty(iter_max),
    iter_max= 40;   
    fprintf(1,'\n \t Por defecto el numero de iteraciones es %d \n',iter_max);
end
end