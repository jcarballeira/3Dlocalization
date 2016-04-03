function[dist_3d,cart_3d]=dist_est_rob_3d(posicion,Mapa_3D,mapmax,mapmin,err_dis,num_medidas,num_barridos,incr_theta)
%--------------------------------------------------------------------------
%   Main Function: dist_est_3d
%   Author: Fernando Martin Monar.
%   Date: December, 2010
%--------------------------------------------------------------------------
% -> Description: a 3D laser scan is generated when the robot is situated
% in position and the known map is given by Mapa_3D.
%--------------------------------------------------------------------------
% -> Inputs:
%       -posicion: Robot's true pose. Vector coordinates are given in
%       Cartesian coordinates and plane orientation (4 dof).
%       -Mapa_3D: Matrix that contains the 3D map of the environment.
%       -mapmax: Vector of 4 elements that corresponds to the map size. The
%       first three corrdinates are the map dimensions, in cartesian
%       coordinates, and the fourth one is the orientation, typically 360
%       degrees.
%       -mapmin: Minimum index in the map. Typically =[1,1,1,1].
%       -err_dis: Sensor noise, standard deviation, in percentage over the
%       distance weighted.
%       -num_medidas: Number of horizontal measurements.
%       -num_barridos: Number of vertical scans.
%       -incr_theta: Laser horizontal resolution.
%--------------------------------------------------------------------------
% -> Outputs: 
%       -dist_3d: Matrix with dimensions num_barridos*num_medidas
%       containing the distances of the laser measurements.
%       -cart_3d: Matrix with dimensions (num_barridos*num_medidas)*3
%       containing the 3D Cartesian coordinates of the laser measurements.
%--------------------------------------------------------------------------
% -> File requirements: this function is called by local_3D_real.m
%--------------------------------------------------------------------------
% -> Data requirements:
%   In Mapa_3D: 0 is an obstacle, 1 represents free space and 0.5 is
%   unknown. 
%--------------------------------------------------------------------------
% -> See also: local_3D_real
%--------------------------------------------------------------------------
cell_size=0.121;
s_range=10/cell_size;
dist_3d=zeros(num_barridos,num_medidas); % Se inicializan las medidas
cart_3d=zeros(num_barridos*num_medidas,3); % Laser estimate in cart coord
%--------------------------------------------------------------------------
%Los dos siguientes son unidades correspondientes a la dinámica del laser,
%en metros, pues el origen de medida del laser no coincide con el origen de
%giro, luego estos datos hay que ajustarlos.
doy=0;%.03/cell_size;
doz=0;%.05/cell_size;
%--------------------------------------------------------------------------
theta=posicion(4)+180;%-90; %angulo respecto al eje del robot de la primera medida
T=1; % Vector traslacion. T=[0.001 a 1]
barrido_vertical=1;
%--------------------------------------------------------------------------
for phi=-40:10:40
    for j=1:num_medidas
        x=posicion(1);
        y=posicion(2);
        z=posicion(3);     
       
        sin_sensor=sin(theta*pi/180); 
        cos_sensor=cos(theta*pi/180); 
        sin_altura=sin(phi*pi/180); 
        cos_altura=cos(phi*pi/180); 
        
        %Se transforman los datos reales del laser en coordenadas
        %cartesianas según la dinámica del láser
        incr_x=T*cos_sensor;
        incr_y=(T*sin_sensor+doy)*cos_altura + doz*sin_altura;
        incr_z=(T*sin_sensor+doy)*sin_altura - doz*cos_altura;      
        
        final=0;
        dis=0;
        cart=[0,0,0];

        while(final < 2) %Bucle que no para hasta que encuentra 2 celdas con obstáculos
            x_round=round(x);
            y_round=round(y);
            z_round=round(z);
            
            if (Mapa_3D(x_round,y_round,z_round)==0)%||(z_round>=mapmax(3))||(z_round<=mapmin(3)))    %~=0),  
                if (dis==0)
                    dis=sqrt((x-posicion(1))^2+(y-posicion(2))^2+(z-posicion(3))^2);
                    cart=[x-posicion(1), y-posicion(2), z-posicion(3)];
                    final=final+1;
                end           
            end

            x=x+incr_x;
            y=y+incr_y;
            z=z+incr_z;
            
            if x>mapmax(1), final=2;end
            if x<mapmin(1), final=2;end
            if y>mapmax(2), final=2;end
            if y<mapmin(2), final=2;end
            if z>mapmax(3), final=2;end
            if z<mapmin(3), final=2;end  
        end

        % se verifica
        if dis<0,dis=0;end
        if dis>s_range, dis=s_range;end
        if cart(1)<0, cart(1)=0;end
        if cart(1)>mapmax(1), cart(1)=mapmax(1);end
        if cart(2)<0, cart(2)=0;end
        if cart(2)>mapmax(2), cart(2)=mapmax(2);end
        if cart(3)<0, cart(3)=0;end
        if cart(3)>mapmax(3), cart(3)=mapmax(3);end
        
        dist_3d(barrido_vertical,j)=dis;
        cart_3d(barrido_vertical*j,:)=cart;
        
        theta=theta-incr_theta; %(incr_th0*pi/180); % incremento de angulo entre sensores
        
    end 
    barrido_vertical=barrido_vertical+1;
    theta=posicion(4)+180;%-90; %angulo respecto al eje del robot de la primera medida
   
end

% se añade el ruido de medida, cuya distribucion es normal
% Generate values from a normal distribution with mean 1 and standard deviation 2.
%           r = 1 + 2.*randn(100,1);
err_m=randn(size(dist_3d)); %NORMAL
%err_m2=randn(size(cart_3d));
%err_m=(rand(size(dist_3d))-0.5);  %UNIFORME
err_level=dist_3d.*err_dis;
%err_level2=cart_3d.*err_dis;
dist_3d=dist_3d+(err_m.*err_level);
%cart_3d=cart_3d+(err_m2.*err_level2);

% Uniform contamination can be included for testing.
CONTAMINATION_LEVEL=0.00; % Percentage of measurements that will be generated by uniform noise (over 1)
CORRECT_MEASUREMENTS=1-CONTAMINATION_LEVEL;  % Number of measurements that corresponde to true lectures (without uniform noise)
err_m2=(rand(size(dist_3d)).*0.5)+0.25;   % Uniform distribution in the interval [0.25,0.75];  
err_cont=dist_3d.*err_m2;      % Equivalent contaminated measurement for each reading. 
err_m3=rand(size(dist_3d));    % Uniform(0,1) distribution
err_m3=(err_m3<=CORRECT_MEASUREMENTS);  % if TRUE, err_m3(i)=1 
                                       % if FALSE, err_me(i)=0

err_m4=ones(size(dist_3d))-err_m3;
dist_3d=err_m3.*dist_3d+err_m4.*err_cont; % The new scan is formed
end

