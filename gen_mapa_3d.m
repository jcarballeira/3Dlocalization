%Programa para crear un mapa en 3d a partir de otro en 2D dado

clc
I=imread('uc3m_p3_parcial_open.bmp');
imshow(I);

[m,n]=size(I);
o=25;

Mapa_3D=zeros(m,n,o);

for i=1:m
    for j=1:n
        for k=1:o
            Mapa_3D(i,j,k)=I(i,j);
        end
    end
end

%Falta obtener una buena representación donde se vea el mapa 3D contenido
%en la matriz Mapa_3D. De todas formas, con estos datos ya puede trabajar
%el algoritmo.

posicion=[50,200,1,0];

mapmax=[m,n,o];
mapmin=[1,1,1];

err_dis=0.02;

num_medidas=61; %medidas cada 3 grados
incr_theta=3; %en grados
num_barridos=9; %barridos cada 5 grados
incr_phi=5; %en grados

dist_est_3d=dist_est_3d(posicion,Mapa_3D,mapmax,mapmin,err_dis,num_medidas,num_barridos,incr_theta,incr_phi);

dist_real_3d=dist_real_3d(datoslaser,num_medidas,num_barridos,incr_theta,incr_phi);

error=fitness_3d(dist_real_3d, dist_est_3d, num_medidas, num_barridos);
