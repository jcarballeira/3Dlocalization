%Programa con el que se comprueba que el punto elegido está a más de 5
%celdillas de los obstáculos

clc
I=imread('uc3m_p3_parcial_open.bmp');
iptsetpref('ImshowAxesVisible','on');

[m,n]=size(I);
o=25;
posicion=[1,1,1,1];
mapmax=[m,n,o,360];
mapmin=[1,1,1,1];

%Creo un mapa auxiliar dilatado 5 celdillas para asegurarme de que no elijo
%un punto a localizar que esté a menos de 5 celdillas de las paredes. es
%una condición de seguridad
se = strel('square',6);
erodedI = imerode(I,se);

Mapa_3D_auxiliar=zeros(m,n,o);
for i=1:m
    for j=1:n
        for k=1:o
            Mapa_3D_auxiliar(i,j,k)=erodedI(i,j);
        end
    end
end

fprintf(1,'\n Introduzca las coordenadas del robot:')
while(Mapa_3D_auxiliar(round(posicion(1)),round(posicion(2)),round(posicion(3)))==0),
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
     if(Mapa_3D_auxiliar(round(posicion(1)),round(posicion(2)),round(posicion(3)))==0),
         fprintf(1,'\n El punto elegido es un obstaculo. \n Debe introducir otro punto.\n');
     end
end