%VA BIEN, PERO LOS JOD... DATOS DEL LASER QUE COGÍ AL PRINCIPIO EMPIEZAN
%CON THETA IGUAL A 1 PERO A PARTIR DE CIERTO ANGULO CAMBIAN A THETA IGUAL A
%0 COMO PRIMER VALOR, LO QUE MOLESTA BASTANTE

%Funcion que devuelve las distancias reales del laser en el 
%mismo formato que las estimadas

function [dist_real]=dist_real_3d(datoslaser,num_medidas,num_barridos,incr_theta,incr_phi)

%datos laser es una matriz de 10980*3 en la que hay 180 medidas en cada
%barrido horizontal y un barrido vertical cada grado, de -20 a 20

cell_size=0.121;
s_range=10/cell_size;

[q,r]=size(datoslaser);

dist_real=zeros(num_barridos,num_medidas);

angbarrido=-20;
ind=1;
barrido_vertical=1;

while((ind<=q)&&(angbarrido<=20))
    medida_horizontal=1;
    %fprintf(1,' \n  datos laser, angbarrido %2.3f %2.3f q ind %2.3f %2.3f \n',datoslaser(ind,1),angbarrido,q,ind);
        
    while (datoslaser(ind,1)==angbarrido)
        dist_real(barrido_vertical,medida_horizontal)=datoslaser(ind,3);
        ind=ind+incr_theta;
        medida_horizontal=medida_horizontal+1;
    end
        
    barrido_vertical=barrido_vertical+1;
    angbarrido=angbarrido+incr_phi;

    while (datoslaser(ind,1)~=angbarrido)
        ind=ind+1;    
    end
end

