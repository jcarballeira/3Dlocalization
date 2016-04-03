function v=laser_real_3d;

load datoslaser;

clear laser_X
clear laser_Y
clear laser_Z
%clear laser_2D_X
%clear laser_2D_Y
tic;
datos_aux=datoslaser;

%elimino lecturas erróneas
ind_datos=1;
for i=1:10980
    if (datos_aux(i,3)<1500)
        datos(ind_datos,1)=0.5*datos_aux(i,1);%grados
        datos(ind_datos,2)=datos_aux(i,2);%grados
        datos(ind_datos,3)=datos_aux(i,3);%cm
        ind_datos=ind_datos+1;
    end
end
ind_datos=ind_datos-1;

for i=1:ind_datos
phi(i)=datos(i,1);
end
for i=1:ind_datos
theta(i)=datos(i,2);
end
for i=1:ind_datos
rho(i)=datos(i,3);
end
for i=1:ind_datos
X_local(i)=rho(i)*cos(phi(i)*pi/180)*cos(theta(i)*pi/180);
Y_local(i)=rho(i)*cos(phi(i)*pi/180)*sin(theta(i)*pi/180);
Z_local(i)=rho(i)*sin(phi(i)*pi/180);
end

v=ind_datos;

ind_suelo=0;
for i=1:ind_datos
    if ((-50<Z_local(i))&&(Z_local(i)<-30))
        ind_suelo=ind_suelo+1;
        X_local_suelo(ind_suelo)=X_local(i);
        Y_local_suelo(ind_suelo)=Y_local(i);
        Z_local_suelo(ind_suelo)=Z_local(i);
    end
end

%pinto el techo
ind_techo=1;
while (datos(ind_techo,1)==datos(ind_techo+1,1))
    ind_techo=ind_techo+1;
end
ind_techo=ind_techo+1;
while (datos(ind_techo,1)==datos(ind_techo+1,1))
    datos_techo(ind_techo,1)=datos(ind_techo,1);
    datos_techo(ind_techo,2)=datos(ind_techo,2);
    datos_techo(ind_techo,3)=datos(ind_techo,3);
    ind_techo=ind_techo+1;
end
ind_techo=ind_techo-1;
for i=1:ind_techo
phi_techo(i)=datos_techo(i,1);
end
for i=1:ind_techo
theta_techo(i)=datos_techo(i,2);
end
for i=1:ind_techo
rho_techo(i)=datos_techo(i,3);
end
for i=1:ind_techo
X_local_techo(i)=rho_techo(i)*cos(phi_techo(i)*pi/180)*cos(theta_techo(i)*pi/180);
Y_local_techo(i)=rho_techo(i)*cos(phi_techo(i)*pi/180)*sin(theta_techo(i)*pi/180);
Z_local_techo(i)=rho_techo(i)*sin(phi_techo(i)*pi/180);
end


%theta=double(int16((180/pi)*atan2(Y_local,X_local)));
%phi=double(int16((180/pi)*atan2(Z_local,sqrt((X_local).^2+(Y_local).^2))));
%rho=sqrt((X_local).^2+(Y_local).^2+(Z_local).^2);

Matriz_esferica_aux=[theta; phi; rho; X_local; Y_local; Z_local];
Matriz_esferica=rot90(Matriz_esferica_aux,1);

%ordeno para eliminar datos con misma rho y theta(ocultos)
Matriz_esferica_ord=sortrows(Matriz_esferica);

%elimino datos que no veria el laser
%k=1;
%v=1;

%u=1;

%Matriz_esferica_ord_aux = circshift(Matriz_esferica_ord,1);

%while (k<(j-2))
%    h=v;
%   
%    if (((Matriz_esferica_ord(k,1)) ~= (Matriz_esferica_ord_aux(k,1))) || ((Matriz_esferica_ord(k,2)) ~= (Matriz_esferica_ord_aux(k,2))))
%            if ((Matriz_esferica_ord(k,1)>= (0-orienta))&&(Matriz_esferica_ord(k,1)<=(180-orienta))&&(Matriz_esferica_ord(k,2)>=phimin)&&(Matriz_esferica_ord(k,2)<= phimax))
%                laser_X(v,1)=Matriz_esferica_ord(k,4)%+ random('Normal',0,ruido);
%                laser_Y(v,1)=Matriz_esferica_ord(k,5)%+ random('Normal',0,ruido);
%                laser_Z(v,1)=Matriz_esferica_ord(k,6)%+ random('Normal',0,ruido);
%                v=v+1;
%             end
%    end
    
%    k=k+1;
    
    %if (h==v)
     %   k=k+2;
    %else
    %    k=k+1;
    %end
    
%end

%t2=toc
t1=toc;
tic;
ind=1;
for d=1:(v-1)
    if( abs(Z_local(d)) <= 0.1)
        laser_2D_X(ind)=X_local(d);
        laser_2D_Y(ind)=Y_local(d);
        laser_2D_Z(ind)=Z_local(d);
        ind=ind+1;
    end
end
ind_bar1=1;
for d=1:(v-1)
    if((48<Z_local(d))&&(Z_local(d)<52))
        laser_2D_X_alto(ind_bar1)=X_local(d);
        laser_2D_Y_alto(ind_bar1)=Y_local(d);
        laser_2D_Z_alto(ind_bar1)=Z_local(d);
        ind_bar1=ind_bar1+1;
    end
end
ind_bar2=1;
for d=1:(v-1)
    if((-52<Z_local(d))&&(Z_local(d)<-48))
        laser_2D_X_bajo(ind_bar2)=X_local(d);
        laser_2D_Y_bajo(ind_bar2)=Y_local(d);
        laser_2D_Z_bajo(ind_bar2)=Z_local(d);
        ind_bar2=ind_bar2+1;
    end
end


theta_2D=double(int16((180/pi)*atan2(laser_2D_Y,laser_2D_X)));
rho_2D=sqrt((laser_2D_X).^2+(laser_2D_Y).^2);

Matriz_esferica_aux_2D=[theta_2D;rho_2D;laser_2D_X;laser_2D_Y];
Matriz_esferica_2D=rot90(Matriz_esferica_aux_2D,1);

%ordeno para eliminar datos con misma theta(ocultos)
Matriz_esferica_ord_2D=sortrows(Matriz_esferica_2D);

p=1;
theta_0=0;
incr_theta_0=3;
v0_real=zeros(1,61);
ind_v0=1;
max_med=15;
while(theta_0 <=180)
    
    while((Matriz_esferica_ord_2D(p,1)~=theta_0)&&(p<(ind-1)))
        p=p+1;
    end
    if (Matriz_esferica_ord_2D(p,1)==theta_0)
        v0_real(1,ind_v0)=Matriz_esferica_ord_2D(p,2);
    else
        v0_real(1,ind_v0)=max_med;
        p=1;
    end
    ind_v0=ind_v0+1;
    theta_0=theta_0+incr_theta_0;

end

gridcell=12.1;
v0_grid=v0_real/gridcell;

%Barrido a nivel alto
theta_2D_alto=double(int16((180/pi)*atan2(laser_2D_Y_alto,laser_2D_X_alto)));
rho_2D_alto=sqrt((laser_2D_X_alto).^2+(laser_2D_Y_alto).^2);

Matriz_esferica_aux_2D_alto=[theta_2D_alto;rho_2D_alto;laser_2D_X_alto;laser_2D_Y_alto];
Matriz_esferica_2D_alto=rot90(Matriz_esferica_aux_2D_alto,1);

%ordeno para eliminar datos con misma theta(ocultos)
Matriz_esferica_ord_2D_alto=sortrows(Matriz_esferica_2D_alto);

p=1;
theta_0=0;
incr_theta_0=3;
v0_real_alto=zeros(1,61);
ind_v0_alto=1;
max_med=15;
while(theta_0 <=180)
    
    while((Matriz_esferica_ord_2D_alto(p,1)~=theta_0)&&(p<(ind-1)))
        p=p+1;
    end
    if (Matriz_esferica_ord_2D_alto(p,1)==theta_0)
        v0_real_alto(1,ind_v0_alto)=Matriz_esferica_ord_2D_alto(p,2);
    else
        v0_real_alto(1,ind_v0_alto)=max_med;
        p=1;
    end
    ind_v0_alto=ind_v0_alto+1;
    theta_0=theta_0+incr_theta_0;

end

v0_grid_alto=v0_real_alto/gridcell;


%para poner los datos según el mapa
% v0_grid=rot90(v0_grid);
% v0_grid=rot90(v0_grid);


t3=toc;
orienta=0;

 A=[0 (1*sin(orienta*pi/180))];
 B=[0 (1*cos(orienta*pi/180))];
 C=[0 0];
% 
% %IMPRIMO FIGURAS
% figure(5)
% plot3(X_local,Y_local,Z_local,'square',...
%                 'LineWidth',1,...
%                 'MarkerEdgeColor','b',...
%                 'MarkerFaceColor',[0 0 1],...
%                 'MarkerSize',1);
% grid on
% xlabel('X [cm]')
% ylabel('Y [cm]')
% zlabel('Z [cm]')
% title('LASER 3D')
% hold on
% plot3(0,0,0,'o',...
%                 'LineWidth',1,...
%                 'MarkerEdgeColor','k',...
%                 'MarkerFaceColor',[0 0 0],...
%                 'MarkerSize',10);
% hold off           
% hold on
% plot3(A,B,C,'-r',...
%                 'LineWidth',2,...
%                 'MarkerEdgeColor','r',...
%                 'MarkerFaceColor',[1 0 0],...
%                 'MarkerSize',8);
% hold off
% % hold on
% % plot3(X_local_suelo,Y_local_suelo,Z_local_suelo,'o',...
% %                 'LineWidth',1,...
% %                 'MarkerEdgeColor','k',...
% %                 'MarkerFaceColor',[0 0 0],...
% %                 'MarkerSize',2);
% % hold off
% hold on
% plot3(X_local_techo,Y_local_techo,Z_local_techo,'o',...
%                 'LineWidth',1,...
%                 'MarkerEdgeColor','k',...
%                 'MarkerFaceColor',[0 0 0],...
%                 'MarkerSize',4);
% hold off
% hold on
% plot3(laser_2D_X,laser_2D_Y,laser_2D_Z,'o',...
%                 'LineWidth',1,...
%                 'MarkerEdgeColor','r',...
%                 'MarkerFaceColor',[1 0 0],...
%                 'MarkerSize',4);
% hold off
% hold on
% plot3(laser_2D_X_alto,laser_2D_Y_alto,laser_2D_Z_alto,'o',...
%                 'LineWidth',1,...
%                 'MarkerEdgeColor','r',...
%                 'MarkerFaceColor',[1 0 0],...
%                 'MarkerSize',4);
% hold off
% hold on
% plot3(laser_2D_X_bajo,laser_2D_Y_bajo,laser_2D_Z_bajo,'o',...
%                 'LineWidth',1,...
%                 'MarkerEdgeColor','r',...
%                 'MarkerFaceColor',[1 0 0],...
%                 'MarkerSize',4);
% hold off
% 
% %IMPRIMO FIGURAS
% figure(6)
% plot(laser_2D_X,laser_2D_Y,'square',...
%                 'LineWidth',1,...
%                 'MarkerEdgeColor','b',...
%                 'MarkerFaceColor',[0 0 1],...
%                 'MarkerSize',3);
% grid on
% xlabel('X [m]')
% ylabel('Y [m]')
% title('LASER 2D')
% 
% figure(9)
% plot(laser_2D_X_alto,laser_2D_Y_alto,'square',...
%                 'LineWidth',1,...
%                 'MarkerEdgeColor','r',...
%                 'MarkerFaceColor',[1 0 0],...
%                 'MarkerSize',3);
% grid on
% xlabel('X alto [m]')
% ylabel('Y alto [m]')
% title('LASER 2D nivel alto')
% 
% 
 figure(8)
 plot4(X_local,Y_local,Z_local,Z_local,'o',...
       'MarkerSize',1);
 grid on
 xlabel('X [cm]')
 ylabel('Y [cm]')
 zlabel('Z [cm]')
 title('LASER 3D')
 hold on
 plot3(0,0,0,'o',...
                 'LineWidth',1,...
                 'MarkerEdgeColor','k',...
                 'MarkerFaceColor',[0 0 0],...
                 'MarkerSize',10);
 hold off           
 hold on
 plot3(A,B,C,'-r',...
                 'LineWidth',2,...
                 'MarkerEdgeColor','r',...
                 'MarkerFaceColor',[1 0 0],...
                 'MarkerSize',8);
 hold off
% % hold on
% % plot3(X_local_suelo,Y_local_suelo,Z_local_suelo,'o',...
% %                 'LineWidth',1,...
% %                 'MarkerEdgeColor','k',...
% %                 'MarkerFaceColor',[0 0 0],...
% %                 'MarkerSize',2);
% % hold off
% hold on
% plot3(X_local_techo,Y_local_techo,Z_local_techo,'o',...
%                 'LineWidth',1,...
%                 'MarkerEdgeColor','k',...
%                 'MarkerFaceColor',[0 0 0],...
%                 'MarkerSize',4);
% hold off
% hold on
% plot3(laser_2D_X,laser_2D_Y,laser_2D_Z,'o',...
%                 'LineWidth',1,...
%                 'MarkerEdgeColor','r',...
%                 'MarkerFaceColor',[1 0 0],...
%                 'MarkerSize',4);
% hold off
% hold on
% plot3(laser_2D_X_alto,laser_2D_Y_alto,laser_2D_Z_alto,'o',...
%                 'LineWidth',1,...
%                 'MarkerEdgeColor','r',...
%                 'MarkerFaceColor',[1 0 0],...
%                 'MarkerSize',4);
% hold off
% hold on
% plot3(laser_2D_X_bajo,laser_2D_Y_bajo,laser_2D_Z_bajo,'o',...
%                 'LineWidth',1,...
%                 'MarkerEdgeColor','r',...
%                 'MarkerFaceColor',[1 0 0],...
%                 'MarkerSize',4);
% hold off
  %                 'LineWidth',1,...
%                 'MarkerEdgeColor','k',...
%                 'MarkerFaceColor',[0 0 0],...
               

% %elimino datos que no veria el laser
% k=1
% v=1
% 
% u=1
% 
% while (k<(j-2))
%     u=k+1
%     while (Matriz_esferica_ord(k,1)==Matriz_esferica_ord(u,1)&& Matriz_esferica_ord(k,2)==Matriz_esferica_ord(u,2) && u<j)
%         u=u+1
%     end
%     if ((Matriz_esferica_ord(k,1)>= 0)&&(Matriz_esferica_ord(k,1)<=180))
%         if ((Matriz_esferica_ord(k,2)>= phimin)&&(Matriz_esferica_ord(k,2)<= phimax))
%             laser_X(v,1)=Matriz_esferica_ord(k,4)+ random('Normal',0,0.01)
%             laser_Y(v,1)=Matriz_esferica_ord(k,5)+ random('Normal',0,0.01)
%             laser_Z(v,1)=Matriz_esferica_ord(k,6)+ random('Normal',0,0.01)
%             v=v+1
%         end
%     end
%     k=u
% end

end