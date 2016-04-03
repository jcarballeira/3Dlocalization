function plot_results(Mapa_3D,poblacion,posicion,bestmem,dist_real_3d,num_medidas,num_barridos,incr_theta,incr_phi)
%--------------------------------------------------------------------------
%   Main Program: plot_results
%   Author: Fernando Martin Monar.
%   Date: December, 2010
%--------------------------------------------------------------------------
% -> Description: The results after applying the Global Localization filter
% are plotted.
%--------------------------------------------------------------------------
% -> Inputs: 
%       -Mapa_3D: Matrix that contains the 3D map of the environment.
%       -poblacion: population set. Matrix with dimensions NP*(D+1) where
%       each row contains a candidate with an associated cost value (first
%       column).
%       -posicion: Robot's true pose. Vector coordinates are given in
%       Cartesian coordinates and plane orientation (4 dof).
%       -bestmem: Vector of D+1 elements containing the solution of the
%       global localization filter (robot's location) and its cost value in
%       the first element.
%       -dist_real_3d: Matrix with dimensions num_barridos*num_medidas
%       containing the distances of the laser measurements.
%       -num_medidas: Number of horizontal measurements.
%       -num_barridos: Number of vertical scans.
%       -incr_theta: Laser horizontal resolution.
%       -incr_phi: Laser vertical resolution.
%--------------------------------------------------------------------------
% -> See also: local_3D_real laser_visual
%-------------------------------------------------------------------------- 

[X_laser,Y_laser,Z_laser]=laser_visual(bestmem,dist_real_3d,num_medidas,num_barridos,incr_theta,incr_phi);

% figure(1)
% hold on
% plot(posicion(2),posicion(1),'square',...
%                  'LineWidth',1,...
%                  'MarkerEdgeColor','b',...
%                  'MarkerFaceColor',[0 0 1],...
%                  'MarkerSize',2);
% hold off       
% xlabel('Y [cm]')
% ylabel('X [cm]')
% title('MAPA')


figure(2);
p = patch(isosurface(Mapa_3D,0.5));
set(p,'FaceColor','red','EdgeColor','none');
daspect([1 1 1])
view(3); axis tight
camlight
lighting gouraud

figure(2)
hold on
plot3(posicion(2),posicion(1),posicion(3),'o',...
                 'LineWidth',1,...
                 'MarkerEdgeColor','k',...
                 'MarkerFaceColor',[0 0 0],...
                 'MarkerSize',10);
hold off  
figure(2)
hold on
plot3(bestmem(3),bestmem(2),bestmem(4),'o',...
                 'LineWidth',1,...
                 'MarkerEdgeColor','k',...
                 'MarkerFaceColor',[0 0 0],...
                 'MarkerSize',6);
hold off  
figure(2)
hold on
plot3(Y_laser,X_laser,Z_laser,'o',...
                 'LineWidth',1,...
                 'MarkerEdgeColor','b',...
                 'MarkerFaceColor',[1 0 0],...
                 'MarkerSize',2);
hold off 
figure(2)
hold on
plot3(poblacion(:,3),poblacion(:,2),poblacion(:,4),'o',...
                 'LineWidth',1,...
                 'MarkerEdgeColor','g',...
                 'MarkerFaceColor',[0 1 0],...
                 'MarkerSize',4);
hold off
xlabel('Y [cm]')
ylabel('X [cm]')
zlabel('Z [cm]')
title('3D MAP')

figure(3)
hold on
plot3(Y_laser,X_laser,Z_laser,'o',...
                 'LineWidth',1,...
                 'MarkerEdgeColor','b',...
                 'MarkerFaceColor',[1 0 0],...
                 'MarkerSize',2);
hold off 

end
