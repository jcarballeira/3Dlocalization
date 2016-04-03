function [X_laser,Y_laser,Z_laser]=laser_visual(bestmem,dist_real_3d,num_medidas,num_barridos,incr_theta,incr_phi)
%--------------------------------------------------------------------------
%   Main Function: laser_visual
%   Author: Fernando Martin Monar.
%   Date: December, 2010
%--------------------------------------------------------------------------
% -> Description: Distances in dist_real_3d are transformed to Cartesian
% coordinates.
%--------------------------------------------------------------------------
% -> Inputs:
%       -bestmem: Vector of D+1 elements containing the solution of the
%       global localization filter (robot's location) and its cost value in
%       the first element.
%       -dist_real_3d: Matrix with dimensions num_barridos*num_medidas
%       containing the distances of the laser measurements.
%       -num_medidas: Number of horizontal measurements.
%       -num_barridos: Number of vertical scans.
%       -incr_theta: Laser horizontal resolution.
%       -incr_phi: Laser vertical resolution.
% -> Outputs:
%       X_laser: vector with X Cartesian coordinates.
%       Y_laser: vector with Y Cartesian coordinates.
%       Z_laser: vector with Z Cartesian coordinates.
%--------------------------------------------------------------------------
% -> See also: plot_results local_3D_real
%--------------------------------------------------------------------------
phi=-40;  %grados
theta=bestmem(5)+180;%-90; %grados
ind=1;
cell_size=0.121;
s_range=10/cell_size;
X_laser=zeros(1,(num_barridos*num_medidas));
Y_laser=zeros(1,(num_barridos*num_medidas));
Z_laser=zeros(1,(num_barridos*num_medidas));

for i=1:num_barridos
    for j=1:num_medidas        
        if (dist_real_3d(i,j)<s_range)
            X_laser(ind)=bestmem(2) + dist_real_3d(i,j)*cos(theta*pi/180);
            Y_laser(ind)=bestmem(3) + dist_real_3d(i,j)*cos(phi*pi/180)*sin(theta*pi/180);
            Z_laser(ind)=bestmem(4) + dist_real_3d(i,j)*sin(phi*pi/180)*sin(theta*pi/180);
%             X_laser(ind)=bestmem(2) + dist_real_3d(i,j)*cos(phi*pi/180)*(-cos(theta*pi/180));
%             Y_laser(ind)=bestmem(3) + dist_real_3d(i,j)*cos(phi*pi/180)*sin(theta*pi/180);
%             Z_laser(ind)=bestmem(4) + dist_real_3d(i,j)*sin(phi*pi/180);
        end        
        ind=ind+1;
        theta=theta-incr_theta;   
    end
    phi=phi+incr_phi;
    theta=bestmem(5)+180;%-90;
end

end
