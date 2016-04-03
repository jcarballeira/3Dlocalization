function pop=completa_pob(NP,NPaux,D,mapmin,mapmax,Mapa_3D,err_dis,num_medidas,num_barridos,incr_theta,incr_phi,version,dist_real_3d)
% if version=6 and the population set has to be increased, this function
% generates new candidates to complete the population.
poppre=zeros(NPaux-NP,D);
cost=zeros(NPaux-NP,1);

for i=1:(NPaux-NP)
   poppre(i,:) = mapmin + rand(1,D).*(mapmax- mapmin);
end
for i=1:(NPaux-NP)
    for j=1:D
        if poppre(i,j)<mapmin(j), poppre(i,j)=mapmin(j);end
        if poppre(i,j)>mapmax(j), poppre(i,j)=mapmax(j);end
    end
end
for i=1:(NPaux-NP)
    new_distmat=dist_est_3d(poppre(i,:),Mapa_3D,mapmax,mapmin,err_dis,num_medidas,num_barridos,incr_theta,incr_phi);
    cost(i,1)=fitness_3d(dist_real_3d,new_distmat,num_medidas,num_barridos,version);
end
pop=[cost poppre];