function pop=inicio_pob(NP,D,mapmin,mapmax)
% Initial population generation. A random population of NP candiates, each
% one with D chromosomes, in randomly generated to cover the whole map
% according to the map limits (mapmin and mapmax.
poppre=zeros(NP,D);
cost=zeros(NP,1);
for i=1:NP
   poppre(i,:) = mapmin + rand(1,D).*(mapmax- mapmin);
end
for i=1:NP
    for j=1:D
        if poppre(i,j)<mapmin(j), poppre(i,j)=mapmin(j);end
        if poppre(i,j)>mapmax(j), poppre(i,j)=mapmax(j);end
    end
end
pop=[cost poppre];