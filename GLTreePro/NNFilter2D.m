% KNNSearch2D query a GL-tree for k nearest neighbor(kNN)
% 
% SYNTAX
% 
% [cutdist,ndel,idel]=KNNFilter2D(p,ptrtree,'r',r);% radius mode
% [cutdist,ndel,idel]=KNNFilter2D(p,ptrtree,'f',f);% consolidator mode
% 
% INPUT PARAMETERS
% 
%       p: [2xN] double array coordinates of reference points
% 
%       ptrtree: a pointer to the previously constructed  GLtree.Warning
%                if the pointer is uncorrect it will cause a crash, there is
%                no way to check this in the mex routine, you have to check
%                it in your script.
% 
%       'r' or 'f': string. Specify the RADIUS mode or the CONSOLIDATOR mode.
% 
%        r or f: double parameter for the mode.
% 
% OUTPUT PARAMETERS
% 
%      cutdist: in the dataset no couple of points will be closer than
%               cutdist. In the radius mode cutdist==radius.
% 
%      ndel: number of points removed from the tree
% 
%      iddel: ids of points removed from the tree 
% 
% 
% 
% GENERAL INFORMATIONS
% 
%         -RADIUS mode: if 2 points are closer than r only one will
%         survive.
%         -CONSOLIDATOR mode: the nearest neighbour graph is computed and
%         the mean distance between points is evaluated. The cutdistance is
%         set to meandist/f. 
%         -points removed form the tree are not physically deleted (memory
%         deaollocated), they are just skipped during search.
%         
% 
% 
% For question, suggestion, bug reports
% giaccariluigi@msn.com
% 
% Author: Luigi Giaccari
%
% Visit:
%<a href="http://www.advancedmcode.org/gltree-pro-version-a-fast-nearest-neighbour-library-for-matlab-and-c.html"> GLTree Home Page</a>


