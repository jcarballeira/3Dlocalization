% RSearch3D query a GL-Tree for Radius search
%
% SYNTAX
% idc=RSearch3D(p,qp,ptrtree,r);
%
% INPUT PARAMETERS
% 
%       p: [3xN] double array coordinates of reference points
% 
%       qp:  [3x1] double array  coordinates of query points.
%               NOTE: differently from NNsearch this function
%               does not support multiple query points.               
%
%       ptrtree: a pointer to the previously constructed  GLtree.Warning
%                if the pointer is uncorrect it will cause a crash, there is
%                no way to check this in the mex routine, you have to check
%                it in your script.
%
%       r: double value, radius to perform the search. 
%
%
% OUTPUT PARAMETERS
%
%   idc: [?x1] column vector, each rows contains an index of a point found in
%        the range described by the radius.
%         
%
% GENERAL INFORMATIONS
%
%         -Conditions for a point to be found in the radius is <=, so
%         points with distance from the query=r will be returned.
%
%
%For question, suggestion, bug reports
% giaccariluigi@msn.com
%
%Author: Luigi Giaccari
%
% Visit:
%<a href="http://www.advancedmcode.org/gltree-pro-version-a-fast-nearest-neighbour-library-for-matlab-and-c.html"> GLTree Home Page</a>

