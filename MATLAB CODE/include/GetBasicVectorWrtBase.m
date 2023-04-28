function [r]=GetBasicVectorWrtBase(biTei, linkNumber)
%%% GetBasicVectorWrtBase function 
% input :
% iTj trasnformation matrix in between i and j 
% output
% r : basic vector from i to j
%extract from iTj  translation part


res = GetTransformationWrtBase(biTei, linkNumber);

r = res(1:3,4);

end