function [iTj_q] = GetDirectGeometry(q, biTei, linkType)
%%% GetDirectGeometryFunction

% Inputs: 
% q : links current position ; 
% iTj : vector of matrices containing the transformation matrices from link
% i to link j
% linkType: vector of size numberOfLinks identiying the joint type, 0 for revolute, 1 for
% prismatic.

% Outputs :
% iTj_q vector of matrices containing the transformation matrices from link i to link j for the input q. 
% The size of iTj is equal to (4,4,numberOfLinks)

numberOfLinks = 7 ;

for i = 1:numberOfLinks
    iTj_q (:,:,i) = DirectGeometry(q(i),biTei(:,:,i),linkType(i));
    
%disp(iTj_q(:,:,i));
end

end