function biTei = DirectGeometry(qi, iTj, linkType)
% DirectGeometry Function 
% inputs: 
% qi : current link position;
% biTri is the constant transformation between the base of the link <i>
% and its end-effector; 
% jointType :0 for revolute, 1 for prismatic

% output :
% biTei : transformation between the base of the joint <i> and its end-effector taking 
% into account the actual rotation/traslation of the joint

if linkType == 0 % rotational   
c = iTj(1:3,1:3)*[cos(qi) -sin(qi) 0; sin(qi) cos(qi) 0; 0 0 1];
c = [c; ones(1,3)*0];

l = iTj(:,end);

biTei = [c l];
elseif linkType == 1 % prismatic

end

end