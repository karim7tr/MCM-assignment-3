function [theta,v] = ComputeInverseAngleAxis(R)
[a, b]=size(R);
  M=(R-R')/2;
   vex=[0.5*(M(3,2)-M(2,3));0.5*(M(1,3)-M(3,1));0.5*(M(2,1)-M(1,2))];
     if a==3 && b==3
         if abs(1-diag(R*R'))<=1e-4
           if abs(1-det(R))<=1e-4
               theta=acos((trace(R)-1)/2);
               [ev,evl]=eig(R);
               e=abs(1-diag(evl))<=1e-4;
               v=ev(:,e);
            else
              error('DETERMINANT OF THE INPUT MATRIX IS 0')
            end
        else
           error('NOT ORTHOGONAL INPUT MATRIX')
          end
    else
       error('WRONG SIZE OF THE INPUT MATRIX')
 
     end
end


