function [ Angle ]= calculateInternalAngle(A,B)

C=dot(A,B);
CosAngle= dot(A,B)/(norm(A)*norm(B));
Angle = acos(CosAngle); % the angle is in radians.

end