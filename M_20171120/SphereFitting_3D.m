function [ CenterP, r ] = SphereFitting_3D(p1,p2,p3,p4)
%CIRCLEFITTING 
% input: x,y,z 
% output cx 
%        cy
%        cz
%        r
% 
% 
% http://imagingsolution.blog107.fc2.com/blog-entry-16.html
% Shere: (x-a)^2+(y-b)^2+(z-c)^2=r^2
%

if ( length(p1)<3 | length(p2)<3 | length(p3)<3 | length(p4)<3 )
    error ('variables are not enough');
end

syms a b c r;

if det([p1 1;p2 1;p3 1;p4 1])==0 % in case of four points on a single surface 
    A = [a b c 1;p1 1;p2 1;p3 1];
    Eqn = [(a-p1(1))^2+(b-p1(2))^2+(c-p1(3))^2-r^2 == 0,
           (a-p2(1))^2+(b-p2(2))^2+(c-p2(3))^2-r^2 == 0,
           (a-p3(1))^2+(b-p3(2))^2+(c-p3(3))^2-r^2 == 0,
           (a-p4(1))^2+(b-p4(2))^2+(c-p4(3))^2-r^2 == 0,
           det(A)==0];

else
    Eqn = [(a-p1(1))^2+(b-p1(2))^2+(c-p1(3))^2-r^2 == 0,
           (a-p2(1))^2+(b-p2(2))^2+(c-p2(3))^2-r^2 == 0,
           (a-p3(1))^2+(b-p3(2))^2+(c-p3(3))^2-r^2 == 0,
           (a-p4(1))^2+(b-p4(2))^2+(c-p4(3))^2-r^2 == 0];
end
   
S = solve ( Eqn, [a, b, c, r] );


CenterP = double([S.a(1) S.b(1) S.c(1)]);
r = abs(double(S.r(1)));

end