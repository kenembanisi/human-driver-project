[x,y,z]=sphere(100);
XX=zeros(size(x,1),size(x,2));
YY=zeros(size(x,1),size(x,2));
ZZ=zeros(size(x,1),size(x,2));
TarP=[1,1,1]; TarR=1;

for i=1:size(x,1)
    for j=1:size(x,2)
        D=sqrt((x(i,j)-TarP(1))^2.+(y(i,j)-TarP(2))^2.+(z(i,j)-TarP(3))^2.);
        if (D<TarR)
            XX(i,j)=x(i,j);
            YY(i,j)=y(i,j);
            ZZ(i,j)=z(i,j);
        else
            XX(i,j)=(x(i,j)-TarP(1))/D+TarP(1);
            YY(i,j)=(y(i,j)-TarP(2))/D+TarP(2);
            ZZ(i,j)=(z(i,j)-TarP(3))/D+TarP(3);
        end
    end
end
