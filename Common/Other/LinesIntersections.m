function pi=LinesIntersections(p0,p1,p2,p3)
p02=p0-p2;
p32=p3-p2;
p10=p1-p0;
num=(dot(p02,p32)*dot(p32,p10))-(dot(p02,p10)*dot(p32,p32));
denom=(dot(p10,p10)*dot(p32,p32))-(dot(p32,p10)*dot(p32,p10));
lamda=num/denom;
num=dot(p02,p32)+lamda*dot(p32,p10);
denom=dot(p32,p32);
mu=num/denom;
A=p0+lamda*p10;
B=p2+mu*p32;
pi=(A+B)/2;
end