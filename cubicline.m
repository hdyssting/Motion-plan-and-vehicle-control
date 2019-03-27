function coff = cubicline(y0,xf,yf,n)


mat = [3*xf^2, xf^3;
       2*xf,   xf^2];
yf_temp = 0:yf/n:yf;
for i=1:length(yf_temp)
    result = [0, yf_temp(i)-y0];
    coff_temp = result/mat;
    coff(i,:) = [coff_temp,0];
end



