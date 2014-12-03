
function y = tukey_fce(x,c)
y           = (1/6*c.^2).*(1-(1-(x./c).^2).^3);
y(abs(x)>c) = (1/6*c.^2);
end


