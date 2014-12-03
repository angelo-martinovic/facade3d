function [retx, rety]=remPclose2Border(x,y, maxx, minx, maxy, miny)
    xm = (x<maxx);
    ym = (y<maxy);
    m = (xm&ym);
    xm = (x>minx);
    ym = (y>miny);
    m = (m&xm);
    m = (m&ym);
    retx = x(m);
    rety = y(m);
end
