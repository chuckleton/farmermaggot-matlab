clear;

R_earth = 6378.1370e3;
eci = [R_earth,0,0]
eci = [6.478e3,0,0]
lla = eci2lla(eci,[2010 1 17 10 20 36]);
lat = lla(1)
lon = lla(2)
alt = lla(3)