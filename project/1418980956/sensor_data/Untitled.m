T = 5*60;

Ta = 0.05;
ta = (0:Ta:T)'.*10^9;
na = length(ta);
a  = repmat([0 1 9.81 0 0 0], na, 1);

Tw = 0.05;
tw = (0:Tw:T)'.*10^9;
nw = length(tw);
w  = repmat([0 0 0.1 0 0 0], nw, 1);

fID = fopen('accelerometer_data.dat','w');
for i = 1:na
    fprintf(fID,'%d\t%u\t%f\t%f\t%f\t%f\t%f\t%f\n',i,ta(i),a(i,1),a(i,2),a(i,3),a(i,4),a(i,5),a(i,6));
end
fclose(fID);

fID = fopen('gyroscope_data.dat','w');
for i = 1:nw
    fprintf(fID,'%d\t%u\t%f\t%f\t%f\t%f\t%f\t%f\n',i,tw(i),w(i,1),w(i,2),w(i,3),w(i,4),w(i,5),w(i,6));
end
fclose(fID);

