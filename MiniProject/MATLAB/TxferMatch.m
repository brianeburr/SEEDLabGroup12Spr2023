%%Motor Matching Script for Mini Project

%Include variable x with column 1 containing position data in encoder
%count and colum 2 containing time stamps in milliseconds.

dx = diff(x(:,1))./diff(x(:,2));  %Currently in encoder counts per millisecond. Lets get this into radians per second.
dx2 = diff(x2(:,1))./diff(x2(:,2));

%  counts / ms   *    1000ms  /  sec    *    2pi radians / 1600 counts

dx = dx * 1000 * 2 * pi ./ 1600;
dx2 = dx2 * 1000 * 2 * pi ./ 1600; %Now dx is in radians per second.

time = (x(:,2) - x(1,2)) ./ 1000;
time2 = (x2(:,2) - x2(1,2)) ./ 1000;

K = 11.5/255;  %this K and sig value match our transfer function.
sig = 13;

open_system('TransferFxnMatch.slx')
out = sim("TransferFxnMatch.slx");

figure(1)
hold on
plot(time, [0;dx])
plot(time2, [0;dx2])
plot(out.velocity);
plot(out.velocity255)
hold off