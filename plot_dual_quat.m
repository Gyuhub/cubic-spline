clear all;
pkg load geometry;
des= dlmread("des.txt");
cur= dlmread("ori.txt");
sz = size(des);
time(1:sz(1),1) = des(:,5);
dt = 0.002;
init_x = [cur(1,1); cur(1,2); cur(1,3)];

figure
  grid on;
  hold on;
  subplot(2,2,1);
  plot(time(:,1),des(:,1),"linewidth",3);
  hold on;
  plot(time(:,1),cur(:,1),"linewidth",3);
  hold on;
  legend("des x","cur x");
  subplot(2,2,2);
  plot(time(:,1),des(:,2),"linewidth",3);
  hold on;
  plot(time(:,1),cur(:,2),"linewidth",3);
  hold on;
  legend("des y","cur y");
  subplot(2,2,3);
  plot(time(:,1),des(:,3),"linewidth",3);
  hold on;
  plot(time(:,1),cur(:,3),"linewidth",3);
  hold on;
  legend("des z","cur z");
  subplot(2,2,4);
  plot(time(:,1),des(:,4),"linewidth",3);
  hold on;
  plot(time(:,1),cur(:,4),"linewidth",3);
  hold on;
  legend("des w","cur w");
  