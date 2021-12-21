clear all;
pkg load geometry;
rotmat = dlmread("rot.txt");
ori = dlmread("ori.txt");
sz = size(rotmat);
time(1:sz(1),1) = rotmat(:,1);
for i=1:3
  col1(1:sz(1),i) = rotmat(1:sz(1),i+1);
  col2(1:sz(1),i) = rotmat(1:sz(1),i+4);
  col3(1:sz(1),i) = rotmat(1:sz(1),i+7);
end
dt = 0.002;
init_x = [ori(1,1); ori(1,2); ori(1,3)];
%init_x = [1; 0; 0];
j = 1;
x = init_x;
y = x;
for t=time(1, 1):dt:time(sz(1),1)
  rmat = [col1(j,1), col2(j,1), col3(j,1); col1(j,2), col2(j,2), col3(j,2); col1(j,3), col2(j,3), col3(j,3)];
  y = rmat * x;
  res(j,1) = y(1);
  res(j,2) = y(2);
  res(j,3) = y(3);
  j=j+1;
end
%len(1:sz(1),1) = sqrt (ori(:,1).^2 + ori(:,2).^2 + ori(:,3).^2);
%ori(:,1) ./= len;  ori(:,2) ./= len;  ori(:,3) ./= len;
%figure
%  grid on;
%  hold on;
%  xlabel("x");
%  ylabel("y");
%  zlabel("z");
%  plot3(res(:,1), res(:,2), res(:,3) ,"linewidth", 2, "linestyle", ':');
%  hold on;
%  plot3(res(1:501,1), res(1:501,2), res(1:501,3) ,"linewidth", 2, "linestyle",'-.');
%  hold on;
%  plot3(res(501:1001,1), res(501:1001,2), res(501:1001,3) ,"linewidth", 2, "linestyle",'-.');
%  hold on;
%  plot3(res(1001:1501,1), res(1001:1501,2), res(1001:1501,3) ,"linewidth", 2, "linestyle",'-.');
%  hold on;
%  plot3(res(1501:2001,1), res(1501:2001,2), res(1501:2001,3) ,"linewidth", 2, "linestyle",'-.');
%  hold on;
%  plot3(res(1,1),res(1,2),res(1,3), '-o', "linewidth",10);
%  hold on;
%  plot3(res(501,1),res(501,2),res(501,3), '-o', "linewidth",6);
%  hold on;
%  plot3(res(1001,1),res(1001,2),res(1001,3), '-o', "linewidth",6);
%  hold on;
%  plot3(res(1501,1),res(1501,2),res(1501,3), '-o', "linewidth",6);
%  hold on;
%  plot3(res(2001,1),res(2001,2),res(2001,3), '-o', "linewidth",6);
%  legend("x", "x=0:1", "x=1:2", "x=2:3", "x=3:4", "t=0", "t=1", "t=2", "t=3", "t=4");
%  view([-37.5 150]);

%figure
%grid on;
%hold on;
%comet3(res(:,1),res(:,2),res(:,3),dt);
%comet3(ori(:,1),ori(:,2),ori(:,3),dt);
%figure
%  grid on;
%  hold on;
%  xlabel("x");
%  ylabel("y");
%  zlabel("z");
%  plot3(res(:,1), res(:,2), res(:,3) ,"linewidth", 2, "linestyle", ':');
%  hold on;
%  plot3(ori(:,1), ori(:,2), ori(:,3) ,"linewidth", 2, "linestyle", ':');
%  hold on;
%  legend("res","ori");
%  view([-37.5 150]);