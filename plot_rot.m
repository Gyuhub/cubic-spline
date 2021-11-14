clear all;
rotmat = dlmread("rotmat.txt");
sz = size(rotmat);
time(1:sz(1),1) = rotmat(:,1);
for i=1:3
  col1(1:sz(1),i) = rotmat(1:sz(1),3*i-1);
  col2(1:sz(1),i) = rotmat(1:sz(1),3*i);
  col3(1:sz(1),i) = rotmat(1:sz(1),3*i+1);
end
dt = 0.002;
init_x = [1; 0; 0];
j = 1;
x = init_x;

for t=0:dt:time(2001,1)
  title(0);
  rmat = [col1(3*(j-1)+1,1), col2(3*(j-1)+2,1), col3(3*(j-1)+3,1); col1(3*(j-1)+1,2), col2(3*(j-1)+2,2), col3(3*(j-1)+3,2); col1(3*(j-1)+1,3), col2(3*(j-1)+2,3), col3(3*(j-1)+3,3)];
  x = rmat * x;
  hold on;
  xlabel("x");
  ylabel("y");
  zlabel("z");
  plot3(x(1), x(2), x(3));
%  drawnow;
end