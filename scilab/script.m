names = ["history_iCub_fixed_iCub_l_hand";
         "history_iCub_fixed_iCub_r_hand"];
[dim1, dim2] = size(names);

for (k = 1:(dim1*dim2))
  // data generated by the Sensor Plugin
  data = fscanfMat(strcat([names(k), ".txt"]));
  // force plot
  subplot(dim1, 2*dim2, 2*k-1);
  plot(data(:,1), data(:,6), 'c', data(:,1), data(:,2), 'b');
  set(gca(),'grid',[1 1]);
  title(names(k));
  xlabel('time (s)');
  ylabel('force (magnitude) (N)');
  legend('original', 'filtered');
  // components plot
  subplot(dim1, 2*dim2, 2*k);
  plot(data(:,1), data(:,3), data(:,1), data(:,4), data(:,1), data(:,5));
  set(gca(),'grid',[1 1]);
  title(names(k));
  xlabel('time (s)');
  ylabel('force (component) (N)');
  legend('x', 'y', 'z');
end
