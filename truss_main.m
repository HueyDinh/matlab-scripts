node_data = readmatrix("node_data.csv");
unkown_list = readmatrix("unknown_list.csv");
f_x = node_data(:,4);
f_y = node_data(:,5);
f_xy = [f_x,f_y]';
lin_force_v = reshape(f_xy,[],1);
disp("This is great.");