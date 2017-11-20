vars = load('simple_cnn_vars.mat');
n_epoch = 700;
rate = 0.01;
p = 2;
test_intv = 50;

output_cnn = simple_cnn(vars, n_epoch, rate, p, test_intv);