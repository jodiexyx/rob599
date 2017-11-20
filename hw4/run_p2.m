vars = load('simple_nn_vars.mat');
n_epoch = 2000;
rate = [0.04, 0.2];
test_intv = 100;

output_nn = simple_nn(vars, n_epoch, rate, test_intv);