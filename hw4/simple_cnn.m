function output = simple_cnn(vars, n_epoch, rate, p, test_intv)
%SIMPLE_CNN Performs simple CNN classification of two types of symbols
%    INPUTS
%     vars: Structure containing relevant variables
%  n_epoch: number of epochs to compute
%     rate: learning rate
%        p: pooling size/step
%test_intv: number of training epochs to pass between testing validation
%   OUTPUTS
%   output: structure containing error and classification results

% Extract inputs from struct
train_cell      = vars.train_cell;
train_labels    = vars.train_labels;
test_cell       = vars.test_cell;
test_labels     = vars.test_labels;
k_cell          = vars.k_cell;
weights         = vars.weights;

% Build arrays to store errors and number of correct classifications
train_epoch_err = zeros(n_epoch, 1);
tr_num_corr_arr = zeros(n_epoch, 1);
test_epoch_err  = zeros(floor(n_epoch/test_intv) + 1, 2);
ts_num_corr_arr = zeros(floor(n_epoch/test_intv) + 1, 2);
ts_i = 1;   % Test iterator

% Because we are only training final weights, pre-compute FCNs
train_fconv = cell(size(train_cell));
test_fconv = cell(size(test_cell));
for i = 1:numel(train_cell)
    train_fconv{i} = cnn_fwd(train_cell{i}, k_cell, p);
end
for i = 1:numel(test_cell)
    test_fconv{i} = cnn_fwd(test_cell{i}, k_cell, p);
end


% Run cnn training/testing
for i = 1:n_epoch
    % Initialize training error and correct classification vars for iteration
    train_err = zeros(numel(train_cell), 1);
    tr_num_corr = 0;
    
    % Train cnn for each training image
    for j = 1:numel(train_cell)
        % Pull training fully connected layer
        fconv = train_fconv{j};
        
        % Apply backpropagation, compute mean-squared error
        [weights, tn_err] = backprop_cnn(fconv,weights,train_labels(j,:),rate);
        train_err(j) = sum(tn_err.^2);
        
        % Check training classifications
        tr_guess = fconv'*weights; % complete ...
        tr_num_corr = tr_num_corr + eval_class(tr_guess, train_labels(j,:));
    end
    
    % Regularly compute testing performance
    if (mod(i, test_intv) == 0) || (i == 1)
        % Initialize testing error and correct classification vars
        test_err = zeros(numel(test_cell), 1);
        ts_num_corr = 0;
        
        % Evaluate accuracy with testing set (no backprop)
        for j = 1:numel(test_cell)
            % Pull testing fully connected layer
            fconv = test_fconv{j};
            
            % Compute estimate for testing image and compute mean-squared error
            ts_guess = fconv'*weights; % complete ...
            ts_err = test_labels(j,:)-ts_guess;
            test_err(j) = sum(ts_err.^2);
            
            % Check testing classifications
            ts_num_corr = ts_num_corr + eval_class(ts_guess, test_labels(j,:));
        end
        
        % Store testing errors/classification accuracy and epochs
        test_epoch_err(ts_i,:) = [i, mean(test_err)];
        ts_num_corr_arr(ts_i,:) = [i, ts_num_corr];
        ts_i = ts_i + 1;    % Iterate testing evaluation index
    end
    
    % Store training errors/classification accuracy
    train_epoch_err(i) = mean(train_err);
    tr_num_corr_arr(i) = tr_num_corr;
end

% Build output structure
output.train_epoch_err = train_epoch_err;
output.test_epoch_err  = test_epoch_err;
output.tr_num_corr_arr = tr_num_corr_arr;
output.ts_num_corr_arr = ts_num_corr_arr;
output.weights = weights;

figure
hold on
plot(1:n_epoch, train_epoch_err, 'r');
plot(test_epoch_err(:,1), test_epoch_err(:,2), 'b');
legend('train error', 'test error');

figure
hold on
plot(1:n_epoch, (numel(train_cell)-tr_num_corr_arr)./numel(train_cell), 'r');
plot(ts_num_corr_arr(:,1), (numel(test_cell)-ts_num_corr_arr(:,2))./numel(test_cell), 'b');
legend('train loss', 'test loss');

end

%------------------------------------------------------------------------------%
%---------------------------Add Helper Functions Here--------------------------%
%------------------------------------------------------------------------------%
function y=relu(x)
    y=max(0,x);
end

function Y=max_pooling(X,p)
    [m,n]=size(X);
    M=ceil(m/p)*p;
    N=ceil(n/p)*p;
    Xp=padarray(X,[(M-m) (N-n)],0,'post');
    
    Y=zeros(M/p, N/p);
    for i=1:(M/p)
        for j=1:(N/p)
            Y(i,j)=max(max(Xp(((i-1)*p+1):(i*p),((j-1)*p+1):(j*p))));
        end
    end
    
end

function output_vector=cnn_fwd(train_cell, k_cell, p)

    all_result = cell(numel(k_cell),1);

    for i = 1:numel(k_cell)
        curr_img=train_cell;
        
        curr_img=max_pooling(relu(imfilter(curr_img, k_cell{i})),p);
        curr_img=max_pooling(relu(imfilter(curr_img, k_cell{i})),p);
        curr_img=max_pooling(relu(imfilter(curr_img, k_cell{i})),p);
        
        %norm
        all_sum = sum(sum(curr_img));
        if all_sum>0
            curr_img=curr_img./all_sum;
        end
        
        all_result{i}=curr_img;
    end

    %combineNormalizedLayers
    output_vector=zeros(0);
    
    for i = 1:numel(k_cell)
        output_vector=[output_vector;reshape(all_result{i},[],1)];
    end
    
end

function [weights, tn_err] = backprop_cnn(fconv,weights,train_labels,rate)
    tn_out=fconv'*weights;
    tn_err=train_labels-tn_out;
    weights=weights+rate*fconv*tn_err;
end

function result=eval_class(tr_guess, label)
    [~,guess_idx]=max(tr_guess);
    [~,ground_truth]=max(label);
    if guess_idx==ground_truth
        result=1;
    else
        result=0;
    end
    
end