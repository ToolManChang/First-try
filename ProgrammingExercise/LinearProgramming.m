function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%exculde 0 state
P_1 = P;
P = zeros(K-1,K-1,5);
G = G([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end],:);
for i = 1:5
    tmp = P_1([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end],:,i);
    P(:,:,i) = tmp(:,[1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end]);
end

%define A and b for constraints
A = [];
b = [];
for i = 1:5
    A_u = eye(K-1)-P(:,:,i);
    b_u = G(:,i);
    A = [A;A_u];
    b = [b;b_u];
end

%exclude inf
i=1;
while i<=length(b)
    if(b(i)==inf)
        A(i,:)=[];
        b(i)=[];
        i=i-1;
    end
    i=i+1;
end

%solve the linear programming problem
V = linprog(-ones(K-1,1),A,b);

%Get the optimal policy
q = zeros(K-1,5);
for i=1:5
    q(:,i) = G(:,i)+P(:,:,i)*V;
end
[~,u_opt] = min(q,[],2);

%putback 0 state
J_opt = zeros(K,1);
J_opt([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end]) = V; 
J_opt(TERMINAL_STATE_INDEX)=0;
u_opt_ind = zeros(K,1);
u_opt_ind([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end]) = u_opt;
u_opt_ind(TERMINAL_STATE_INDEX)=HOVER;
end

