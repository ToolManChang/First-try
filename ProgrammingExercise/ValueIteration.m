function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
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

%exclude 0 state
P_1 = P;
P = zeros(K-1,K-1,5);
G = G([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end],:);
for i = 1:5
    tmp = P_1([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end],:,i);
    P(:,:,i) = tmp(:,[1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end]);
end

%initial value
V_0 = ones(K-1,1);
%q value (temporal value for each action)
V_u = zeros(K-1,5);

%compute V_u
for i = 1:5
    V_u(:,i) = G(:,i)+P(:,:,i)*V_0;
end
[V,u_opt] = min(V_u,[],2);

while(norm(V-V_0)>0.0001)
    V_0 = V;
    for i = 1:5
        V_u(:,i) = G(:,i)+P(:,:,i)*V_0;
    end
    [V,u_opt] = min(V_u,[],2);
end

%Put the teminal state back
J_opt = zeros(K,1);
J_opt([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end]) = V; 
J_opt(TERMINAL_STATE_INDEX)=0;
u_opt_ind = zeros(K,1);
u_opt_ind([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end]) = u_opt;
u_opt_ind(TERMINAL_STATE_INDEX)=HOVER;
end