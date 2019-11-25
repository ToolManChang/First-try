function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%initial policy
u_0 = ones(K-1,1)*5;

%exculde 0 state
P_1 = P;
P = zeros(K-1,K-1,5);
G = G([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end],:);
for i = 1:5
    tmp = P_1([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end],:,i);
    P(:,:,i) = tmp(:,[1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end]);
end

%%get transition prob and cost for current policy
P_u = zeros(K-1,K-1);
G_u = zeros(K-1,1);
for k = 1:K-1
    P_u(k,:) = P(k,:,u_0(k));
    G_u(k) = G(k,u_0(k));
end

%solve J for current policy
J = (eye(K-1)-P_u)\G_u;

%find best action among u (new policy)
J_u = zeros(K-1,5);
for i = 1:5
    J_u(:,i) = G(:,i)+P(:,:,i)*J;
end
[~,u] = min(J_u,[],2);

J_0 = zeros(K-1,5);

while(norm(J_0-J)>0.0001)
    J_0 = J;
    u_0=u;
    P_u = zeros(K-1,K-1);
    G_u = zeros(K-1,1);
    for k = 1:K-1 %calculate transition matrix of current policy
        P_u(k,:) = P(k,:,u(k));
        G_u(k) = G(k,u(k));
    end
    
    %solve J if it can be solved   
    J = (eye(K-1)-P_u)\G_u;
   
    %find best action among u (new policy)
    for i = 1:5
        J_u(:,i) = G(:,i)+P(:,:,i)*J;
    end
    [~,u] = min(J_u,[],2); 
end

%add terminal state back
J_opt = zeros(K,1);
J_opt([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end]) = J; 
J_opt(TERMINAL_STATE_INDEX)=0;
u_opt_ind = zeros(K,1);
u_opt_ind([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:end]) = u;
u_opt_ind(TERMINAL_STATE_INDEX)=HOVER;
end
