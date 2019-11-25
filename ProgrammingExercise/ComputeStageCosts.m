function G = ComputeStageCosts( stateSpace, map )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
    
    M = size(map,1);
N = size(map,2);

%transition from x_k to x_k^1 (only apply action)
P_01 = zeros(K,K+1,5);
%transition due to the wind, add state C
P_12 = zeros(K+1,K+2,5);
%transition due to the shooter, add state C, T
P_23 = zeros(K+2,K+2,5);
%transition due to the shooter, add state C, T
P_3T = zeros(K+2,K,5);

%buld index map
indexmap = zeros(M,N,2);
for k = 1:K
    m = stateSpace(k,1);
    n = stateSpace(k,2);
    phi = stateSpace(k,3);
    indexmap(m,n,phi+1)=k;
end

%build shoot range map (whether in the shooting range)
shootmap = zeros(M,N);
[m_s,n_s] = find(map==SHOOTER);
for m=1:M
    for n=1:N
        for s = 1:length(m_s)
            if abs(m-m_s(s))+abs(n-n_s(s))<=R
                shootmap(m,n)=1;
            end
        end
    end
end

%build u matrix
u = zeros(5,2);
u(1,:) = [0,1];
u(2,:) = [0,-1];
u(3,:) = [1,0];
u(4,:) = [-1,0];
u(5,:) = [0,0];


%build P_01
for g = 1:5
    for i = 1:K
        if(i==TERMINAL_STATE_INDEX)
           P_01(i,K+1,g)=1;
        else
            %take one action
            tmp_state = stateSpace(i,:)+[u(g,:),0];
            m = tmp_state(1);
            n = tmp_state(2);
            phi = tmp_state(3);
            if(m>M || n>N || m<1 || n<1 || m>0&&n>0&&m<=M&&n<=N&&map(m,n)==TREE)%if crash
                P_01(i,i,g)=0;%nothing happen
            else %not crash
                %find the index of tmp_state
                tmp_index = indexmap(m,n,phi+1);
                P_01(i,tmp_index,g)=1;
            end
        end
        
    end
end

P_01_1 = P_01(:,:,1);

%build P_12: K+1:T, K+2:C
for g = 1:5
    for i = 1:K+1
        if(i==K+1)%x_k^1 at T
            P_12(i,K+1,g)=1;
        else
            for a = 1:4 %influenced by wind
                tmp_state = stateSpace(i,:)+[u(a,:),0];
                m = tmp_state(1);
                n = tmp_state(2);
                phi = tmp_state(3);
                if(m>M || n>N || m<1 || n<1 || m>0&&n>0&&m<=M&&n<=N&&map(m,n)==TREE)%crash
                    P_12(i,K+2,g)=P_12(i,K+2,g)+P_WIND/4;
                else%not crash
                    %find the index of tmp_state
                    tmp_index = indexmap(m,n,phi+1);
                    P_12(i,tmp_index,g)=P_WIND/4;
                end                
            end
            %not influnced by the wind
            P_12(i,i,g)=1-P_WIND;
        end
    end
end

P_12_1 = P_12(:,:,1);
%build P_23: K+1:T, K+2:C
for g = 1:5
    for i = 1:K+2
        if(i==K+2) %crash
            P_23(i,K+2,g)=1;
        else
            if(i==K+1) %terminal
                P_23(i,K+1,g)=1;
            else
                m = stateSpace(i,1);
                n = stateSpace(i,2);
                phi = stateSpace(i,3);
                if(shootmap(m,n)==1)%if in the shooting range
                    P_nshot = 1;
                   for s = 1:length(m_s)
                       d = abs(m-m_s(s))+abs(n-n_s(s));%l1 distance
                        if d<=R
                            P_nshot = P_nshot*(1-GAMMA/(1+d));
                        end
                   end 
                   %not shot then stay
                   P_23(i,i,g)=P_nshot;
                   P_23(i,K+2,g)=1-P_nshot;
                else
                   P_23(i,i,g)=1;
                end
            end
        end
    end
end

P_23_1 = P_23(:,:,1);

%build P_3T: K+1:T, K+2:C
[m_p,n_p] = find(map==PICK_UP); 
[m_b,n_b] = find(map==BASE);
base_index = find(stateSpace==[m_b,n_b,0],1);
for g = 1:5%iterate in actions
    for i = 1:K+2
        if(i==K+2)%crash
            P_3T(i,base_index,g)=1;
        else
            if(i==K+1) %terminal
                P_3T(i,TERMINAL_STATE_INDEX,g)=1;  
            else
                %at pick up state
                pick_i = indexmap(m_p,n_p,1);
                afpick_i = indexmap(m_p,n_p,2);
                if(i==pick_i)
                    P_3T(i,afpick_i,g)=1;
                else
                    P_3T(i,i,g)=1;
                end                
            end
        end
    end
end

P_3T_1 = P_3T(:,:,1);

%%%%%%%%%%%%%%%%%Above: same to prob2)
%construct cost of x_k^3
g_3 = ones(K+2,1);
g_3(K+1) = 0;
g_3(K+2) = Nc;
%compute cost of x_k^2
g_2 = P_23(:,:,1)*g_3;
%compute cost of x_k^1
g_1 = P_12(:,:,1)*g_2;
%compute cost
g = zeros(K,5);
for l = 1:5 %iterate over actions
    g(:,l) = P_01(:,:,l)*g_1;
    for k = 1:K 
        if(sum(P_01(k,:,l))==0)%if action not allowed
            g(k,l)=inf;
        end
    end
end
G = g;
end

