%% Postfault Operation Strategy With Minimum Harmonic Injection for Cascaded H-Bridge Inverters Driving a Multiphase Motor Drives﻿

%% Description
% This script calculates the PWM reference with minimum xy
% (MXY) component injection for five-level cascaded H-bridge inverter
% driving a five-phase motor drive a fault when some power cells
% (H-bridges) are bypassed.
% The results are compared with the obtained with the minimum 
% infinity-norm (MIN) post fault strategy [1].
%
% *References* :
%
% [1] Ó. López et al., "Postfault Operation Strategy for Cascaded H-Bridge 
%     Inverters Driving a Multiphase Motor," in IEEE Transactions on 
%     Industrial Electronics, vol. 71, no. 5, pp. 4309-4319, May 2024, 
%     <https://ieeexplore.ieee.org/document/10144607/  doi: 10.1109/TIE.2023.3281688> .
%

%% Initialization
clearvars

% Five-phase amplitude invariant Clarke transformation:
C=2/5*[cos((0:4)*2*pi/5);...
    sin((0:4)*2*pi/5);...
    cos((0:4)*4*pi/5);...
    sin((0:4)*4*pi/5);...
    ones(1,5)/sqrt(2)];

% Matrix to calculate the alpha-beta components of a five-phase vector:
global A;
A=C(1:2,:);
% Cost matrix with unity weights for the x and y components.
global Q;
wx=1; % Weight for the x component.
wy=1; % Weight for the y component.
Q=2*C'*diag([0,0,wx,wy,0])*C;

%% Voltage reference

% Number of simulation points (i.e., number of switching cycles):
N=800;

% Sequence of reference voltage vectors in the alpha-beta frame for two
% fundamental periods:
nup=1.85; %(p.u.) Amplitude of the reference voltage in the alpha-beta frame
nur=nup*[-cos((0:4*pi/(N-1):4*pi)); sin((0:4*pi/(N-1):4*pi))];

% Number of active cells during the first fundamental period:
k1=[2;2;2;2;2]; % Any cell is bypassed.
assert(nup<1/max(uMagnitude(k1)),'The reference voltage lays in the overmodulation region');

% Number of active cells during the second fundamental period:
k2=[1;2;2;2;2]; % One cell is bypassed in phase a (e.g., cell a1)
assert(nup<1/max(uMagnitude(k2)),'The reference voltage lays in the overmodulation region');

v=zeros(5,N); % Memory allocation for the PWM reference results.
iter=zeros(1,N); % Memory allocation for the iteration number results.

%% MXY postfault strategy with cold start
v0=zeros(5,1); % The starting vector is zero.
for j=1:N
    if j<N/2 % We are in the first fundamental period.
        l=-k1;
        u= k1;
    else % We are in the second fundamental period.
        l=-k2;
        u= k2;
    end
    [v(:,j),iter(j)] = MXY(nur(:,j),l,u,v0);
    assert(iter(j)); % Check that there is no convergence error.
end

% Plots:
sgtitle('MXY postfault strategy with cold start')
subplot(4,1,1)
stairs(nur');
grid 'on'
title('\alpha\beta components of the reference voltage')
legend([{'\alpha'},{'\beta'}])
xlim([1,N])

subplot(4,1,2)
stairs(v');
grid 'on'
title('PWM reference')
legend(('a':'e')')
xlim([1,N])

subplot(4,1,3)
stairs((C*v)')
grid 'on'
title('Space vector components of the PWM reference')
legend([{'\alpha'},{'\beta'},{'x'},{'y'},{'0'}])
xlim([1,N])

subplot(4,1,4)
stairs(iter)
grid 'on'
title(['Number of iterations (Total: ',int2str(sum(iter)),')'])

xlim([1,N])

saveas(gcf, 'MXY_cold.png')

%% MXY postfault strategy with warm start
v0=zeros(5,1); % The very first switching cycle is calculated with cold start.
for j=1:N
    if j<N/2 % We are in the first fundamental period.
        l=-k1;
        u= k1;
    else % We are in the second fundamental period.
        l=-k2;
        u= k2;
    end
    [v(:,j),iter(j)] = MXY(nur(:,j),l,u,v0);
    v0=v(:,j); % Start vector for the next switching cycle (warm start)
    assert(iter(j)); % Check that there is no convergence error.
end

% Plots:
sgtitle('MXY postfault strategy with warm start')
subplot(4,1,1)
stairs(nur');
grid 'on'
title('\alpha\beta components of the reference voltage')
legend([{'\alpha'},{'\beta'}])
xlim([1,N])

subplot(4,1,2)
stairs(v');
grid 'on'
title('PWM reference')
legend(('a':'e')')
xlim([1,N])

subplot(4,1,3)
stairs((C*v)')
grid 'on'
title('Space vector components of the PWM reference')
legend([{'\alpha'},{'\beta'},{'x'},{'y'},{'0'}])
xlim([1,N])

subplot(4,1,4)
stairs(iter)
grid 'on'
title(['Number of iterations (Total: ',int2str(sum(iter)),')'])
xlim([1,N])

saveas(gcf, 'MXY_warm.png')

%% MIN postfault strategy
for j=1:N
    if j<N/2 % We are in the first fundamental period.
        k=k1;
    else % We are in the second fundamental period.
        k=k2;
    end
    v(:,j) = MIN(nur(:,j),k).*k;
end

% Plots:
sgtitle('MIN postfault strategy')
subplot(3,1,1)
stairs(nur');
grid 'on'
title('\alpha-\beta components of the reference voltage')
legend([{'\alpha'},{'\beta'}])
xlim([1,N])

subplot(3,1,2)
stairs(v');
grid 'on'
title('PWM reference')
legend(('a':'e')')
xlim([1,N])

subplot(3,1,3)
stairs((C*v)')
grid 'on'
title('Space decomposition of the output voltage')
legend([{'\alpha'},{'\beta'},{'x'},{'y'},{'0'}])
xlim([1,N])

saveas(gcf, 'MIN.png')

%% End
disp 'Script finalized successfully'.

%% Local functions

function [v,iter] = MXY(nur,l,u,v)
% Postfault operation strategy with minimum xy (MXY) injection.
% Input parameters:
% nur = voltage reference in the alpha-beta frame (2x1 column vector).
%   l = lower voltage achievable for each inverter phase (5x1 column vector).
%   u = upper voltage achievable for each inverter phase (5x1 column vector).
%   v = starting voltage vector (5x1 column vector)
% Global parameters:
%   Q  = cost matrix (5x5 matrix)
%   A  = matrix to calculate the alpha and beta components of v (2x5 matrix).
% Output results:
%   v  = PWM reference with MXY injection (5x1 column vector).
%        It is the solution of
%           minimize   v'*Q*v/2
%           subject to C*v=nur
%           and        l<=v<=u
% iter = number of iterations (iter=0 indicates convergence error).

% Initialization:
global Q;
global A;
iter=0;

% Compute L and U to be subsets of the active constraints at v:
WS=zeros(5,1); % Memory allocation for the variable that stores the working sets information.
m=0; % Number of active constraints.
for i=1:5
    if v(i)<=l(i)
        WS(i)=-1; % If WS==-1 the i in the working set L.
        v(i)=l(i); % And v(i) is clamped to the lowest achievable voltage.
        m=m+1;
    elseif v(i)>=u(i)
        WS(i)=+1;  % If WS==+1 the i in the working set U.
        v(i)=u(i); % And v(i) is clamped to the lowest achievable voltage.
        m=m+1;
    end
end

% Begin iteration process:
for k=1:7
    inLU= WS~=0;
    ninLU= WS==0;
    % Solve the equality constrained problem to find the step voltage
    % vector p:
    switch m
        case 0  % Zero active constraint. There are infinite solutions differing in their zero sequence.
            % There solution of the optimization problem is straightforward
            % considering no xy injection (value of cost function
            % q(v)=0). We chose the solution with considering generalized
            % min-max zero sequence injection:
            vx=5/2*A'*nur;
            vz=-0.5*(min(vx+u)+max(vx+l)); % Generalized min-max injection. Equation (3) in [1].
            p=vx+vz-v;
        
        case 1  % One active constraint. There is a single solution.
            % The solution of the optimization problem is straightforward
            % considering non xy injection (value of cost function
            % q(v)=0). We inject appropriate zero-sequence to clamp the
            % active constraint.
            vx=5/2*A'*nur;
            i= find(WS,1); % Active constraint
            if WS(i)==-1 % The active constraint corresponds to the lower bound.
                vz=l(i)-vx(i);
            else % The active constraint corresponds to the lower bound.
                vz=u(i)-vx(i);
            end
            p=vx+vz-v;

        case 2 % Two active constraint. There is a single solution.
            %E=I(inLU,:); % Not used
            h=A*v-nur;
            % Eb=I(ninLU,:); % Not used
            Qh=Q(ninLU,ninLU); % Qh=Eb*Q*Eb';
            Ah=A(:,ninLU); % Ah=A*Eb'
            gh=Q(ninLU,:)*v; % gh=Eb*Q*v;
            % P=eye(5-m); % Not used
            M=Ah(:,1:2);
            N=Ah(:,3:end);
            % Inverse of matrix N (2x2 matrix)
            iM=[M(2,2),-M(1,2); -M(2,1), M(1,1)]/(M(1,1)*M(2,2)-M(1,2)*M(2,1));
            Y=[iM;0,0]; % Y=P*[iM;zeros(5-m-2,2)];
            Z=[-iM*N;1]; % Z=P*[-iM*N;eye(5-m-2)];
            z=Z'*(Qh*Y*h-gh)/(Z'*Qh*Z);
            ph=-Y*h+Z*z;
            p=zeros(5,1);
            p(ninLU)=ph; % p=Eb'*ph;
            
        case 3 % Three active constraint. There is a single solution.
            %E=I(inLU,:); % Not used
            h=A*v-nur;
            % Eb=I(ninLU,:); % Not used
            Qh=Q(ninLU,ninLU); % Qh=Eb*Q*Eb';
            Ah=A(:,ninLU); % Ah=A*Eb'
            gh=Q(ninLU,:)*v; % gh=Eb*Q*v;
            % P=eye(5-m); % Not used
            M=Ah; %M=Ah(:,1:2);
            % N=Ah(:,3:end)=[]; % Not used. Inverse of matrix N (2x2 matrix)
            iM=[M(2,2),-M(1,2); -M(2,1), M(1,1)]/(M(1,1)*M(2,2)-M(1,2)*M(2,1));
            
            Y=iM; % Y=P*[iM;zeros(5-m-2,2)];
            % Z=[-iM*N]; % Z=P*[-iM*N;eye(5-m-2)]=[]; % Not used
            % z=(Z'*Qh*Z)^-1*Z'*(Qh*Y*h-gh)=[]; % Not used
            ph=-Y*h; % ph=-Y*h+Z*z;
            p=zeros(5,1);
            p(ninLU)=ph; % p=Eb'*ph;
            
        otherwise % More than tree active constraints. There is no solution.
            assert(0,'The reference vector lays in the overmodulation region');
    end
    if any(abs(p)>1e-5) % The voltage step is nonzero
        % Compute the step-length parameter delta:
        [delta,i]=ComputeStepLenght(p,v,l,u);
        % Update v before the next iteration:
        v=v+delta*p;
        if delta<1 % There is a new blocking constraint, which requires updating the working sets:
            if p(i)<0 % The blocking constraint is at the lower bound.
                % Add i to the lower working set:
                WS(i)=-1;
                % Update the active constraints count:
                m=m+1;
                % Clamp phase i to the lower voltage limit:
                v(i)=l(i);
            else % The blocking constraint is at the upper bound.
                % Add i to the upper working set:
                WS(i)=1;
                % Update the active constraints count:
                m=m+1;
                % Clamp phase i to the upper voltage limit:
                v(i)=u(i);
            end
        end
    else % The voltage step is null (p=0).
        % Solve equality constrained problem to find the Lagrange
        % multipliers mu;
        switch m
            case 0 % No active constraints.
                mu=[]; % There is no Lagrange multipliers to calculate.
            case 1 % One active constraint.
                mu=0; % One Lagrange multiplier, which is zero because q(v)=0;
            case {2,3}
                lambda=Y'*Qh*ph+Y'*gh;
                mu=Q(inLU,:)*v-A(:,inLU)'*lambda;  %mu=E*Q*(v+p)-E*A'*lambda; (Note that p=0)
                mu5=zeros(5,1);
                mu5(inLU)=mu;
                mu5(WS==1)=-mu5(WS==1);
                mu=mu5(inLU);
        end
        
        if all(mu>=0) % All Lagrange multipliers are nonzero.
            % Stop with solution v*=v for the output voltage.
            iter=k;
            break
        else % Drop one constraint.
            % Constraint with the most negative multiplier:
            [~,j]=min(mu5);
            % Remove j from the working sets:
            WS(j)=0;
            % Update the active constraints count:
            m=m-1;
        end
    end % if p>0
end % for k=1,2,...
end

function  [delta,i]=ComputeStepLenght(p,v,l,u)
% Compute the step-length parameter (delta) and the blocking constraint (i).
delta=1;
i=0;
for j = 1:5
    if p(j)>0 % p(j) going towards the upper limit.
        deltaj=(u(j)-v(j))/p(j);
        if deltaj<delta
            delta=deltaj;
            i=j;
        end
    elseif p(j)<0 % p(j) going towards the lower limit.
        deltaj=(l(j)-v(j))/p(j);
        if deltaj<delta
            delta=deltaj;
            i=j;
        end
    end
end
end

function m=MIN(nur,k)
% Minimum infinity-norm postfault operation strategy.
% Input parameters:
% nur = voltage reference in the alpha-beta frame (2x1 column vector).
%   k = number of active cells in each phase (5x1 column vector).
% Output:
%   m = PWM references (5x1 column vector).
    U=uMagnitude(k);
    M=-U.*(nur(1)*sin((0:4)'*2*pi/5)-nur(2)*cos((0:4)'*2*pi/5));
    [~,s]=max(abs(M)); % Sector identification
    phis=2*pi/5*(s-1);
    m=zeros(5,1);
    kk=max(k);
    for i=[1:s-1,s+1:5]
        phii=2*pi/5*(i-1);
        m(i)=sign(sin(phii-phis))*M(s);
        m(s)=m(s)+(kk-k(i))*m(i)*cos(phii-phis);
    end
    m(s)=(m(s)+5/2*(nur(1)*cos(phis)+nur(2)*sin(phis)))/k(s);
end

function U=uMagnitude(k)
% Calculates the magnitude of the u vectors [1].
% Input parameters:
%   k = number of active cells for each phase (5x1 column vector).
% Output:
%   U = Amplitude of u vectors  (5x1 column vector).
    U=zeros(5,1);
    for i=1:5
        phii=2*pi/5*(i-1);
        for j=1:5
            phij=2*pi/5*(j-1);
            U(i)=U(i)+k(j)*abs(sin(phii-phij));
        end
        U(i)=5/2/U(i);
    end
end
