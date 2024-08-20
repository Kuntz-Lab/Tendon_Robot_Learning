%
% Comments from the author:
%
%   The comments should give you an idea of how to use the function, but let us
%   know if the usage is unclear.  We parameterized tendon path in terms of
%   distance and angle from the backbone, both defined by arbitrary polynomials
%   whose coefficients are stored in matrices C and D.  You input the routing
%   coefficient matrices C and D along with a vector of tendon tensions tau.
%   An example configuration is implemented if no arguments are entered.   The
%   program should efficiently solve the problem and integrate the full shape
%   in about 0.1 seconds.  You can comment out the plots, of course.
%
% The paper first describing the kinematic model:
%
%   https://ieeexplore.ieee.org/abstract/document/5957337
%
% A recent paper that looks at stiffness (maybe)
%
%   https://ieeexplore.ieee.org/document/8606257
%

function Tendon_Actuated_Robot_Fast(C,D,tau)

  function [r,r_dot,r_ddot]=get_r_info(s)

    r=zeros(3,N_t);
    r_dot=zeros(3,N_t);
    r_ddot=zeros(3,N_t);
    S_a=zeros(N_a,1);
    S_ad=zeros(N_a,1);
    S_add=zeros(N_a,1);
    S_m=zeros(N_m,1);
    S_md=zeros(N_m,1);
    S_mdd=zeros(N_m,1);
    for j=1:N_a
      S_a(j,1)=s^(j-1);
      S_ad(j,1)=(j-1)*s^(max([0,(j-2)]));
      S_add(j,1)=(j-1)*(j-2)*s^(max([0,(j-3)]));
    end
    for j=1:N_m
      S_m(j,1)=s^(j-1);
      S_md(j,1)=(j-1)*s^(max([0,(j-2)]));
      S_mdd(j,1)=(j-1)*(j-2)*s^(max([0,(j-3)]));
    end
    for j=1:N_t
    r(:,j)=D(j,:)*S_m*[sin(C(j,:)*S_a);cos(C(j,:)*S_a);0];
    r_dot(:,j)=D(j,:)*S_md*[sin(C(j,:)*S_a);cos(C(j,:)*S_a);0]+...
              +D(j,:)*S_m*[cos(C(j,:)*S_a)*C(j,:)*S_ad;-sin(C(j,:)*S_a)*C(j,:)*S_ad;0];
    r_ddot(:,j)=D(j,:)*S_mdd*[sin(C(j,:)*S_a);cos(C(j,:)*S_a);0]+...
              +2*D(j,:)*S_md*[cos(C(j,:)*S_a)*C(j,:)*S_ad;-sin(C(j,:)*S_a)*C(j,:)*S_ad;0]+...
              +D(j,:)*S_m*[cos(C(j,:)*S_a)*C(j,:)*S_add;-sin(C(j,:)*S_a)*C(j,:)*S_add;0]+...
              +D(j,:)*S_m*[-sin(C(j,:)*S_a)*(C(j,:)*S_ad)^2;-cos(C(j,:)*S_a)*(C(j,:)*S_ad)^2;0];
    end
  end

  function u_hat=hat(u)
    u_hat=[0,-u(3),u(2);u(3),0,-u(1);-u(2),u(1),0];
  end

  function y_dot = deriv(s, y)

    %Turn ODE input iN_to N_amed variables
    %p = [y(1);y(2);y(3)];
    R = reshape(y(4:12),3,3);
    v = [y(13);y(14);y(15)];
    u = [y(16);y(17);y(18)];
    vhat=hat(v);
    uhat=hat(u);
    [r,r_dot,r_ddot]=get_r_info(s);

    A=zeros(3,3);B=zeros(3,3);G=zeros(3,3);H=zeros(3,3);
    a=zeros(3,1);b=zeros(3,1);
    si_dot=zeros(N_t,1);
    for j=1:N_t

      rhat=hat(r(:,j));
      pi_dot_b=uhat*r(:,j)+r_dot(:,j)+v;
      si_dot(j)=norm(pi_dot_b);

      Ai=-tau(j)*hat(pi_dot_b)^2/norm(pi_dot_b)^3;
      Bi=rhat*Ai;
      Gi=-Ai*rhat;
      Hi=-Bi*rhat;

      ai=Ai*(uhat*pi_dot_b+uhat*r_dot(:,j)+r_ddot(:,j));
      bi=rhat*ai;

      A=A+Ai;
      B=B+Bi;
      G=G+Gi;
      H=H+Hi;
      a=a+ai;
      b=b+bi;
    end

    c=-uhat*K_bt*u-vhat*K_se*(v-[0;0;1])-b;
    d=-uhat*K_se*(v-[0;0;1])-a;

    %Compute state variable derivatives
    p_dot = R*v;
    R_dot = R*uhat;
    xi_dot= [K_se+A, G; B, K_bt+H]\[d;c];
    v_dot=xi_dot(1:3);
    u_dot=xi_dot(4:6);
    y_dot = [p_dot;reshape(R_dot,9,1);v_dot;u_dot;norm(v);si_dot];
  end

  close all

  %routing parameters - each row represents one tendon path
  %i^th tendon path is characterized by angle phi_i(s) and magnitude r_mag_i(s)
  %phi_i(s)=Ci1+Ci2*s+Ci3*s^2+Ci4*s^3+...
  %r_mag_i(s)=Di1+Di2*s+Di3*s^2+Di4*s^3+...
  %number of tendons and coefficients is inferred from the size of C and D
  %Example: if we want 2 straight tendons combined with a left and right helix:
  if nargin<1
    C=[0,0;
       pi,0;
       pi/2,20;
       -pi/2,-20];
    D=[.01;
       .01;
       .01;
       .01];
    %Tendon Tension
    tau=[2;0;20;2];
  end

  N_t=size(C,1); %number of tendons
  N_a=size(C,2); %number of angle coeffs per tendon
  N_m=size(D,2); %number of magnitude coeffs per tendon

  %Rod parameters
  L=.2;
  ro=.01;
  ri=0.00;
  I=1/4*pi*(ro^4-ri^4);
  Ar=pi*(ro^2-ri^2);
  J=2*I;
  E=2.1*10^6;
  nu=.3;
  Gmod=E/(2*(1+nu));

  %Inverses of the stiffness matrices
  K_bt=[(E*I),0,0;0,(E*I),0;0,0,(J*Gmod)];
  K_bt_inv=[1/(E*I),0,0;0,1/(E*I),0;0,0,1/(J*Gmod)];
  K_se=[(Gmod*Ar),0,0;0,(Gmod*Ar),0;0,0,(E*Ar)];
  K_se_inv=[1/(Gmod*Ar),0,0;0,1/(Gmod*Ar),0;0,0,1/(E*Ar)];

  tic
  %Solve the Problem
  [r,r_dot,~]=get_r_info(0);
  u0=[0;0;0];
  v0=[0;0;1];
  for i=1:30
    u0hat=hat(u0);
    n0=0;
    m0=0;
    for k=1:N_t
      pi_dot_0=(u0hat*r(:,k)+r_dot(:,k)+v0);
      n0=n0-tau(k)*pi_dot_0/norm(pi_dot_0);
      m0=m0-tau(k)*hat(r(:,k))*pi_dot_0/norm(pi_dot_0);
    end
    v0new=K_se_inv*n0+[0;0;1];
    u0new=K_bt_inv*m0;
    %pi_dot_0=(hat(K_bt_inv*(-tau*hat(r)*pi_dot_0/norm(pi_dot_0)))*r+r_dot+K_se_inv*(-tau*pi_dot_0/norm(pi_dot_0))+[0;0;1]);
    if norm(v0new-v0)/norm(v0)<1e-5 && norm(u0new-u0)/norm(u0)<1e-6
      break
    else
      v0=v0new;
      u0=u0new;
    end
  end
  iterations=i
  correct_guess=[v0new;u0new]

  p_init=[0;0;0];
  R_init=reshape(eye(3),9,1);
  v_init=correct_guess(1:3);
  u_init=correct_guess(4:6);
  y_init=[p_init;R_init;v_init;u_init;0;zeros(N_t,1)];
  [s,y]=ode45(@deriv,0:.005:L,y_init);

  computation_time=toc

  %resulting tendon lengths
  L=y(end,end-N_t+1:end)

  %plot the backbone
  %plot3(y(:,1),y(:,2),y(:,3),'b','LineWidth',3)
  %hold on

  angles=0:.1:2*pi+.1;
  disk_r=D(1);
  for i=1:length(y)
    p = [y(i,1);y(i,2);y(i,3)];
    R = reshape(y(i,4:12),3,3);
    points=repmat(p, [1, length(angles)])+R*disk_r*[sin(angles);cos(angles);zeros(1,length(angles))];
    points=repmat(p, [1, length(angles)])+R*disk_r*[sin(angles);cos(angles);zeros(1,length(angles))];
    plot3(points(1,:),points(2,:),points(3,:),'r','LineWidth',1)
    hold on
  end


  %plot all the tendons
  for k=1:N_t

    pt=zeros(length(s),3);
    for i=1:length(s)
      p = [y(i,1);y(i,2);y(i,3)];
      R = reshape(y(i,4:12),3,3);
      [r,r_dot,r_ddot]=get_r_info(s(i));
      pt(i,1:3)= p+R*r(:,k);
    end

    plot3(pt(:,1),pt(:,2),pt(:,3),'k','LineWidth',3)
    hold on

  end

  axis([-.2 .2 -.2 .2 0 .250])
  daspect([1 1 1])
  grid on
  xlabel('x')
  ylabel('y')
  zlabel('z')
  hold on
  %axis off
  view([0 1 0])
  %legend('backbone','tendons')

  %Fancy plot of the backbone material
  % g_array=[y(:,4:6),zeros(length(y),1),...
  %           y(:,7:9),zeros(length(y),1),...
  %           y(:,10:12),zeros(length(y),1),...
  %           y(:,1:3),ones(length(y),1)];
  %
  % plotsection(g_array,ro,[1 1 1]*.8)
  % camlight headlight
  % lighting phong

end
