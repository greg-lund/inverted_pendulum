using MatrixEquations       # Ricatti
using LinearAlgebra         # General matrix functionality, i.e. I
using ForwardDiff           # Calculating Jacobians
using Plots
using Optim

function dynamics(x,u,p)
  # state x = [x,theta,x_dot,theta_dot]
  # u: control input scalar
  # p = [a,b,c]

  a,b,c = p
  return [x[3],
          x[4],
          u,
          -a*sin(x[2]) - b*x[4] - c*cos(x[2])*u]
end

function autoLinDisc(x0,u0,p,dt)
  # Get linearized and discretized matrices from dynamics using ForwardDiff

  A = ForwardDiff.jacobian(x -> dynamics(x,u0,p),x0)
  B = ForwardDiff.derivative(u -> dynamics(x0,u,p),u0)
  c = dynamics(x0,u0,p) - A*x0 - B*u0

  # Discretize using Forward Euler
  return (I+A*dt,B*dt,c*dt)

end

function simLQR(K,x0,params,dt,duration)
  N = Int(floor(duration/dt))
  X = zeros(4,N)
  U = zeros(N)
  X[:,1] = x0
  for i in 1:N-1
    x = X[:,i]
    u = -dot(K,x - [0,pi,0,0])
    U[i] = u 
    nstep = 100
    for j in 1:nstep
      ddt = dynamics(x,u,params)
      x = x + ddt * (dt/nstep)
    end
    X[:,i+1] = x
  end
  return X,U
end

function plotLQR(K,x0,params,dt,duration)
  X,U = simLQR(K,x0,params,dt,duration)
  
  p1 = plot(X[1,:],label="",ylabel="x")
  p2 = plot(X[2,:],label="",ylabel="theta")
  p3 = plot(X[3,:],label="",ylabel="dx")
  p4 = plot(X[4,:],label="",ylabel="dtheta")
  
  plot(p1,p2,p3,p4,layout=(2,2))
end

function J(v,params,dt,A,B)
  Q = diagm(v[1:4]); R = v[5]
  _,_,K,_,_ = ared(A,B,R,Q)

  X,U = simLQR(K,[0.0,pi-0.2,0,0],params,dt,2)
  x = X[1,:]
  return norm(x,Inf)
end

function optK(params,dt)
  A,B,_ = autoLinDisc([0,pi,0,0],0,params,dt)
  f(v) = J(v,params,dt,A,B)

  lower = 1e-4 * ones(5); upper = 50*ones(5)
  options = Optim.Options(time_limit=60.0,show_trace=true,show_every=10,allow_f_increases=true,f_tol=1e-6)

  res = optimize(f,lower,upper,ones(5),Fminbox(LBFGS()), options)

  Q = diagm(res.minimizer[1:4]); R = res.minimizer[5]
  _,_,K,_,_ = ared(A,B,R,Q)
  return K
end

#params = [30.78382418855733, 0.4740118875157444, 1.2619764896261447]
params = [62.5,0.98,4.85]

x0 = [0,pi,0,0]; u0 = 0
control_dt = 1e-2
duration = 2

A,B,_ = autoLinDisc(x0,u0,params,control_dt)
At = A[2:2:end,2:2:end]; Bt = B[2:2:end]
#_,_,K,_,_ = ared(At,Bt,1,1); K = [0,K[1],0,K[2]]
Q = diagm([50,1,1,1]); R = 0.1
_,_,K,_,_ = ared(A,B,R,Q)
plotLQR(K,[0,pi-0.2,0,0],params,control_dt,duration)
