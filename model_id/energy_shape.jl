using LinearAlgebra         # General matrix functionality, i.e. I
using ForwardDiff           # Calculating Jacobians
using CSV, DataFrames       # Saving and loading trajectories
using Plots, LaTeXStrings

function dynamics(x,u,p)
  # state x = [x,theta,x_dot,theta_dot]
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

function sim(x0,utraj,dt,p)
  N = length(utraj)
  X = zeros(4,N)
  X[:,1] = x0
  nstep = 1
  for i in 1:N-1
    x = X[:,i]
    u = utraj[i]
    for j in 1:nstep
      x = x + dynamics(x,u,p) * (dt/nstep)
    end
    X[:,i+1] = x
  end
  return X
end

function plotTraj(X,dt)
  N = size(X,2)
  t = 0:dt:(N-1)*dt
  x = X[1,:]; theta = X[2,:]
  xdot = X[3,:]; thetadot = X[4,:]
  p1 = plot(t,x,ylabel=L"x",label="")
  p2 = plot(t,theta,ylabel=L"\theta",label="")
  p3 = plot(t,xdot,ylabel=L"\dot{x}",label="")
  p4 = plot(t,thetadot,ylabel=L"\dot{\theta}",label="")
  plot(p1,p2,p3,p4,layout=(2,2))
end

#params = [54.191503701532845, 0.3095185897453433, 5.0869402268016986]
params = [54.191503701532845, 0., 5.0869402268016986]

N = 300
dt = 1e-2

xtraj = zeros(4,N)
xtraj[2,1] = 0.01 # X-offset to start control

k = [0.01,0,0]
for i in 1:N-1
  x = xtraj[:,i]
  u = k[1] * x[4] * cos(x[2]) * (0.5*x[4]^2-params[1]*(cos(x[2])+1))
  x = x + dynamics(x,u,params) * dt
  xtraj[:,i+1] = x
end

plotTraj(xtraj,dt)
