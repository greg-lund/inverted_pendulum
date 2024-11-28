using LinearAlgebra         # General matrix functionality, i.e. I
using ForwardDiff           # Calculating Jacobians
using CSV, DataFrames
using Plots, LaTeXStrings
using Optim

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

function readTraj(filename)
  df = CSV.read(filename,DataFrame,header=0)
  t = df.Column1
  x = df.Column2
  theta = df.Column3

  return t,x,theta
end

function getFiniteDiffs(x,t)
  N = length(x)
  v = vcat([(x[i+1]-x[i])/(t[i+1]-t[i]) for i in 1:N-1],0)
  a = vcat([(v[i+1]-v[i])/(t[i+1]-t[i]) for i in 1:N-1],0)
  return v,a
end

function sim(x0,utraj,dt,params)
  N = length(utraj)
  X = zeros(4,N)
  X[:,1] = x0
  nstep = 1000
  for i in 1:N-1
    x = X[:,i]
    u = utraj[i]
    for j in 1:nstep
      x = x + dynamics(x,u,params) * (dt/nstep)
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

function simControl(x0,params,dt,duration)
  a,b,c = params
  umax = 1.0
  k = 1
  kx = 10
  kv = 5
  N = Int(floor(duration/dt))
  X = zeros(4,N)
  U = zeros(N)
  X[:,1] = x0
  for i in 1:N-1
    x = X[:,i]
    if abs(x[2]) > 3.
      X = X[:,1:i]
      U = U[1:i]
      break
    end
    ue = -b*x[4]/(c*cos(x[2])) + cos(x[2])*x[4]*(1/(2*a)*x[4]^2 - cos(x[2]) - 1)
    u = k * ue - kx * x[1] - kv*x[3]
    u = max(min(u,umax),-umax)
    U[i] = u 
    nstep = 100
    for j in 1:nstep
      ddt = dynamics(x,u,params)
      x = x + ddt * (dt/nstep)
    end
    X[:,i+1] = x
  end
  return collect(0:dt:(length(U)-1)*dt),X,U
end


# LP formulation:
#
# max Bt*u
# s.t. -xmax < Bx*u < xmax
#      -umax < u < umax
#      -treg < u - uprev < treg
#      Bt*u + ct < xgoal
#
# C = [Bx;-Bx;I;-I;I;-I;Bt] shape (7N,N)
# d = [xmax1;xmax1;umax1;umax1;(treg+uprev)1;(treg+uprev)1;(xgoal-ct)1] shape (7N,)
#
# max 1'Bt*u
# s.t. C*u < d
#
# thetaddot = -a * sin(theta) - b * thetadot - c * cos(theta) * u

params = [54.22,0.075,5.6]

x0 = [0.0,0.02,0,0.2]
dt = 1e-2
duration = 15.0
t,X,U = simControl(x0,params,dt,duration)
println("Max theta: $(maximum(abs.(X[2,:])))")

plotTraj(X,dt)
