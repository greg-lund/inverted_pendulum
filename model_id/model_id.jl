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
  nstep = 100
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

function optControlParam(filename,natParams)
  t,x,theta = readTraj(filename)
  dt = t[2]-t[1]
  
  xdot,xddot = getFiniteDiffs(x,t)
  thetadot,thetaddot = getFiniteDiffs(theta,t)
  x0 = [x[1],theta[1],xdot[1],thetadot[1]]
  
  # Optimize for the parameters
  J(c) = norm( sim(x0,xddot,dt,[natParams[1],natParams[2],c[1]])[2,:] - theta )
  result = optimize(J, [5.0])
  optC = result.minimizer[1]

  return [natParams[1],natParams[2],optC]
end

function optParams(filename)
  t,x,theta = readTraj(filename)
  dt = t[2]-t[1]
  
  xdot,xddot = getFiniteDiffs(x,t)
  thetadot,thetaddot = getFiniteDiffs(theta,t)
  x0 = [x[1],theta[1],xdot[1],thetadot[1]]
  
  # Optimize for the parameters
  J(p) = norm( sim(x0,xddot,dt,p)[2,:] - theta )
  result = optimize(J, params)
  opt_params = result.minimizer

  return opt_params
end

function plotFit(filename,params)
  t,x,theta = readTraj(filename)
  dt = t[2]-t[1]
  xdot,xddot = getFiniteDiffs(x,t)
  thetadot,thetaddot = getFiniteDiffs(theta,t)
  x0 = [x[1],theta[1],xdot[1],thetadot[1]]
  X = sim(x0,xddot,dt,params)
  
  p1 = plot(t,x,ylabel=L"x")
  plot!(t,X[1,:],label="sim")
  p2 = plot(t,theta,ylabel=L"\theta")
  plot!(t,X[2,:],label="sim")
  p3 = plot(t,xdot,ylabel=L"\dot{x}")
  plot!(t,X[3,:],label="sim")
  p4 = plot(t,thetadot,ylabel=L"\dot{\theta}")
  plot!(t,X[4,:],label="sim")
  plot(p1,p2,p3,p4,layout=(2,2))
end

natParams = [54.22, 0.075, 0]
params = [54.22,0.075,5.6]

#filename = "data/simplePendulum.txt"
#simple_params = optParams(filename)
#plotFit(filename,params)

filename = "data/swingup5.txt"
#p = optControlParam(filename,natParams)
plotFit(filename,params)

#for i in 0:9
#  if i == 0
#    filename = "data/swingup.txt"
#  else
#    filename = "data/swingup$i.txt"
#  end
#  p = optControlParam(filename,natParams)
#  println("$filename: $p")
#end
