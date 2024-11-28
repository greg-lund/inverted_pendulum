using LinearAlgebra         # General matrix functionality, i.e. I
using ForwardDiff           # Calculating Jacobians
using CSV, DataFrames       # Saving and loading trajectories
using Plots, LaTeXStrings
using Convex, SCS

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

function trajMatrix(xtraj,utraj,p,dt)
  """
  x1 = A0x0 + B0u0 + c0
  x2 = A1A0x0 + A1B0u0 + B1u1 + A1c0 + c1
  x3 = A2A1A0x0 + A2A1B0u0 + A2B1u1 + B2u2 + A2A1c0 + A2c1 + c2
  x4 = A3A2A1A0x0 + A3A2A1B0u0 + A3A2B1u1 + A3B2u2 + B3u3 + A3A2A1c0 + A3A2c1 + A3c2 + c3
  """
  (d,N) = size(xtraj)
  At = collect(1.0 * I(d))
  Bt = zeros(d,N)
  ct = zeros(d)
  for i in 1:(N-1)
    (A,B,c) = autoLinDisc(xtraj[:,i],utraj[i],p,dt)
    At = A*At
    Bt = A*Bt
    Bt[:,i] = B
    ct = A*ct + c
  end
  return At,Bt,ct
end

function cartPosMatrix(N,dt)
  """
  x4 = A^4 x0 + A^3Bu0 + A^2Bu1 + ABu2 + Bu3
  """
  A(n) = [1. n*dt; 0 1]
  B(n) = [n * dt^2; dt]
  AB(n) = hcat(reduce(hcat,[B(n-i) for i in 1:n]),zeros(2,N-n))
  At = reduce(vcat,[A(i) for i in 1:N])
  Bt = reduce(vcat,[AB(i) for i in 1:N])
  Bx = Bt[1:2:end,:]; Bv = Bt[2:2:end,:]
  return Bx,Bv
end

function readTraj(filename)
  df = CSV.read(filename,DataFrame,header=0)
  t = df.Column1
  x = df.Column2
  theta = df.Column3
  return t,x,theta
end

function writeTraj(x,dt,filename)
  N = size(x,2)
  file = open(filename,"w")
  write(file,"const float control_dt = $dt;\n")
  write(file,"const float vel[] = {")
  for k in 1:N-1
    write(file,"$(x[3,k]), ")
  end
  write(file,"$(x[3,end])};\n")
  close(file)
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

function initTraj(N,dt,p)
  u = -(2*pi)^2*0.08*cos.(2*pi*(0:dt:(N-1)*dt)) .* exp.(-2:2/(N-1):0)
  return sim(zeros(4),u,dt,p),u
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

function lsOpt(xtraj,utraj,dt,params,niter)
  # Trust region schedule
  trust_region = 0.05
  pos_weight = 0.003

  xgoal = [0,pi,0,0]

  d,N = size(xtraj)
  Bx,_ = cartPosMatrix(N,dt)
  BxBx = Bx'Bx

  for k in 1:niter
    _,Bt,ct = trajMatrix(xtraj,utraj,params,dt)

    unew = (Bt'Bt + pos_weight*BxBx + trust_region * I) \ (trust_region*utraj - Bt'*(ct-xgoal))
    obj = norm(Bt*unew + ct - xgoal) + pos_weight*norm(Bx*unew) + trust_region*norm(unew-utraj)
    #println("Iteration $k. Cart position norm: $(norm(Bx*unew)). Control diff norm: $(norm(unew-utraj))")
    println("Iteration $k. End goal error: $(norm(Bt*unew+ct-xgoal)). Max cart position: $(maximum(abs.(Bx*unew))). Control diff: $(norm(unew-utraj))")
    utraj = copy(unew)
    xtraj = sim(zeros(4),unew,dt,params)
    if norm(Bt*unew+ct-xgoal) < 1e-2 && maximum(abs.(Bx*unew)) <= 0.125
      break
    end
  end
  return xtraj,utraj
end

function cvxOpt(xtraj,utraj,dt,params,niter)
  d,N = size(xtraj)
  trust_region = 1e-2
  xgoal = [0,pi,0,0]
  xmax = 0.12
  umax = 3.0

  Bx,_ = cartPosMatrix(N,dt)
  Bt = Variable(d,N)
  ct = Variable(d)
  uprev = Variable(N)
  u = Variable(N)

  obj = norm(Bt*u+ct-xgoal)
  constr = [-xmax <= Bx*u, Bx*u <= xmax] # Cart position
  constr += [-trust_region <= u-uprev, u-uprev <= trust_region] # Trust region
  constr += [-umax <= u, u <= umax] # Max control 

  prob = minimize(obj,constr)

  for k in 1:niter
    _,mBt,mct = trajMatrix(xtraj,utraj,params,dt)
    fix!(Bt,mBt)
    fix!(ct,mct)
    fix!(uprev,utraj)
    
    solve!(prob,SCS.Optimizer,silent_solver = true,warmstart=k>1)

    utraj = u.value
    xtraj = sim(zeros(4),utraj,dt,params)
    println("Iteration $k. End goal error: $(norm(mBt*utraj+ct.value-xgoal)). Max cart position: $(maximum(abs.(Bx*utraj))). Control diff: $(norm(uprev.value-utraj))")
    if norm(mBt*utraj+ct.value-xgoal) < 1e-2
      break
    end
  end
  return xtraj,utraj
end

params = [54.22,0.075,5.6]

N = 350
dt = 1e-2
niter = 10000
xtraj,utraj = initTraj(N,dt,params)
xopt,uopt = lsOpt(xtraj,utraj,dt,params,niter)
#xopt,uopt = cvxOpt(xtraj,utraj,dt,params,niter)

writeTraj(xopt,dt,"data/control_traj.h")

plotTraj(xopt,dt)
