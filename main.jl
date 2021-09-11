using LinearAlgebra
include("quatOpperations.jl")
include("structs.jl")
include("forces.jl")

simLength=20.0 #time to simulate for
dt= 0.01#Time step
#I is the inertia tensor of the aircraft.
#x is forward through the nose
#y is to the right
#z is staight down
#     Γ xx xy xz  |
#I = | yx yy yz  |
#    L zx zy zz |
Io = [ 1.0  0  0;
       0  1.0  0;
       0  0  1.0] #A sphere

m = 1.0
g = [0,0,9.81]

Vo = 100.0 #initial airspeed

#direction quaternions
global dir = quat(0.0, 1.0, 0.0, 0.0) #nose points in this direction
#roll= 0.0
global bodyZ = quat(0.0, 0.0, 0.0, -1.0)#bottom of vehicle points in this direction

##STATE VARIABLES
#position vector
x= [0.0,0.0,0.0]
v= [Vo,0.0,0.0]
w= [0.0,0.0,0.0]#angular velocity

#other initialization
wdot=[0.0,0.0,0.0]#angular accel
vdot=[0.0,0.0,0.0]#accel
t=[0.0]
curState=fullState(dir,bodyZ,x,v,w,wdot,vdot,t)
pastStates=[curState]

for t in 0.0:dt:simLength
      #movement
      global x = x + (v .* dt)
      global dir= rotQuat(dir,w,dt) #adjust positions
      global bodyZ= rotQuat(bodyZ,w,dt)
      global v = v + (vdot .* dt) #adjust velocities
      global w = w + (wdot .* dt)


      #newtons laws (find next accelerations)
      global vdot = inertial(F(curState),dir,bodyZ)./m - g#The aerodynamic force is a function of many thnings.
      global wdot = Io \ (M(curState)-cross(w,Io*w))#Moments in body frame!!
      global curState=fullState(dir,bodyZ,x,v,w,wdot,vdot,t)
      push!(pastStates,curState)
end


using Plots
aEnd= trunc(Int,simLength/dt)
framerate= 20.0
aStep=trunc(Int,1/(framerate*dt)) #may have to modify this. Could cause problems

anim1 = @animate for i ∈ 1:aStep:aEnd #start,step,end
    animState = pastStates[i]

    xb=[animState.dir.x,animState.dir.y,animState.dir.z]
    zb=[animState.bodyZ.x,animState.bodyZ.y,animState.bodyZ.z]
    yb=cross(zb,xb)

    plot3d([0.0,xb[1]],[0.0,xb[2]],[0.0,xb[3]], xlims = (-1,1 ), ylims = (-1,1 ), zlims = (-1,1 ), c = :steelblue, labels="x")
    plot!([0.0,zb[1]],[0.0,zb[2]],[0.0,zb[3]], c = :red, labels="z")
    plot!([0.0,yb[1]],[0.0,yb[2]],[0.0,yb[3]], c = :green, labels="y")

end
#save the gif
gif(anim1, "Rotations.gif", fps = trunc(Int,framerate))

#initialize
lim=0.0#find a limit for the plot axes
posx=[]
posy=[]
posz=[]

for i in 1:length(pastStates)

      for j in 1:3

            if abs(pastStates[i].x[j]) > lim#find maximums/minimums
                  global lim=abs(pastStates[i].x[j])
                  #maxPos[j]=pastStates[i].x[j]
            #elseif pastStates[i].x[j] < minPos[j]#find minimums
            #      minPos[j]=pastStates[i].x[j]
            end
      end
      #build vectors for the 2nd plot (position)
      push!(posx,pastStates[i].x[1])
      push!(posy,pastStates[i].x[2])
      push!(posz,pastStates[i].x[3])
end

anim2 = @animate for i ∈ 1:aStep:aEnd #start,step,end
      animState = pastStates[i]

      xb=[animState.dir.x,animState.dir.y,animState.dir.z]
      zb=[animState.bodyZ.x,animState.bodyZ.y,animState.bodyZ.z]
      yb=cross(zb,xb)

      plot3d(posx[1:i],posy[1:i],posz[1:i], xlims = (-lim,lim ), ylims = (-lim,lim ), zlims = (-lim,lim ), c = :blue, labels="Flight Path")
      plot!([posx[i],posx[i]+lim*0.1*xb[1]],[posy[i],posy[i]+lim*0.1*xb[2]],[posz[i],posz[i]+lim*0.1*xb[3]],  c = :steelblue, labels="x")
      plot!([posx[i],posx[i]+lim*0.1*zb[1]],[posy[i],posy[i]+lim*0.1*zb[2]],[posz[i],posz[i]+lim*0.1*zb[3]], c = :red, labels="z")
      plot!([posx[i],posx[i]+lim*0.1*yb[1]],[posy[i],posy[i]+lim*0.1*yb[2]],[posz[i],posz[i]+lim*0.1*yb[3]], c = :green, labels="y")

end
#save the gif
gif(anim2, "Movement.gif", fps = trunc(Int,framerate))
