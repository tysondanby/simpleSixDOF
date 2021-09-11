function multQuat(a,b)#a and b are quaternions being multiplied such that
    #result = a*b
    w = a.w*b.w - b.x*a.x - b.y*a.y - b.z*a.z
    x = b.w*a.x + b.x*a.w - b.y*a.z + b.z*a.y
    y = b.w*a.y + b.x*a.z + b.y*a.w - b.z*a.x
    z = b.w*a.z - b.x*a.y + b.y*a.x + b.z*a.w

    return quat(w,x,y,z)
end

#Rotates a quaternion (quat) that represents a rigid body.
#Rotation is based on the angular velocity (omeg) and the time step (dt).
function rotQuat(p,omeg,dt) #(struct,3-tuple,double)
    #result = a rotated quaternion representing the rigid body one dt later.

    #find axis of rotation and angle to rotate through.
    angSpd= norm(omeg)#angular speed about dir

    if angSpd == 0 #AVOID NaNs
        dir= omeg
    else
        dir= omeg ./ angSpd #Axis of rotation
    end

    th= angSpd*dt#theta (angular displacement) about the dir axis

    #create quaternions for rotation
    th = th/2.0 #quaternions double the angle input to them
    q= quat(cos(th),sin(th)*dir[1],sin(th)*dir[2],sin(th)*dir[3])#the rotation quaternion
    qinv= quat(cos(-th),sin(-th)*dir[1],sin(-th)*dir[2],sin(-th)*dir[3])#other rotation quaternion

    #make output
    out= multQuat(multQuat(q,p),qinv)
    mag= sqrt(out.w^2+out.x^2+out.y^2+out.z^2) #Make sure magnitude is 1
    out.w = out.w/mag
    out.x = out.x/mag
    out.y = out.y/mag
    out.z = out.z/mag
    return out
end
