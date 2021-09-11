function F(state)
    return [0.0,0.0,0.0]#some thrust
end

function M(state)
    M=[0.0,-0.10,0.20]
    return [M[1],-M[2],-M[3]]#Automatically corrects for aircraft convention
end

function inertial(F,xb,zb) #Takes a force from body frame and puts it in the inertial frame. Works with any vector.
    front= [xb.x,xb.y,xb.z]
    down= [zb.x,zb.y,zb.z]
    out= F[1]*front - F[2]*cross(front,down) + F[3]*down
    return out
end
