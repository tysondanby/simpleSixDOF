mutable struct quat #used for quaternions

    w
    x
    y
    z
end

mutable struct fullState #used for keeping track of past states
                 #Purpose could be for debug or graphical representations
    dir
    bodyZ
    x#position vector
    v#velocity vector
    w#angular velocity
    wdot
    vdot
    t
end
