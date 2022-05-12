#!/usr/bin/env julia

using RobotOS

@rosimport geometry_msgs.msg: Point, Pose2D, Twist
rostypegen()
using .geometry_msgs.msg

maxloops = 1000
rosloops = 0

function hpd_callback(msg::Twist) 
    hpd =[msg.linear.x, msg.linear.y, msg.linear.z,msg.angular.z]
    println(hpd)
end

function sensor1_callback(msg::Point) 
    sensor1 =[msg.x, msg.y, msg.z]
    println(sensor1)
end


function sendvalues(pub_obj,a)
    
    msg = Twist()
    msg.linear.x = a
    msg.linear.y = rand()
    msg.linear.z = rand()
    msg.angular.z = rand()
    publish(pub_obj, msg)
   
end

function main()
    
    a=rand()
    
    t=10 #Segundos 
    hz = 30 # Frecuencia de actualizacion
    loop_rate = Rate(hz)

    for k in 1:(t*hz) 
        tic = time()
        
        sendvalues(pub,k)
        
        rossleep(loop_rate)
        toc = time()
        dt = toc-tic
        #println(1/dt)
    end
    
end


if ! isinteractive()
    init_node("Controller")  
    pub = Publisher{Twist}("control", queue_size=10)
    sub = Subscriber{Twist}("Datos", hpd_callback,  queue_size=10)
    sub2 = Subscriber{Point}("sensor1", sensor1_callback,  queue_size=10)
    main()
end

