#!/usr/bin/env julia

using RobotOS

@rosimport geometry_msgs.msg: Point, Pose2D, Twist
rostypegen()
using .geometry_msgs.msg

maxloops = 1000
rosloops = 0


function hpd_callback(msg::Twist) 
    global hdp
    hpd =[msg.linear.x, msg.linear.y, msg.linear.z,msg.angular.z]
    #println(hpd)
end

function sensor1_callback(msg::Point) 
    sensor1 =[msg.x, msg.y, msg.z]
    #println(sensor1)
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
    hz = 10 # Frecuencia de actualizacion
    samples = t*hz
    loop_rate = Rate(hz)

    psi = Array{Float64}(undef, samples)
    vector = []
    h = zeros(4, samples)
    J = zeros(4,4)
    hpd = zeros(4,1)
    for k in 1:(t*hz) 
        tic = time()
        #push!(vector, k)
        #psi[k]=rand()
        h[:,k] = [0,0,0,rand()]
        psi = h[4,k]
        hxp = hpd[1]
        # Ley de Control
        J11 = cos(psi);
        J12 = -sin(psi);
        J13 = 0;
        J14 = 0;

        J21 = sin(psi);
        J22 = cos(psi);
        J23 = 0;
        J24 = 0;

        J31 = 0;
        J32 = 0;
        J33 = 1;
        J34 = 0;

        J41 = 0;
        J42 = 0;
        J43 = 0;
        J44 = 1;

        J = [J11 J12 J13 J14;J21 J22 J23 J24;J31 J32 J33 J34;J41 J42 J43 J44]

        
        sendvalues(pub,k)
        
        rossleep(loop_rate); toc = time()
        dt = toc-tic
        #println(k , ": " ,psi[k] ," ", vector[k])
        println(hxp)
    end
    #println(J)
end


if ! isinteractive()
    init_node("Controller")  
    pub = Publisher{Twist}("control", queue_size=10)
    sub = Subscriber{Twist}("Datos", hpd_callback,  queue_size=10)
    sub2 = Subscriber{Point}("sensor1", sensor1_callback,  queue_size=10)
    main()
end

