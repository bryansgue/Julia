#!/usr/bin/env julia

#pkg> status ## Es para check los paquetes instalados 
using RobotOS
using LinearAlgebra
using Plots

@rosimport geometry_msgs.msg: Point, Pose2D, Twist, Pose
#@rosimport std_msgs.msg: Int32
rostypegen()
using .geometry_msgs.msg
#using .std_msgs.msg

maxloops = 1000
rosloops = 0


function pos_callback(pose::Pose) 
    
    global pos =[pose.position.x, pose.position.y, pose.position.z, pose.orientation.z]
    #println(hpd)
end

function sensor1_callback(msg::Point) 
    sensor1 =[msg.x, msg.y, msg.z]
    #println(sensor1)
end


function sendvalues(pub_obj,vc)  
    msg = Twist()
    msg.linear.x = vc[1]
    msg.linear.y = vc[2]
    msg.linear.z = vc[3]
    msg.angular.z = vc[4]
    publish(pub_obj, msg) 
end

function runcontroller(pub_obj,runvar)  
    msg = Point()
    msg.x = runvar
    publish(pub_obj, msg) 
end

function main()
    rossleep(1)
    #xd = 5 * sin(mul*0.04*t)+0.1;         xd_p = 5*mul*0.04*cos(mul*0.04*t);     xd_pp = -5*mul*mul*0.04*0.04*sin(mul*0.04*t);
    #yd = 5 * sin(mul*0.08*t)+0.1;         yd_p = 5*mul*0.08*cos(mul*0.08*t);     yd_pp = -5*mul*mul*0.08*0.08*sin(mul*0.08*t);               
    #zd = 1 * sin (0.08 * t) +2 ;          zd_p = 0.08*cos(0.08*t);

    # Posicion deseada
    global hxd = 12.3456789
    global hyd = 21.2112
    global hzd = 16.3654
    global psid = 0.123456

    t=40 #Segundos 
    hz = 30 # Frecuencia de actualizacion
    samples = t*hz # datos de muestreo totales
    loop_rate = Rate(hz) # Tiempo de muestre espera

    psi = Array{Float64}(undef, samples)
    vector = []
    h = zeros(4, samples)
    hd = zeros(4, samples)
    he = zeros(4, samples)
    J = zeros(4,4)

    global K1 = Matrix{Float64}(I, 4, 4)
    global K2 = Matrix{Float64}(I, 4, 4)

    #values = var
    
    #am = var
    
    runcontroller(pub_run,1)
    
    
    for k in 1:(t*hz) 
        
        tic = time()

        hd[:,k] = [hxd,hyd,hzd,psid]
        #h[:,k] = [pos[1],pos[2],pos[3],pos[4]]
        h[:,k] = pos
        he[:,k] = hd[:,k]-h[:,k]

        psi = h[4,k]
        
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

        #VMref = pinv(J)*(hdp+K1*tanh(K2*he'));
        vc = pinv(J) *(K1*tanh.(K2*he[:,k]))
        #VMref = pinv(J)*(K1*tanh(K2*he'));
        
        sendvalues(pub,vc)
        
        
        toc = time()
        dt = toc-tic
        rossleep(loop_rate) 
     
        println(he[:,k])
    end
    runcontroller(pub_run,0)
    #am = var
    
end

if ! isinteractive()
    init_node("Controller")  
    pub = Publisher{Twist}("control", queue_size=10)
    pub_run = Publisher{Point}("Run", queue_size=10)
    sub = Subscriber{Pose}("Position", pos_callback,  queue_size=10)
    sub2 = Subscriber{Point}("sensor1", sensor1_callback,  queue_size=10)
    main()
end

