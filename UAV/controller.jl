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
end

function sensor1_callback(msg::Point) 
    sensor1 =[msg.x, msg.y, msg.z]
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
    # Espera 1 seg para comenzar la fiesta!!
    rossleep(1)

    # Trayectoria deseada
    #hxd = 5 * sin(mul*0.04*t)+0.1;         hxdp = 5*mul*0.04*cos(mul*0.04*t);     hxdpp = -5*mul*mul*0.04*0.04*sin(mul*0.04*t);
    #hyd = 5 * sin(mul*0.08*t)+0.1;         hydp = 5*mul*0.08*cos(mul*0.08*t);     hydpp = -5*mul*mul*0.08*0.08*sin(mul*0.08*t);               
    #hzd = 1 * sin (0.08 * t) +2 ;          hzdp = 0.08*cos(0.08*t);

    # Posicion deseada
    global hxd = 12.3456789
    global hyd = 21.2112
    global hzd = 16.3654
    global psid = 0.123456

    t=30 #Segundos 
    hz = 30 # Frecuencia de actualizacion
    samples = t*hz # datos de muestreo totales
    loop_rate = Rate(hz) # Tiempo de muestre espera
    
    # Inicializacion de matrices
    h = zeros(4, samples)
    hd = zeros(4, samples)
    he = zeros(4, samples)
    J = zeros(4,4)
    global K1 = Matrix{Float64}(I, 4, 4)
    global K2 = Matrix{Float64}(I, 4, 4)

    # Activa controlador para el robot
    runcontroller(pub_run,1)
    
    println("OK, controller is runnig!!!")
    for k in 1:(t*hz) 
        
        tic = time()

        hd[:,k] = [hxd,hyd,hzd,psid]
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

        # Envia velocidad de manipulavilidad al robot
        sendvalues(pub,vc)
        
        toc = time()
        dt = toc-tic
        rossleep(loop_rate) 
     
        #println(k)
    end
    
    runcontroller(pub_run,0)
    
    x = 1:1:samples
    hxe= he[1,1:samples]
    hye= he[2,1:samples]
    hze= he[3,1:samples]
    hpsie= he[4,1:samples]

    plot!(x,hxe, title = "Erro de control", label = ["hxe"], lw = 1)
    plot!(x,hye,label = ["hye"], lw = 1)
    plot!(x,hze,label = ["hze"], lw = 1)
    plot!(x,hpsie,label = ["hpsie"], lw = 1)
    savefig("error.pdf")

end

if ! isinteractive()
    init_node("Controller")  
    pub = Publisher{Twist}("control", queue_size=10)
    pub_run = Publisher{Point}("Run", queue_size=10)
    sub = Subscriber{Pose}("Position", pos_callback,  queue_size=10)
    sub2 = Subscriber{Point}("sensor1", sensor1_callback,  queue_size=10)
    main()
end

