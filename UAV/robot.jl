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


function vc_callback(pose::Twist) 
    
    global vc =[pose.linear.x, pose.linear.y, pose.linear.z, pose.angular.z]
    
    #println(hpd)
end

function run_callback(msg::Point) 
    global run =[msg.x, msg.y, msg.z]

end


function set_pos(pub_pos,h)  
    msg = Pose()
    msg.position.x = h[1]
    msg.position.y = h[2]
    msg.position.z = h[3]
    msg.orientation.z = h[4]
    publish(pub_pos, msg) 
end


function main()
 
    println("Run the controller, please!")
    runnow =  0
    while runnow == 0
        set_pos(pub_pos, [rand() rand() 1 rand()])
        #running = rungo
        try
            runnow = run
        catch
            runnow = 0
        end
        rossleep(Rate(10))
    end
    println("Yes, you are ready!!!")
    # Configuracion de samples
    t=40 #Segundos 
    hz = 30 # Frecuencia de actualizacion
    samples = t*hz # datos de muestreo totales
    loop_rate = Rate(hz) # Tiempo de muestre espera

    # Inicializacion de matrices
    h = zeros(4, samples+1)
    hp = zeros(4, samples+1)
    
    # Espera arranque
    #rossleep(0.1)
    for k in 1:(t*hz) 
        
        tic = time()

        # Cinematica del robot
        psi = h[4,k]
        rot = [cos(psi) -sin(psi) 0 0; sin(psi) cos(psi) 0 0; 0 0 1 0; 0 0 0 1]
        hp[:,k] = rot*vc
        

        # Integracion numerica
        h[:,k+1] = hp[:,k]*(1/hz) + h[:,k] 
        println(h[:,k+1])
        # Enviar valores
        set_pos(pub_pos, h[:,k+1])
        

        rossleep(loop_rate) 
        toc = time()
        dt = toc-tic
    end
#    runcontroller(pub_run,0)
    #am = var
    
end

if ! isinteractive()
    init_node("Robot")  
    
    
    
    
    sub_vc = Subscriber{Twist}("control", vc_callback,  queue_size=10)
    sub_run = Subscriber{Point}("Run", run_callback,  queue_size=10)
    pub_pos = Publisher{Pose}("Position", queue_size=10)
    pub_run = Publisher{Point}("Run", queue_size=10) 
    main()
end