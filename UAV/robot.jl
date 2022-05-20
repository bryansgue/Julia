#!/usr/bin/env julia

#pkg> status ## Es para check los paquetes instalados 
using RobotOS
using LinearAlgebra
using Plots

@rosimport geometry_msgs.msg: Point, Pose2D, Twist, Pose
@rosimport nav_msgs.msg: Odometry
@rosimport std_msgs.msg: Int32MultiArray

rostypegen()
using .geometry_msgs.msg
using .nav_msgs.msg
using .std_msgs.msg

function vc_callback(pose::Twist) 
    global v_ref =[pose.linear.x, pose.linear.y, pose.linear.z, pose.angular.z]
end

function config_callback(msg::Int32MultiArray) 
    global config = msg.data
end

function set_odo(pub_odo,h,hp)  
    msg = Odometry()
    msg.pose.pose.position.x = h[1]
    msg.pose.pose.position.y = h[2]
    msg.pose.pose.position.z = h[3]
    msg.pose.pose.orientation.z = h[4]
    msg.twist.twist.linear.x = hp[1]
    msg.twist.twist.linear.y = hp[2]
    msg.twist.twist.linear.z = hp[3]
    msg.twist.twist.angular.z = hp[4]

    publish(pub_odo, msg) 
end

function main()

    # Espera arranque
    println("Run the controller, please!")
    runnow =  0
    while runnow == 0
        set_odo(pub_pos, [0 0 1 0], [0 0 0 0]) # Posicion y velocidad inicial
        try
            runnow = config[1]
            global t = config[2] #Segundos de controlador            
            #rossleep(0.2)
        catch
            runnow = 0    
            rossleep(Rate(10))   
        end 
        
    end
    

    hz = 10 # Frecuencia de actualizacion
    samples = t*hz # datos de muestreo totales
    loop_rate = Rate(hz) # Tiempo de muestre espera

    # Inicializacion de matrices
    h = zeros(4, samples+1)
    hp = zeros(4, samples)
    v = zeros(4, samples+1)
    vref = zeros(4, samples+1)
        
    # Estados iniciales del robot
    h[:,1] = [0 0 1 0]
    hp[:,1] = [0 0 0 0]

    println("OK, controller is runnig!!!")

    #Begins the controller
    for k in 1:(t*hz) 
        
        tic = time()
        a=0
        b=0
        ts = (1/hz)
        
        x = [1.4189;0.0213;1.2633;-0.2427;1.9647;0.1000;0.1000;0.1000;1.9293;0.7411;-0.0313;0.1000;0.0289;0.8760;0.1000;0.8701;0.1000;0.1000;0.9200]
        w = v[4,k]
        # INERCIAL MATRIX
        M11=x[1];
        M12=0;
        M13=0;
        M14=x[2];
        M21=0;
        M22=x[3] ;
        M23=0;
        M24=x[4];
        M31=0;
        M32=0;
        M33=x[5];
        M34=0;
        M41=b*x[6];
        M42=a*x[7];
        M43=0;
        M44=x[8]*(a^2 + b^2)+x[9];

        M=[M11 M12 M13 M14; M21 M22 M23 M24; M31 M32 M33 M34; M41 M42 M43 M44];

        #CENTRIOLIS MATRIX
        C11=x[10];
        C12=w*x[11];
        C13=0;
        C14=a*w*x[12];
        C21=w*x[13];
        C22=x[14];
        C23=0;
        C24=b*w*x[15];
        C31=0;
        C32=0;
        C33=x[16];
        C34=0;
        C41=a*w*x[17];
        C42=b*w*x[18];
        C43=0;
        C44=x[19];

        C=[C11 C12 C13 C14; C21 C22 C23 C24; C31 C32 C33 C34; C41 C42 C43 C44];

        # GRAVITATIONAL MATRIX
        G11=0;
        G21=0;
        G31=0;
        G41=0;

        G=[G11;G21;G31;G41];

        vref[:,k] = v_ref

        vp = inv(M)*(vref[:,k]-C*v[:,k]-G);
        v[:,k+1] = v[:,k] +vp*ts;

        # Cinematica del robot
        psi = h[4,k]
        rot = [cos(psi) -sin(psi) 0 0; sin(psi) cos(psi) 0 0; 0 0 1 0; 0 0 0 1]
        hp[:,k] = rot*v[:,k] # CAmbio aqui      

        # Integracion numerica
        h[:,k+1] = hp[:,k]*(1/hz) + h[:,k] 
        
        # Enviar valores de posicion y velocidad (Odometria)
        set_odo(pub_pos, h[:,k+1], v[:,k+1])
        
        rossleep(loop_rate) 
        toc = time()
        dt = toc-tic
    end    
end

if ! isinteractive()
    init_node("Robot")  
    sub_vc = Subscriber{Twist}("/UAV/Controller", vc_callback,  queue_size=10)
    sub_run = Subscriber{Int32MultiArray}("/UAV/Config", config_callback,  queue_size=10)
    
    # Lanza los publisher del sistema
    pub_config = Publisher{Int32MultiArray}("/UAV/Config", queue_size=10)
    pub_pos = Publisher{Odometry}("/UAV/Odometry", queue_size=10)
    pub = Publisher{Twist}("/UAV/Controller", queue_size=10)
    main()
end