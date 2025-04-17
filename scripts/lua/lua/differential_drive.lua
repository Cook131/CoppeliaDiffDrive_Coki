sim=require'sim'
simROS2=require'simROS2'

function sysCall_init()
    motorLeft=sim.getObject("../leftMotor")
    motorRight=sim.getObject("../rightMotor")

    -- Parámetros del robot diferencial (ajusta si tu modelo es diferente)
    wheelRadius = 0.0975     -- radio de la rueda en metros
    wheelBase = 0.331        -- distancia entre ruedas en metros (de Pioneer)

    -- Inicializar velocidades
    linearVel = 0.0
    angularVel = 0.0

    -- Suscribirse al tópico /cmd_vel
    velSub = simROS2.createSubscription('/cmd_vel', 'geometry_msgs/msg/Twist', 'twist_callback')
end

-- Callback cuando llega un mensaje de tipo Twist
function twist_callback(msg)
    linearVel = msg.linear.x       -- en m/s
    angularVel = msg.angular.z     -- en rad/s
end

function sysCall_actuation()
    -- Cinemática diferencial inversa
    leftSpeed = (linearVel - (wheelBase / 2) * angularVel) / wheelRadius
    rightSpeed = (linearVel + (wheelBase / 2) * angularVel) / wheelRadius

    -- Aplicar velocidades a las ruedas
    sim.setJointTargetVelocity(motorLeft, leftSpeed)
    sim.setJointTargetVelocity(motorRight, rightSpeed)
end
