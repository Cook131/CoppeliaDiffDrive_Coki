sim = require 'sim'
simROS2 = require('simROS2')

-- Obtener el handle del sensor de visión
visionSensor = sim.getObject('/visionSensor')

-- Publicador para el tópico de la cámara
imagePub = nil

function sysCall_init()
    -- Crear publicador en /camera/image
    imagePub = simROS2.createPublisher('/PioneerP3DX/visionSensor', 'sensor_msgs/msg/Image')
end

function sysCall_actuation()
    -- Obtener resolución del sensor (ancho, alto)
    local resolution = sim.getVisionSensorResolution(visionSensor)
    local width = resolution[1]
    local height = resolution[2]

    -- Capturar imagen (valores en [0,1])
    local image = sim.getVisionSensorImage(visionSensor)

    -- Armar mensaje sensor_msgs/Image
    local msg = {}
    msg.header = {}             -- Timestamp o frame_id
    msg.height = height
    msg.width = width
    msg.encoding = 'rgb8'       -- Formato RGB 8 bits
    msg.is_bigendian = 1
    msg.step = width * 3        -- Bytes por fila: ancho * 3 canales
    msg.data = {}

    -- Convertir cada valor de [0,1] a [0,255]
    for i = 1, #image do
        msg.data[i] = math.floor(image[i] * 255)
    end

    -- Publicar la imagen en ROS2
    simROS2.publish(imagePub, msg)
end

function sysCall_cleanup()
    -- Cerrar el publicador al terminar
    simROS2.shutdownPublisher(imagePub)
end
