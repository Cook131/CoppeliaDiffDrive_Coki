def sysCall_init():
    global sim, math, sphere, start_time, initial_position

    sim = require('sim')
    math = require('math')  # necesario para usar sin(), pi, etc.

    # Obtener el handle del objeto llamado exactamente "Sphere"
    sphere = sim.getObject('/Sphere')

    # Guardar la posición inicial de la esfera
    initial_position = sim.getObjectPosition(sphere, -1)

    # Guardar el tiempo de inicio
    start_time = sim.getSimulationTime()
    
def sysCall_actuation():
    global sim, math, sphere, start_time, initial_position

    # Calcular tiempo transcurrido
    t = sim.getSimulationTime() - start_time

    # Parámetros del movimiento
    amplitude = 0.2      # en metros
    frequency = 0.5      # en Hz

    # Calcular nueva posición X usando una onda seno
    new_x = initial_position[0] + amplitude * math.sin(2 * math.pi * frequency * t)

    # Aplicar la nueva posición (sólo cambia X)
    sim.setObjectPosition(sphere, -1, [new_x, initial_position[1], initial_position[2]])
    
def sysCall_sensing():
    pass

def sysCall_cleanup():
    pass
