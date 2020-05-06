import glob
import os
import sys
import random
import time
import numpy as np
import cv2

try:
    sys.path.append(glob.glob('../../WindowsNoEditor/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

IM_WIDTH = 700
IM_HEIGHT = 500


def draw_lines(img,lines):
    for line in lines:
        coords = line[0]
        cv2.line(img, (coords[0], coords[1]), (coords[2], coords[3]), [255,255,255], 3)

def drive_zone(edges, vertices):
    mask = np.zeros_like(edges)
    cv2.fillPoly(mask, vertices,255)
    masked = cv2.bitwise_and(edges,mask)
    return masked

# https://youtu.be/2hM44nr7Wms?t=1889
def process_img(image):
    image.convert(carla.ColorConverter.CityScapesPalette)  
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    # Array dónde guardamos los datos de la cámara en RGB, porque en realidad lo guarda en RGBA
    i3 = i2[:, :, :3]

    #Creación del edge y ejecucion
    edges = cv2.Canny(i3,200,300)
    
    edges = cv2.GaussianBlur(edges,(5,5),0)
    vertices = np.array([ [0,500], [0,300], [200,200], [500,200], [700,300], [700,500],])
    process_img = drive_zone(edges,np.int32([vertices]))
    
    lines = cv2.HoughLinesP(process_img, 1, np.pi/180, 180, 20, 15)

    draw_lines(process_img,lines)

    cv2.imshow("Edges", process_img)
    
    return i3/255.0

lista_actores = []

try:
    # Creamos el cliente
    cliente = carla.Client('localhost', 2000)
    cliente.set_timeout(5.0)
    # Creamos el mundo
    world = cliente.get_world()
    # Recogemos la librería de blueprints
    blueprint_library = world.get_blueprint_library()

    # Creamos el blueprint del coche para al crear el coche añadir el blueprint
    model3 = blueprint_library.filter("model3")[0]
    print(model3)

    # Seleccionamos aleatoriamente un punto de respawn
    spawn_point = random.choice(world.get_map().get_spawn_points())
    # Creamos e insertamos el coche en el mundo con el blueprint creado anteriormente y el punto de respawn aleatorio
    coche = world.spawn_actor(model3, spawn_point)
    
    # Hacemos que el coche acelere y el volante esté recto
    #coche.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    coche.set_autopilot(True)
    # Añadimos el coche a la lista de actores
    lista_actores.append(coche)

    # Blueprint para la cámara
    bp_camara = blueprint_library.find("sensor.camera.semantic_segmentation")
    bp_camara.set_attribute("image_size_x", f"{IM_WIDTH}")
    bp_camara.set_attribute("image_size_y", f"{IM_HEIGHT}")
    bp_camara.set_attribute("fov", "110")

    # Creamos el spawn point de la cámara un poco delante y un poco arriba para simular que está en la parte superior del coche
    spawn_point_camera = carla.Transform(carla.Location(x=2.5, z=0.7))

    # Creamos el sensor y lo spawneamos en el spawnpoint de la cámara que es relativo y lo pegamos al coche
    sensor_camara = world.spawn_actor(bp_camara, spawn_point_camera, attach_to=coche)
    # Añadimos el sensor de la cámara a la lista de actores para que al finalizar se elimine
    lista_actores.append(sensor_camara)
    
    # Un listener de la cámara con una función lambda a la que pasará los datos de la misma
    sensor_camara.listen(lambda data: process_img(data))
    
    time.sleep(1115)

finally:
    # Eliminamos todos y cada uno de los actores de la lista
    for actor in lista_actores:
        actor.destroy()
    print("Todos los actores destruidos")