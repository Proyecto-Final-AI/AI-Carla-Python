import glob
import os
import sys
import random
import time
import numpy as np
import cv2
import threading

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

IM_WIDTH = 1280
IM_HEIGHT = 720

def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1*(3/5))
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])

def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))
    
    left_fit_average = np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis=0)

    left_line = make_coordinates(image, left_fit_average)
    right_line = make_coordinates(image, right_fit_average)

    return np.array([left_line, right_line])

def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny

def display_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            cv2.line(line_image, (x1, y1), (x2, y2), [0,255,0], thickness=10) # Con cv2 dibujamos las lineas en la imagen
    return line_image

def region_of_interest(image):
    polygons = np.array([[(200, 700), (1120, 700), (630, 310)]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

# https://youtu.be/2hM44nr7Wms?t=1889
def process_img(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    # Array dónde guardamos los datos de la cámara en RGB, porque en realidad lo guarda en RGBA
    i3 = i2[:, :, :3]

    canny_image = canny(i3)

    cropped_image = region_of_interest(canny_image)

    lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
    averaged_lines = average_slope_intercept(i3, lines)
    line_image = display_lines(i3, averaged_lines)

    combo_image = cv2.addWeighted(i3, 0.8, line_image, 1, 1)

    cv2.imshow("Video", combo_image)
    cv2.waitKey(1)

def obst_det(data):
    print(data.distance)

def thread_function(coche):
    while True:
        control = coche.get_control()
        print(control)
        time.sleep(2)

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
    bp_camara = blueprint_library.find("sensor.camera.rgb")
    bp_camara.set_attribute("image_size_x", f"{IM_WIDTH}")
    bp_camara.set_attribute("image_size_y", f"{IM_HEIGHT}")
    bp_camara.set_attribute("fov", "110")

    # Creamos el spawn point de la cámara un poco delante y un poco arriba para simular que está en la parte superior del coche
    spawn_point_camera = carla.Transform(carla.Location(x=1.2, z=1.6))

    # Creamos el sensor y lo spawneamos en el spawnpoint de la cámara que es relativo y lo pegamos al coche
    sensor_camara = world.spawn_actor(bp_camara, spawn_point_camera, attach_to=coche)
    # Añadimos el sensor de la cámara a la lista de actores para que al finalizar se elimine
    lista_actores.append(sensor_camara)
    
    # Un listener de la cámara con una función lambda a la que pasará los datos de la misma
    sensor_camara.listen(lambda data: process_img(data))

    # Blueprint para el sensor de obstaculos
    # bp_sensor_obstaculos = blueprint_library.find("sensor.other.obstacle")

    # # SpawnPoints para el sensor de obstaculos
    # spawn_point_sensor_obstaculos = carla.Transform(carla.Location(x=2.5, z=0.5))

    # sensor_obstaculos = world.spawn_actor(bp_sensor_obstaculos, spawn_point_sensor_obstaculos, attach_to=coche)
    # lista_actores.append(sensor_obstaculos)

    # sensor_obstaculos.listen(lambda data: obst_det(data))

    # Creación de un thread
    # thread = threading.Thread(target=thread_function, args=(coche,))
    # thread.start()

    while True:
        entrada = input()
        if entrada == 'Q':
            break


finally:
    # Eliminamos todos y cada uno de los actores de la lista
    for actor in lista_actores:
        actor.destroy()
    print("Todos los actores destruidos")