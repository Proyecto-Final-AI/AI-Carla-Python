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

SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480
LIST_ACTORS = []

def create_car(blueprint_library, world):
   
    # Obtenemos el plano del modelo del coche que vamos a crear 
    car_model = blueprint_library.filter("model3")[0]
    
    # Seleccionamos aleatoriamente un punto de respawn
    spawn_point = random.choice(world.get_map().get_spawn_points())
    
    # Creamos e insertamos el coche en el mundo con el blueprint creado anteriormente y el punto de respawn aleatorio
    car = world.spawn_actor(car_model, spawn_point)
    
    # Hacemos que el coche acelere y el volante esté recto
    #coche.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    
    #Hacemos que el coche conduzca solo
    car.set_autopilot(True)
    
    return car


def create_camera(blueprint_library, world, car):
    
    # Blueprint para la cámara sensor.camera.semantic_segmentation
    bp_cam = blueprint_library.find("sensor.camera.semantic_segmentation")
    bp_cam.set_attribute("image_size_x", f"{SCREEN_WIDTH}")
    bp_cam.set_attribute("image_size_y", f"{SCREEN_HEIGHT}")
    bp_cam.set_attribute("fov", "110")
    
    # Creamos el spawn point de la cámara un poco delante y un poco arriba para simular que está en la parte superior del coche
    spawn_point_cam = carla.Transform(carla.Location(x=2.5, z=0.6))
    
    # Creamos el sensor y lo spawneamos en el spawnpoint de la cámara que es relativo y lo pegamos al coche
    cam_sensor = world.spawn_actor(bp_cam, spawn_point_cam, attach_to=car)
     
    return cam_sensor 

def draw_lines(img,lines):
    try:
        for line in lines:
            coords = line[0]
            cv2.line(img, (coords[0], coords[1]), (coords[2], coords[3]), [255,255,255], 3)
    except:
        pass
    
    
def drive_zone(edges, vertices):
    mask = np.zeros_like(edges)
    cv2.fillPoly(mask, vertices,255)
    masked = cv2.bitwise_and(edges,mask)
    return masked

def process_img(image):
    image.convert(carla.ColorConverter.CityScapesPalette)  
    i = np.array(image.raw_data)
    i2 = i.reshape((SCREEN_HEIGHT, SCREEN_WIDTH, 4))
    # Array dónde guardamos los datos de la cámara en RGB, porque en realidad lo guarda en RGBA
    i3 = i2[:, :, :3]
    cv2.imshow("Edges", i3)

    #Creación del edge y ejecucion
    edges = cv2.Canny(i3,200,300)
    #Creamos el tamaño de la parte de la camara con la que nos queremos quedar
    vertices = np.array([ [0,480], [0,300], [200,200], [480,200], [640,300], [640,480],])
    draw_zone = drive_zone(edges, np.int32([vertices]))
    
    lines = cv2.HoughLinesP(draw_zone, 1, np.pi/180, 180, 20, 15)
    draw_lines(draw_zone, lines)
    
    cv2.waitKey(25)
    

def main():
    try:
        # Creamos el cliente
        cliente = carla.Client('localhost', 2000)
        cliente.set_timeout(5.0)
        
        # Creamos el mundo
        world = cliente.get_world()
        
        # Recogemos la librería de blueprints
        blueprint_library = world.get_blueprint_library()
        
        #Obtenemos el coche a través de la función create_car
        car = create_car(blueprint_library, world) 
        # Añadimos el coche a la lista de actores
        LIST_ACTORS.append(car)
        print("Coche añadido")    

        #Obtenemos la camara a través de la función create_camera
        camera = create_camera(blueprint_library, world, car)
        # Añadimos la camara a la lista de actores
        LIST_ACTORS.append(camera)
        print("Camara añadida") 
        
        camera.listen(lambda data :process_img(data))   
        time.sleep(1115)        
        
    finally:
        for actor in LIST_ACTORS:
            actor.destroy()
        print("Destruidos todos los actores")    


main()