import glob
import os
import sys

try:
    sys.path.append(glob.glob('../../WindowsNoEditor/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import time
import numpy as np
import cv2

actors_list = []
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480
class Carla:
        
    def __init__(self):
        
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()

class Car:
    
    def __init__(self, carla):
        self.blueprint_car = carla.carla_client.blueprint_library.filter("model3")[0]
        self.spawn_point = random.choice(carla.carla_client.world.get_map().get_spawn_points())
        self.car = carla.carla_client.world.spawn_actor(blueprint_car, spawn_point)
        self.car.set_autopilot(True)   

class Camera:
    
    def __init__(self,carla,car):
        
        self.blueprint_cam = carla.carla_client.blueprint_library.find("sensor.camera.rgb")
        self.blueprint_cam.set_attribute("image_size_x", f"{SCREEN_WIDTH}")
        self.blueprint_cam.set_attribute("image_size_y", f"{SCREEN_HEIGHT}")
        self.blueprint_cam.set_attribute("fov", "110")
        
        self.spawn_point_cam = carla.Transform(carla.Location(x=2.5, z=0.6))

        self.camera = carla.carla_client.world.spawn_actor(blueprint_cam, spawn_point_cam, attach_to=car.car)


def show_video(image):
    
    i = np.array(image.raw_data)
    i2 = i.reshape((SCREEN_HEIGHT, SCREEN_WIDTH, 4))
    # Array dónde guardamos los datos de la cámara en RGB, porque en realidad lo guarda en RGBA
    i3 = i2[:, :, :3]
    cv2.imshow("Edges", i3)
    cv2.waitKey(25)

def cahnge_spawn_point(car):
    car.


def main():

    try:
        
        carla_client = Carla()
        car = Car(self,carla_client)
        actors_list.append(car.car)
        
        cam = create_cam(self,carla_client, car)
        actors_list.append(cam.camera)

        cam.listen(lambda image: show_video(image))
        
        while(True):
        
            spawn_new_car(carla_client)
            
            time.sleep(5)        
            for actor in actors_list:
                actor.destroy()
            print("Destruidos todos los actores")  
    
    finally:
        for actor in actors_list:
            actor.destroy()
        print("Destruidos todos los actores")    

main()