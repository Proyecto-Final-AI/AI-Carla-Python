import glob
import os
import sys
import random
import time
import numpy as np
import cv2
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

SHOW_PREVIEW = False
IM_WIDTH = 640
IM_HEIGHT = 480

class CarEnv:
    SHOW_CAM = SHOW_PREVIEW
    STEER_AMT = 1.0
    THROTTLE = 0.0
    BRAKE = 0.0
    im_width = IM_WIDTH
    im_height = IM_HEIGHT
    front_camera = None

    def __init__(self):
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(20.0)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.model_3 = self.blueprint_library.filter("model3")[0]

    def reset(self):
        self.collision_hist = []
        self.obstacle_hist = []
        self.actor_list = []

        self.THROTTLE = 0.0
        self.BRAKE = 0.0

        self.transform = random.choice(self.world.get_map().get_spawn_points())
        self.vehicle = self.world.spawn_actor(self.model_3, self.transform)
        self.actor_list.append(self.vehicle)

        self.rgb_cam = self.blueprint_library.find('sensor.camera.rgb')
        self.rgb_cam.set_attribute("image_size_x", f"{self.im_width}")
        self.rgb_cam.set_attribute("image_size_y", f"{self.im_height}")
        self.rgb_cam.set_attribute("fov", f"110")

        transform = carla.Transform(carla.Location(x=2.5, z=0.7))
        self.sensor = self.world.spawn_actor(self.rgb_cam, transform, attach_to=self.vehicle)
        self.actor_list.append(self.sensor)
        self.sensor.listen(lambda data: self.process_img(data))

        self.vehicle.apply_control(carla.VehicleControl(throttle=self.THROTTLE, brake=self.BRAKE))
        time.sleep(4)

        colsensor = self.blueprint_library.find("sensor.other.collision")
        self.colsensor = self.world.spawn_actor(colsensor, transform, attach_to=self.vehicle)
        self.actor_list.append(self.colsensor)
        self.colsensor.listen(lambda event: self.collision_data(event))

        ## Obstacle Sensors

        frontal_transform = carla.Transform(carla.Location(x=2.5, y=0, z=1.5))
        frontal_right_transform = carla.Transform(carla.Location(x=2.5, y=1, z=1.5), carla.Rotation(yaw=45))
        frontal_left_transform = carla.Transform(carla.Location(x=2.5, y=-1, z=1.5), carla.Rotation(yaw=315))

        central_right_transform = carla.Transform(carla.Location(x=0, y=1, z=1.5), carla.Rotation(yaw=90))
        central_left_transform = carla.Transform(carla.Location(x=0, y=-1, z=1.5), carla.Rotation(yaw=270))

        back_transform = carla.Transform(carla.Location(x=-2.5, y=0, z=1.5), carla.Rotation(yaw=180))
        back_right_transform = carla.Transform(carla.Location(x=-2.5, y=1, z=1.5), carla.Rotation(yaw=135))
        back_left_transform = carla.Transform(carla.Location(x=-2.5, y=-1, z=1.5), carla.Rotation(yaw=225))

        self.obs_sensor = self.blueprint_library.find("sensor.other.obstacle")
        self.obs_sensor_long = self.blueprint_library.find("sensor.other.obstacle")
        self.obs_sensor_long.set_attribute("distance", "25")

        self.obs_sensor_frontal = self.world.spawn_actor(self.obs_sensor, frontal_transform, attach_to=self.vehicle)
        self.obs_sensor_frontal_long = self.world.spawn_actor(self.obs_sensor_long, frontal_transform, attach_to=self.vehicle)
        self.obs_sensor_frontal_right = self.world.spawn_actor(self.obs_sensor, frontal_right_transform, attach_to=self.vehicle)
        self.obs_sensor_frontal_left = self.world.spawn_actor(self.obs_sensor, frontal_left_transform, attach_to=self.vehicle)
        
        self.obs_sensor_central_right = self.world.spawn_actor(self.obs_sensor, central_right_transform, attach_to=self.vehicle)
        self.obs_sensor_central_left = self.world.spawn_actor(self.obs_sensor, central_left_transform, attach_to=self.vehicle)
        
        self.obs_sensor_back = self.world.spawn_actor(self.obs_sensor, back_transform, attach_to=self.vehicle)
        self.obs_sensor_back_right = self.world.spawn_actor(self.obs_sensor, back_right_transform, attach_to=self.vehicle)
        self.obs_sensor_back_left = self.world.spawn_actor(self.obs_sensor, back_left_transform, attach_to=self.vehicle)

        self.actor_list.append(self.obs_sensor_frontal)
        self.actor_list.append(self.obs_sensor_frontal_long)
        self.actor_list.append(self.obs_sensor_frontal_right)
        self.actor_list.append(self.obs_sensor_frontal_left)
        self.actor_list.append(self.obs_sensor_central_right)
        self.actor_list.append(self.obs_sensor_central_left)
        self.actor_list.append(self.obs_sensor_back)
        self.actor_list.append(self.obs_sensor_back_right)
        self.actor_list.append(self.obs_sensor_back_left)

        self.obs_sensor_frontal.listen(lambda event: self.obstacle_data(event))
        self.obs_sensor_frontal_long.listen(lambda event: self.obstacle_data(event))
        self.obs_sensor_frontal_right.listen(lambda event: self.obstacle_data(event))
        self.obs_sensor_frontal_left.listen(lambda event: self.obstacle_data(event))
        
        self.obs_sensor_central_right.listen(lambda event: self.obstacle_data(event))
        self.obs_sensor_central_left.listen(lambda event: self.obstacle_data(event))
        
        self.obs_sensor_back.listen(lambda event: self.obstacle_data(event))
        self.obs_sensor_back_right.listen(lambda event: self.obstacle_data(event))
        self.obs_sensor_back_left.listen(lambda event: self.obstacle_data(event))

        while self.front_camera is None:
            time.sleep(0.01)

        self.vehicle.apply_control(carla.VehicleControl(throttle=self.THROTTLE, brake=self.BRAKE))

        return self.front_camera

    def collision_data(self, event):
        self.collision_hist.append(event)
    
    def obstacle_data(self, event):
        self.obstacle_hist.append(event)

    def process_img(self, image):
        i = np.array(image.raw_data)
        i2 = i.reshape((self.im_height, self.im_width, 4))
        i3 = i2[:, :, :3]
        if self.SHOW_CAM:
            cv2.imshow("", i3)
            cv2.waitKey(1)
        self.front_camera = i3

    def set_control(self, steer_mov, throttle, brake):
        self.THROTTLE = throttle
        self.BRAKE = brake
        self.vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer_mov+self.STEER_AMT, brake=brake))

    def get_kmh(self):
        v = self.vehicle.get_velocity()
        kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
        return kmh