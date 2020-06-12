import glob
import os
import sys
import random
import time
import numpy as np
import cv2
import math
from collections import deque
from keras.applications.xception import Xception
from keras.layers import Dense, GlobalAveragePooling2D
from keras.optimizers import Adam
from keras.models import Model
from keras.callbacks import TensorBoard

import tensorflow as tf
import keras.backend.tensorflow_backend as backend
from threading import Thread

from tqdm import tqdm

try:
    sys.path.append(glob.glob('WindowsNoEditor/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

#Tamaño de la cámara
CAM_WIDTH = 640
CAM_HEIGHT = 480
#Mostrar camara o no
SHOW_CAM = False

#Variable con el valor de aceleración
THROTTLE = 1.0
#Variable con el valor de frenada
BRAKE = 0.0

#Segundos que dura cada episodio
SECONDS_PER_EPISODE = 20

#Tamaño de la deque de replays
REPLAY_MEMORY_SIZE = 5_000
MIN_REPLAY_MEMORY_SIZE = 1_000

MINIBATCH_SIZE = 16

PREDICTION_BATCH_SIZE = 1
TRAINIG_BATCH_SIZE = MINIBATCH_SIZE // 4
UPDATE_TARGET_EVERY = 5

MODEL_NAME = "Xception"

#Total de la CPU que utilizaremos
MEMORY_FACTION = 0.8

#Número de episodios que hara
EPISODES = 100

DISCOUNT = 0.99

#Por defecto la mejor opción
epsilon = 1

#Variable de expectativa de porcentaje de acierto al cual debe llegar
EPSILON_DECAY = 0.95

#Mínimo porcentaje que coje
MIN_EPSILON = 0.001

AGGREGATE_STATE_EVERY = 10

MEMORY_FRACTION = 0.4


# Nuestra propia classe de TensorBoard para crear las 
class ModifiedTensorBoard(TensorBoard):

    # Reescribimos el constructor para poner el primer episodio y escribir
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.step = 1
        self.writer = tf.compat.v1.summary.FileWriter(self.log_dir)

    # Hacemos que pase de todas las funciones
    def set_model(self, model):
        pass
   
    def on_batch_end(self, batch, logs=None):
        pass

    def on_train_end(self, _):
        pass
    
    # Cambiamos como se guardan a traves del numero de prueba
    def on_epoch_end(self, epoch, logs=None):
        self.update_stats(**logs)
        
    # Metodo para sobreescribir las estadísticas
    def update_stats(self, **stats):
        self._write_logs(stats, self.step)





class CarEnvironmet():
    #Variables de la cámara
    show_cam = SHOW_CAM
    cam_width = CAM_WIDTH
    cam_height = CAM_HEIGHT

    #Variables de acción al empezar ( aceleración y frenada consecutivamente )
    throttle = THROTTLE
    brake = BRAKE
    
    #Variable los datos de la cámara delantera
    front_camera = None

    #Constructor de CarEnvironment
    def __init__(self):
        #Puntos que serviran para saber si conduce como queremos o no
        self.points = 0
        #Definir el cliente con el que nos conectaremos a carla
        self.carla_client = carla.Client("localhost", 2000)
        self.carla_client = carla.set_timeout(5000)
        
        #Definir el mundo donde va a crearse el coche
        #Hacemos que se cargue el mapa 6, si no es el actual se cambia
        if (carla_client.get_world().get_map().name != "Town06"):
            carla_client.load_world("Town06")
        self.world = carla_client.get_world()
        
        #Variable que contiene todos los modelos de los objetos
        self.blueprint_library = self.world.get_blueprint_library()
        
        #Variable con el modelo del coche
        self.car_model = self.blueprint_library.filter("model3")[0]
        
        
    #Funcion de reset del coche      
    def reset_car(self):    
        #Variable de array que contendrá las colisiones que tenga el coche
        self.collision_hits = []
        
        #Creamos la lista de actores que contendra el coche y todos los sensores
        self.actor_list = []
        
        #Creamos un array con la lista de las veces que ha invadido algún carril
        self.lane_invasion = []

        #Variable que contiene las ditáncias de los sensores
        self.sensor_distances = []

        #Variable con la velocidad actual del coche
        self.car_vel = 0

       
        #Crear el sitio de spawn del coche de forma random
        self.car_spawn_point = random.choice(self.world.get_map().get_spawn_points())     
        #Creamos el coche
        self.car = self.world.spawn_actor(self.car_model, self.car_spawn_point)  
        #Metemos el coche dentro de la lista de actores
        self.actor_list.append(self.car)
        
        
        #Creamos el sensor de la cámara en la parte frontal del coche
        self.rgb_cam = self.blueprint_library.find("sensor.camera.rgb") 
        #Definimos el ancho y largo de la cámara a parte del fov
        self.rgb.set_attribute("image_size_x", f"{self.cam_width}")
        self.rgb.set_attribute("image_size_y", f"{self.cam_height}")
        self.rgb.set_attribute("fov", f"110")
        #Punto de spawn de la cámara
        cam_spawn = carla.Transform(carla.Location(x=2.5, z=0.7))
        #Hacemos spawn de la cámara y la añadimos a la lista de actores
        self.cam = self.world.spawn_actor(self.rgb_cam, cam_spawn, attach_to=self.car)
        self.actor_list.append(self.cam)
        #Creamos una función Lamba que nos lleva a una función que nos procesa la imagen a través de los datos que nos pasa el sensor self.cam
        self.sensor.listen(lambda image: self.process_image(image))

        #Creamos el sensor de colisiones
        collision_sensor = self.blueprint_library.find("sensor.other.collision")
        #Hacemos spawn en el mismo sitio que la cámara y que esté vinculado al coche
        self.collision = self.world.spawn_actor(collision_sensor, cam_spawn, attach_to=self.car)
        #Como con la cámara cada vez que el sensor se active llamaremos a una función con los datos de la colisión a través de una función lambda
        self.collision.listen(lambda event: self.collision_event(event))


        #Creamos el sensor para controlar el carril
        #TODO:



        #En esta parte crearemos todos los sensores de distáncia que tiene el coche
        self.obs_sensor = self.blueprint_library.find("sensor.other.obstacle")

        #Sensor frontal
        frontal_transform = carla.Transform(carla.Location(x=2.5, y=0, z=1.5))
        self.obs_sensor_frontal = self.world.spawn_actor(self.obs_sensor, frontal_transform, attach_to=self.car)
        self.actor_list.append(self.obs_sensor_frontal)
        self.obs_sensor_frontal.listen(lambda event: self.distance_sensor(event, "frontal"))

        #Sensor frontal derecho
        frontal_right_transform = carla.Transform(carla.Location(x=2.5, y=1, z=1.5), carla.Rotation(yaw=45))
        self.obs_sensor_frontal_right = self.world.spawn_actor(self.obs_sensor, frontal_right_transform, attach_to=self.car)
        self.actor_list.append(self.obs_sensor_frontal_right)
        self.obs_sensor_frontal_right.listen(lambda event: self.distance_sensor(event,"frontal_right"))
       
        #Sensor frontal izquierdo
        frontal_left_transform = carla.Transform(carla.Location(x=2.5, y=-1, z=1.5), carla.Rotation(yaw=315))
        self.obs_sensor_frontal_left = self.world.spawn_actor(self.obs_sensor, frontal_left_transform, attach_to=self.car)
        self.actor_list.append(self.obs_sensor_frontal_left)
        self.obs_sensor_frontal_left.listen(lambda event: self.distance_sensor(event,"frontal_left"))

        #Sensor central derecho
        central_right_transform = carla.Transform(carla.Location(x=0, y=1, z=1.5), carla.Rotation(yaw=90))
        self.obs_sensor_central_right = self.world.spawn_actor(self.obs_sensor, central_right_transform, attach_to=self.car)
        self.actor_list.append(self.obs_sensor_central_right)
        self.obs_sensor_central_right.listen(lambda event: self.distance_sensor(event,"central_right"))

        #Sensor central izquierdo
        central_left_transform = carla.Transform(carla.Location(x=0, y=-1, z=1.5), carla.Rotation(yaw=270))
        self.obs_sensor_central_left = self.world.spawn_actor(self.obs_sensor, central_left_transform, attach_to=self.car)
        self.actor_list.append(self.obs_sensor_central_left)
        self.obs_sensor_central_left.listen(lambda event: self.distance_sensor(event, "central_left"))

        #Sensor posterior
        back_transform = carla.Transform(carla.Location(x=-2.5, y=0, z=1.5), carla.Rotation(yaw=180))
        self.obs_sensor_back = self.world.spawn_actor(self.obs_sensor, back_transform, attach_to=self.car)
        self.actor_list.append(self.obs_sensor_back)
        self.obs_sensor_back.listen(lambda event: self.distance_sensor(event, "back"))

        #Sensor posterior derecho
        back_right_transform = carla.Transform(carla.Location(x=-2.5, y=1, z=1.5), carla.Rotation(yaw=135))
        self.obs_sensor_back_right = self.world.spawn_actor(self.obs_sensor, back_right_transform, attach_to=self.car)
        self.actor_list.append(self.obs_sensor_back_right)
        self.obs_sensor_back_right.listen(lambda event: self.distance_sensor(event, "back_right"))

        #Sensor posterior izquierdo
        back_left_transform = carla.Transform(carla.Location(x=-2.5, y=-1, z=1.5), carla.Rotation(yaw=225))
        self.obs_sensor_back_left = self.world.spawn_actor(self.obs_sensor, back_left_transform, attach_to=self.car)
        self.actor_list.append(self.obs_sensor_back_left)
        self.obs_sensor_back_left.listen(lambda event: self.distance_sensor(event, "back_left"))


        #Creamos un time sleep para asegurarnos que todo está spawneado correctaente
        time.sleep(2)

        #Variable que contiene el timestamp de cuando se empieza el episodio
        self.episode_start = time.time()

        #Hacemos que el coche vaya para adelante nada mas hacer spawn
        self.car.apply_control(carla.VehicleControl(throttle=self.throttle, brake=self.brake))

        return self.front_camera



    #Función que calcula los puntos que pierde cuando pasa algún evento en los sensores
    def distance_sensor(self, event, key):
        self.distance_sensor[key] = event.distance
        
        #También guardamos la velocidad que lleva el coche actualmentes
        v = self.car.get_velocity()
        self.car_vel = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
        
        decisiones = Decisiones(distance_sensor)
        
        self.reward = self.reward + decisiones.points

    #Funcón para añadir a la lista lane_invasion que ha invadido un carril
    def lane_invasion(self, event):
        self.lane_invasion.append(event)

    #Función que añade una colisión al array de colisiones del coche
    def collision_event(self, event):
        self.collision_hits.append(event)

    #Función para mostrar y guardar los datos de la cámara    
    def process_img(self, image):
        #Creamos una variable que contiene todos los datos de la cámara
        i = np.array(image.raw_data)
        
        #Lo reescalamos a el tamño que queremos mostrar
        i2 = i.reshape((self.im_height, self.im_width, 4))
        i3 = i2[:, :, :3]
        
        #Si la variable global SHOW_CAM és True se muestra la cámara
        if self.SHOW_CAM:
            cv2.imshow("", i3)
            cv2.waitKey(1)

        #Guardamos los datos de la cámara aunque no se muestren
        self.front_camera = i3
    
    
    
    def step(self, car_action):
        
        #Ahora vamos a crear los steps para que el coche sepa lo que tiene que hacer si perdir ayuda a nadie        
        if action == 0:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer= 0))
            self.count_only_rigth = 0;
            self.count_only_left = 0;
        elif action == 1:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer= 0, brake=1.0))
            self.count_only_rigth = 0;
            self.count_only_left = 0;
        elif action == 2:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1*self.STEER_AMT))
            self.count_only_rigth=self.count_only_rigth+1;

        elif action == 3:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1*self.STEER_AMT))
            self.count_only_left=self.count_only_left+1;
            
            
        if len(self.collision_hits) != 0:
            done = False
            reward = reward - 100
        elif:
            done: False
            reward = reward + 1  
        
        if self.count_only_rigth > 20:
            reward = reward -100
        
        if self.count_only_left > 20:
            reward = reward -100
        
        reward = reward + self.points
                
        #Si el episodio ya ha durado el tiempo que le hemos puesto se acaba
        if self.episode_start + SECONDS_PER_EPISODE > time.time:
            done = True

        
        return self.front_camera, reward, done, None 


#Crearemos un agente el cual aprenda a través de aprendizage reforzado
class DQNAgent:
    
    #Inicializamos el agente creado a traves del anterior modelo
    def __init__(self):
        
        #Creamos el model a traves de una función
        self.model = self.create_model()
        
        #Escogemos la red neuronal
        self.target_model = self.create_model()
        self.target_model.set_weights(self.model.get_weights())
        
        #Obtenemos el tamaño de replay_memory
        self.replay_memory = deque(maxlen=REPLAY_MEMORY_SIZE)
        
        #Creamos donde se guardará la información para las gráficas a través de la clase que hemos hecho antes
        self.tensorboard = ModifiedTensorBoard(log_dir=f"logs/{MODEL_NAME}-{int(time.time())}")
        #Variable que cuenta las veces que se hace
        self.target_update_counter = 0
        #Creamos el gráfico
        self.graph = tf.compat.v1.get_default_graph()
        
        #Variable para saber si se acaba
        self.terminate = False
        #Variable que te dice cual és el episodio que se está haciendo que és el último que se ha loggeado y lo iniciamos a 0
        self.last_logget_episode = 0
        #Variable que marca si el entrenamiento ha empezado
        self.training_initialize = False
        
    #Función para crear el modelo
    def create_model(self):
        #Creamos el modelo base
        base_model = Xception(weights=None, include_top=False, input_shape=(IM_HEIGHT, IM_WIDTH,3))
        x = base_model.output
        x = GlobalAveragePooling2D()(x)
        
        #Layer en el que li dones un input i et retorna un output
        pretidictions = Dense(3, activation="linear")(x)
        #Creamos el modelo final
        model = Model(inputs=base_model.input, outputs=predictions)
        model.compile(loss="mse", optimizer=Adam(lr=0.001), metrics=["accuracy"])
        
        return model
    
    
    #Función para añadir el step actual al array de replay_memory
    def update_replay_memory(self, new_state):
        self.replay_memory.append(new_state)
        
    #Funció para que la red neuronal entrene
    def train(self):
        #Comprobación de que la longitud actual de la variable donde se guardan los steps es más pequeña que las veces que le hemos puesto que sea lo minimo
        #Si no es más pequeña significa que ya puede aprender algo la red neuronal
        if len(self.replay_memory) < MIN_REPLAY_MEMORY_SIZE:
            return
        
        #Variable que hace la variación de la descendencia que tiene el step actual de la red neuronal
        minibatch = random.sample(self.replay_memory, MIN_REPLAY_MEMORY_SIZE)
        
        #Variable que contiene los datos del step que cojemos
        current_state = np.array(self.replay_memory, MINIBATCH_SIZE)/255
        
        #Creación de las listas actuales con los datos del modelo actual
        with self.graph.as_default():
            session = backend.get_session()
            init = tf.compat.v1.global_variables_initializer()
            session.run(init)
            current_qs_list = self.model.predict(current_states, PREDICTION_BATCH_SIZE)

        #Createmos las listas futuras con los valores del siguiente modelo
        new_current_states = np.array([transition[3] for transition in minibatch])/255
        with self.graph.as_default():
            session = backend.get_session()
            init = tf.compat.v1.global_variables_initializer()
            session.run(init)
            future_qs_list = self.target_model.predict(new_current_states, PREDICTION_BATCH_SIZE)

        X = []
        y = []
 
      #Actualizaremos la red neuronal una vez que nos aseguremos que se han acabado todos los replays minimos
        for index, (current_state, action, reward, new_state, done) in enumerate(minibatch):
            #Comprovamos que se haya acabado
            if not done:
                #Si aun no ha acabado cogemos el máximo numero que haya en la lista de los futuros
                max_future_q = np.max(future_qs_list[index])
                #Y creamos una nueva meta a la que llegar
                new_q = reward + DISCOUNT * max_future_q
            #Si ha acabado ponedermos la nueva q con el último valor del reward
            else:
                new_q = reward

            
            current_qs = current_qs_list[index]
            current_qs[action] = new_q

            X.append(current_state)
            y.append(current_qs)

        #Vamos a guardar los datos para que pueda tensorboard hacer las graficas
        log_this_step = False
        if self.tensorboard.step > self.last_logged_episode:
            log_this_step = True
            self.last_log_episode = self.tensorboard.step

        with self.graph.as_default():
            session = backend.get_session()
            init = tf.compat.v1.global_variables_initializer()
            session.run(init)
            self.model.fit(np.array(X)/255, np.array(y), batch_size=TRAINING_BATCH_SIZE, verbose=0, shuffle=False, callbacks=[self.tensorboard] if log_this_step else None)


        if log_this_step:
            self.target_update_counter += 1

        if self.target_update_counter > UPDATE_TARGET_EVERY:
            self.target_model.set_weights(self.model.get_weights())
            self.target_update_counter = 0
    
    #Buscaremos los valores Q en el estado que le hemos pasado por parametro
    def get_qs(self, state):
        #Devuelve el valor Q del modelo actual
        return self.model.predict(np.array(state).reshape(-1, *state.shape)/255)[0]


    def train_in_loop(self):
        X = np.random.uniform(size=(1, IM_HEIGHT, IM_WIDTH, 3)).astype(np.float32)
        y = np.random.uniform(size=(1, 3)).astype(np.float32)
        with self.graph.as_default():
            session = backend.get_session()
            init = tf.compat.v1.global_variables_initializer()
            session.run(init)
            self.model.fit(X,y, verbose=False, batch_size=1)

        self.training_initialized = True

        while True:
            if self.terminate:
                return
            self.train()
            time.sleep(0.01)
    


if __name__ == '__main__':
    FPS = 60
    # Para inicializar las estadísticas
    ep_rewards = [-200]

    # Para que cada prueba sea diferente
    random.seed(1)
    np.random.seed(1)
    tf.compat.v1.set_random_seed(1)

    #Para que se fraccione la memória en el caso de que hagan diferentes pruebas
    gpu_options = tf.compat.v1.GPUOptions(per_process_gpu_memory_fraction=MEMORY_FRACTION)
    backend.set_session(tf.compat.v1.Session(config=tf.compat.v1.ConfigProto(gpu_options=gpu_options)))

    # Creamos la carpeta de modelos
    if not os.path.isdir('models'):
        os.makedirs('models')

    # Creamos el agente y el entorno
    agent = DQNAgent()
    env = CarEnv()


    # Creamos y iniciamos el hilo de entrenamiento
    trainer_thread = Thread(target=agent.train_in_loop, daemon=True)
    trainer_thread.start()
    while not agent.training_initialized:
        time.sleep(0.01)

    # Inicialiciamos las predicciones
    agent.get_qs(np.ones((env.im_height, env.im_width, 3)))

    # Iteramos cada episodio
    for episode in tqdm(range(1, EPISODES + 1), ascii=True, unit='episodes'):
        # Vaciamos la variable que almacena las colisiones
        env.collision_hist = []

        # Actualizamos la tensorboard
        agent.tensorboard.step = episode

        # Reseteamos el episodio
        episode_reward = 0
        step = 1

        # Reseteamos el entorno
        current_state = env.reset()

        # Reset del flag de acabar y actualizamos el tiempo en el que empieza
        done = False
        episode_start = time.time()

        while True:

            if np.random.random() > epsilon:
                action = np.argmax(agent.get_qs(current_state))

            else:
                action = np.random.randint(0, 3)
                time.sleep(1/FPS)


            new_state, reward, done, _ = env.step(action)

            # Actualizamos los puntos conseguidos o perdidos
            episode_reward += reward

            # Actualizamos el replay memory
            agent.update_replay_memory((current_state, action, reward, new_state, done))

            current_state = new_state
            step += 1

            if done:
                break

        # Cuando acaba la prueba re destruyen todos los agentes
        for actor in env.actor_list:
            actor.destroy()

        # Guardamos en los logs y actualizamos los rewards que se han conseguido
        ep_rewards.append(episode_reward)
        if not episode % AGGREGATE_STATS_EVERY or episode == 1:
            average_reward = sum(ep_rewards[-AGGREGATE_STATS_EVERY:])/len(ep_rewards[-AGGREGATE_STATS_EVERY:])
            min_reward = min(ep_rewards[-AGGREGATE_STATS_EVERY:])
            max_reward = max(ep_rewards[-AGGREGATE_STATS_EVERY:])
            agent.tensorboard.update_stats(reward_avg=average_reward, reward_min=min_reward, reward_max=max_reward, epsilon=epsilon)

            # Se guarda el modelo solo cuando es mejor o igual al mejor que se ha guardado
            if min_reward >= MIN_REWARD:
                session = backend.get_session()
                init = tf.compat.v1.global_variables_initializer()
                session.run(init)
                agent.model.save(f'models/{MODEL_NAME}__{max_reward:_>7.2f}max_{average_reward:_>7.2f}avg_{min_reward:_>7.2f}min__{int(time.time())}.model')

        # Hacemos el decay del epsilon
        if epsilon > MIN_EPSILON:
            epsilon *= EPSILON_DECAY
            epsilon = max(MIN_EPSILON, epsilon)

    agent.terminate = True
    trainer_thread.join()
    session = backend.get_session()
    init = tf.compat.v1.global_variables_initializer()
    session.run(init)
    agent.model.save(f'models/{MODEL_NAME}__{max_reward:_>7.2f}max_{average_reward:_>7.2f}avg_{min_reward:_>7.2f}min__{int(time.time())}.model')    
    
        
        
        
        