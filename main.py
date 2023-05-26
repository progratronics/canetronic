import RPi.GPIO as GPIO
import time
from myFunctions import *
import drivers
import math
import smbus
import threading
import numpy as np

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
#********************************************************************************
# Déclaration des variables
#********************************************************************************
threshold_pitch_min = 10
threshold_pitch_max = 40
threshold_roll_min = 20
threshold_roll_max = 70

Trig1 = 18
Echo1 = 23
Trig2 = 16
Echo2 = 20

Vibrator = 22
#********************************************************************************
# Définition des E/S
#********************************************************************************
GPIO.setup(Trig1,GPIO.OUT) # Le type d'E/S
GPIO.setup(Echo1,GPIO.IN) # Le type d'E/S
GPIO.output(Trig1, False) # Initialiser à 0 pour ne pas envoyer de signal

GPIO.setup(Trig2,GPIO.OUT) # Le type d'E/S
GPIO.setup(Echo2,GPIO.IN) # Le type d'E/S
GPIO.output(Trig2, False) # Initialiser à 0 pour ne pas envoyer de signal

GPIO.setup(Vibrator,GPIO.OUT) # Le type d'E/S

display = drivers.Lcd()

bus = smbus.SMBus(1)
addressI2C = 0x68
power_mgmt_1 = 0x6b
bus.write_byte_data(addressI2C, power_mgmt_1, 0)
#********************************************************************************
# Les fonctions utilisées
#********************************************************************************
headObstacleDist_shared_variable = None
footObstacleDist_shared_variable = None
condition_ultrasonic = threading.Condition()
def ultrasonic_measurements():
    global headObstacleDist_shared_variable
    global footObstacleDist_shared_variable
    while True:
        GPIO.output(Trig1, True)
        time.sleep(0.00001) # On envoie une onde ultrasonore pendant 10 µs
        GPIO.output(Trig1, False)
        
        while GPIO.input(Echo1)==0: # On vient d'arrêter d'envoyer l'onde ultrasonore
            debutImpulsion1 = time.time()

        while GPIO.input(Echo1)==1: # On vient de recevoir l'onde ultrasonore envoyée
            finImpulsion1 = time.time()

        headObstacleDist = round((finImpulsion1 - debutImpulsion1) * 340 * 100 / 2, 1)
        #********************************************************************************
        GPIO.output(Trig2, True)
        time.sleep(0.00001) # On envoie une onde ultrasonore pendant 10 µs
        GPIO.output(Trig2, False)
        
        while GPIO.input(Echo2)==0: # On vient d'arrêter d'envoyer l'onde ultrasonore
            debutImpulsion2 = time.time()

        while GPIO.input(Echo2)==1: # On vient de recevoir l'onde ultrasonore envoyée
            finImpulsion2 = time.time()

        footObstacleDist = round((finImpulsion2 - debutImpulsion2) * 340 * 100 / 2, 1) 

        with condition_ultrasonic:
            # Mettre à jour la variable partagée
            headObstacleDist_shared_variable = headObstacleDist
            footObstacleDist_shared_variable = footObstacleDist

            # Notifier tous les threads en attente que la valeur a été mise à jour
            condition_ultrasonic.notify_all()

        time.sleep(0.2)
#********************************************************************************
def obstacle_detection():
    while True:
        with condition_ultrasonic:
            # Attendre jusqu'à ce que la valeur soit mise à jour par le premier thread
            condition_ultrasonic.wait()

            # Récupérer la dernière valeur mise à jour
            headObstacleDist = headObstacleDist_shared_variable
            footObstacleDist = footObstacleDist_shared_variable

        time.sleep(3.0)
#********************************************************************************
acc_x_shared_variable = None
acc_y_shared_variable = None
acc_z_shared_variable = None
gyro_x_shared_variable = None
gyro_y_shared_variable = None
gyro_z_shared_variable = None
condition_mpu6050 = threading.Condition()
def mpu6050_measurements():
    global acc_x_shared_variable
    global acc_y_shared_variable
    global acc_z_shared_variable
    global gyro_x_shared_variable
    global gyro_y_shared_variable
    global gyro_z_shared_variable
    while True:
        acc_x = read_raw_data(bus, addressI2C, 0x3b)
        acc_y = read_raw_data(bus, addressI2C, 0x3d)
        acc_z = read_raw_data(bus, addressI2C, 0x3f)
        gyro_x = read_raw_data(bus, addressI2C, 0x43)
        gyro_y = read_raw_data(bus, addressI2C, 0x45)
        gyro_z = read_raw_data(bus, addressI2C, 0x47)

        with condition_mpu6050:
            acc_x_shared_variable = acc_x
            acc_y_shared_variable = acc_y
            acc_z_shared_variable = acc_z
            gyro_x_shared_variable = gyro_x
            gyro_y_shared_variable = gyro_y
            gyro_z_shared_variable = gyro_z
            condition_mpu6050.notify_all()

        time.sleep(0.1)
#********************************************************************************     
def pitchnroll_computing():
    while True:
        with condition_mpu6050:
            condition_mpu6050.wait()
            acc_x = acc_x_shared_variable
            acc_y = acc_y_shared_variable
            acc_z = acc_z_shared_variable
            gyro_x = gyro_x_shared_variable
            gyro_y = gyro_y_shared_variable 
            gyro_z = gyro_z_shared_variable

        roll = math.atan2(acc_y, acc_z) * 180 / math.pi
        pitch = math.atan(-acc_x / math.sqrt(acc_y * acc_y + acc_z * acc_z)) * 180 / math.pi
        gyro_roll = gyro_x / 131
        gyro_pitch = gyro_y / 131
        alpha = 0.98
        roll = round(alpha * (roll + gyro_roll * 0.01) + (1 - alpha) * roll)
        pitch = round(alpha * (pitch + gyro_pitch * 0.01) + (1 - alpha) * pitch)

        if threshold_pitch_min < pitch < threshold_pitch_max and threshold_roll_min < roll < threshold_roll_max:
            pass
        else:
            print("Tenez bien votre canne!")
            
        time.sleep(0.2)
#********************************************************************************
def distance_traveled_computing():
    delta_t = 0.1
    dist_z = 0.0
    vel_z = 0
    acceleration_z_old_value = 0
    difference = 0
    while True:
        with condition_mpu6050:
            condition_mpu6050.wait()
            acc_z = acc_z_shared_variable

        sensitivity_scale = 2  # Échelle de sensibilité +/- 2g pour l'accéléromètre
        acceleration_scale = sensitivity_scale / 32768.0  # Échelle de conversion en g

        # Conversion en m/s²
        acceleration_x = round(acc_z * acceleration_scale * 9.8)
        
        difference = abs(acceleration_z - acceleration_z_old_value)

        if difference > 1:
            dist_z += abs(round(vel_z * delta_t + 0.5 * acceleration_z * delta_t ** 2,2))
            vel_z += acceleration_z * delta_t

        acceleration_z_old_value = acceleration_z

        time.sleep(0.1)
#********************************************************************************
# Définition des threads
#********************************************************************************
thread1 = threading.Thread(target=ultrasonic_measurements)
thread2 = threading.Thread(target=obstacle_detection)

thread3 = threading.Thread(target=mpu6050_measurements)
thread4 = threading.Thread(target=pitchnroll_computing)
thread5 = threading.Thread(target=distance_traveled_computing)
#********************************************************************************
# Démarrage des threads
#********************************************************************************
thread1.start()
thread2.start()

thread3.start()
thread4.start()
thread5.start()
#********************************************************************************
# Attente des threads
#********************************************************************************
thread1.join()
thread2.join()

thread3.join()
thread4.join()
thread5.join()
#********************************************************************************
GPIO.cleanup() # Nettoyer les tensions sur les pins pour une nouvelle utilisation