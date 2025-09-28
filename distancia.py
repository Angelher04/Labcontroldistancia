#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
import math

# Inicializar ladrillo y motores
ev3 = EV3Brick()
MA = Motor(Port.A)
MB = Motor(Port.B)

# Diámetro de rueda (m) → cámbialo si usas otra
DIAMETRO_RUEDA = 0.056   # 56 mm (rueda EV3 estándar)
CIRCUNFERENCIA = DIAMETRO_RUEDA * math.pi

# Distancia objetivo
DISTANCIA = 3.0  # metros
GRADOS_OBJETIVO = (DISTANCIA / CIRCUNFERENCIA) * 360

# PID parámetros
Kp = 2.0
Ki = 0.0
Kd = 0.5

dt = 10  # periodo de muestreo en ms

# Variables PID
integral = 0
last_error = 0

# Resetear encoders
MA.reset_angle(0)
MB.reset_angle(0)

ev3.speaker.beep()
print("Iniciando PID para recorrer 3 m...")

while True:
    # 1. Medir posición actual (promedio de ambos motores)
    pos_actual = (MA.angle() + MB.angle()) / 2
    error = GRADOS_OBJETIVO - pos_actual

    # 2. Verificar si ya llegó
    if abs(error) < 10:  # margen de 10 grados ≈ 1.5 mm
        break

    # 3. PID
    integral += error * (dt/1000)
    derivative = (error - last_error) / (dt/1000)
    output = Kp*error + Ki*integral + Kd*derivative

    # 4. Limitar salida
    output = max(min(output, 720), -720)  # máx ±720 °/s

    # 5. Mandar velocidad a los motores
    MA.run(output)
    MB.run(output)

    # 6. Guardar error y esperar
    last_error = error
    wait(dt)

# Frenar
MA.stop(Stop.BRAKE)
MB.stop(Stop.BRAKE)
ev3.speaker.beep()
print("Recorrido completado (3 m).")
