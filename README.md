# Control de Roomba 650

Programa en Python para controlar un Roomba 650 por su interfaz serie Open Interface (OI). El puerto predeterminado es `COM8` a `115200` baudios.

## Requisitos

- Python 3.10 o posterior.
- Un adaptador compatible con el puerto OI del Roomba. La interfaz usa señales serie TTL de 0–5 V; no conectes un adaptador RS-232 convencional.
- Paquetes de Python indicados en `requirements.txt`.

## Instalación en Windows

Desde PowerShell, en la carpeta del proyecto:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

Si PowerShell bloquea la activación, puedes ejecutar los programas directamente:

```powershell
py main.py
py odometria.py
```

Para elegir otro puerto o baudios:

```powershell
py main.py --port COM8 --baud 115200
py odometria.py --port COM8 --baud 115200
```

Cierra un programa antes de abrir el otro: ambos usan el mismo puerto serie.

## Control por terminal

Ejecuta `py main.py`. Enciende el Roomba con CLEAN, colócalo sobre suelo plano y despejado, y usa:

- `w`: avanzar; `s`: retroceder.
- `a` y `d`: girar a izquierda o derecha.
- `x`: parar; `b`: consultar sensores; `q`: salir.

Los movimientos duran entre 0,1 y 10 segundos y se detienen al terminar ese intervalo.

## Mapa, odometría y rutas

Ejecuta `py odometria.py`. En la ventana del mapa, las flechas teleoperan el Roomba; espacio o `x` paran; `r` reinicia las coordenadas; `i` muestra los acumulados de odometría; `q` cierra la ventana.

En el cuadro **Ruta**, escribe tramos separados por comas y pulsa **Ejecutar**. Por ejemplo:

```text
F100,R90,F150
```

- `F100`: avanzar 100 cm; `B20`: retroceder 20 cm.
- `L90`: girar 90° a la izquierda; `R90`: girar 90° a la derecha.
- Cada tramo recto está limitado a 500 cm y cada giro a 360°.
- **Volver** invierte el orden y sentido de los tramos recorridos. El regreso es aproximado porque la odometría acumula error por deslizamiento y diferencias entre ruedas.
- **Parar** o la barra espaciadora detienen el movimiento. La ruta también se detiene al detectar contacto con el parachoques o una condición de seguridad.

## Límites de detección

Los sensores originales del Roomba 650 no miden la distancia frontal a los obstáculos. El parachoques detecta contacto; por ello, la ruta puede detenerse al tocar un obstáculo, pero no garantiza evitar ese primer contacto ni detectar objetos a 10 cm.

## Archivos

- `main.py`: control por terminal y comunicación serie.
- `odometria.py`: mapa, odometría, control manual y rutas.
- `requirements.txt`: dependencias instalables con pip.
- `.gitignore`: excluye el entorno virtual y archivos generados localmente.
