# Turtlebot para Oficina de Integração

## Checklist
- ~~Controle proporcional motores~~
- ~~Odometria~~
- ~~Cinemática direta~~
- ~~Cinemática inversa~~
- ~~Config. básica Kinect~~
- ~~Pacote de conversão da mensagem de odometria~~
- Movimento omni direcional
- Rede do Raspberry
- Aplicativo

## Pastas
- ino
    - Programas para o Arduino
- launch
    - Arquivos de launch do ROS
- simulation
    - Arquivos da simulação (modelos, etc.)
- src
    - Programas para o ROS para rodar no PC/Raspberry

## Bibliotecas usadas no Arduino (instalar pela Arduino IDE)
- Adafruit motor shield library (não é o v2!)
- Rosserial (0.7.9)

## Tutoriais
- Depois de instalar o ROS, colar a pasta do repositório no seu workspace/src
- Comandos
    - Iniciar o ROS master
        - ```roscore```
    - Ver lista de tópicos ativos
        - ```rostopic list```
    - Ver tipo de mensagem de um tópico
        - ```rostopic type [tópico]```
    - Ver output de um tópico
        - ```rostopic echo [tópico]```
    - Rodar programas
        - Simulador (acho que não vai rodar no Windows)
            - ```roslaunch turtlebot_oficina simulator.launch```
        - Teleoperação (controlar o robô pelo teclado)
            - Requer pacote teleop_twist_keyboard (http://wiki.ros.org/teleop_twist_keyboard)
            - ```roslaunch turtlebot_oficina teleop.launch```
        - Serial
            - Requer pacote rosserial_server (http://wiki.ros.org/rosserial_server)
            - ```roslaunch turtlebot_oficina serial.launch```
        - Visualizador do kinect
            - ```roslaunch turtlebot_oficina kinect_visualizer.launch```
        - Driver de kinect
            - PENDENTE (falta kinect real)
- Programas exemplo
    - Publisher:
        - Arquivo em src/scripts/example_publisher.py
        - ```rosrun turtlebot_oficina src/example_publisher.py```
    - Subscriber:
        - Arquivo em src/scripts/example_subscriber.py
        - ```rosrun turtlebot_oficina src/example_subscriber.py```
