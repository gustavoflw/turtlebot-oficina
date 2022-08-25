# Turtlebot para Oficina de Integração

## Pastas
- ino
    - Programas para o Arduino
- launch
    - Arquivos de launch do ROS
- simulation
    - Arquivos da simulação (modelos, etc.)
- src/scripts
    - Executáveis para o ROS em Python

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
    
