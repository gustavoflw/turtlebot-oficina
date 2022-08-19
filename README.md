# Turtlebot para Oficina de Integração

## Pastas
- ino
    - Programas para o Arduino
- launch
    - Arquivos de launch do ROS
- src/scripts
    - Executáveis para o ROS em Python

## Tutoriais
- Iniciar o ROS master
    - ```roscore```
- Ver lista de tópicos ativos
    - ```rostopic list```
- Ver tipo de mensagem de um tópico
    - ```rostopic type [tópico]```
- Ver output de um tópico
    - ```rostopic echo [tópico]```
- Rodar programas
    - Simulador
        - ```roslaunch turtlebot_oficina simulator.launch```
    - Teleoperação
        - Requer pacote teleop_twist_keyboard (http://wiki.ros.org/teleop_twist_keyboard)
        - ```roslaunch turtlebot_oficina teleop.launch```
    - Serial
        - Requer pacote rosserial_server (http://wiki.ros.org/rosserial_server)
        - ```roslaunch turtlebot_oficina serial.launch```
    - Visualizador do kinect
        - ```roslaunch turtlebot_oficina kinect_visualizer.launch```