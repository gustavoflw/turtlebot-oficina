# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

''' Inicia o nodo '''
rospy.init_node('helloworld_publisher_node')

''' O publisher é composto de:
    - o tópico no qual ele vai publicar
    - o tipo de mensagem '''
helloworld_publisher = rospy.Publisher('/topico_helloworld', String, queue_size=10)

''' Frequência em Hz
    - Usado dentro do loop pra fazer o programa dormir '''
sleep_rate = rospy.Rate(1)

''' Loop principal
    - Publicação de mensagens
        - Feito manualmente chamando a função .publish() de cada publisher
    - Recebimento de mensagens
        - Automático com a chamada da função .sleep() '''
while not rospy.is_shutdown():
    helloworld_msg = "hello world {}".format(rospy.get_time())      # Faz uma mensagem com o tempo atual;
    print(helloworld_msg)                                           # Printa a mensagem;
    helloworld_publisher.publish(helloworld_msg)                    # Publica a mensagem;
    sleep_rate.sleep()                                              # Dorme de acordo com a frequência definida