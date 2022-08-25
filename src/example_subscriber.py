# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

''' Função de callback 
    - Chamada toda vez que há uma nova mensagem 
    - Ver a definição do subscriber abaixo '''
def helloworld_callback(msg):
    print(msg.data)

''' Inicia o nodo '''
rospy.init_node('helloworld_subscriber_node')

''' O subscriber é composto de:
    - o tópico do qual vai receber
    - o tipo de mensagem
    - a função que será chamada toda vez que receber mensagem '''
rospy.Subscriber("/topico_helloworld", String, helloworld_callback)    

''' Frequência em Hz
    - Usado dentro do loop pra fazer o programa dormir '''
sleep_rate = rospy.Rate(30)

''' Loop principal
    - Publicação de mensagens
        - Feito manualmente chamando a função .publish() de cada publisher
    - Recebimento de mensagens
        - Automático com a chamada da função .sleep() '''
while not rospy.is_shutdown():
    sleep_rate.sleep()              # Dorme de acordo com a frequência definida
                                    # (de forma oculta chama a função de callback!);