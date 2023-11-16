# Ponderada 3

## Descrição da atividade
Desenvolvimento de um nó de ROS que seja um chatbot capaz de entender comandos escritos em linguagem natural para interagir com um robô de serviço fictício. O chatbot deve fornecer ao usuário a possibilidade de enviar comandos de posição para o robô de forma simples e intuitiva.

Para cada comando registrado, o sistema deve ser capaz de extrair a intenção do usuário a partir de um dicionário de intenções, filtradas por expressões regulares. A partir daí, um segundo dicionário deve ser usado, capaz de vincular intenções à funções que o robô deve executar. Para essa ponderada, o script em Python não precisa se comunicar com o nav2 e nem com o robô, mas é necessário dar um feedback ao usuário de que a ação foi compreendida e está sendo executada. Por fim, está liberado o uso de frameworks como Chatterbot e NLTK.

## Explicação
As funcionalidades do pacote criado são: 
- Pedir para ir até determinado local
- Pedir para explicar o que ele faz

## Como rodar
1. Construção do workspace
```
cd chatbot
colcon build
source install/local_setup.bash
```
2. Executar o chat
```
ros2 run simple_chatbot chatbot_node.py
```

## Video do funcionamento 
[link](https://github.com/luanaparra/modulo8_ponderados/blob/main/ponderada3/video-chatbot.mp4)


