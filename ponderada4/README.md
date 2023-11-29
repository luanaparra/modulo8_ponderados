# Ponderada 4

## Descrição 
Utilizando um LLM (local ou API externa), crie um chatbot simples com instruções customizadas para ajudar um usuário a pesquisar normas de segurança em ambientes industriais. O sisema deve contar com uma interface gráfica e responder de forma sucinta e clara sobre o que lhe foi perguntado.

Exemplo de prompt:

>>> Quais EPIs são necessários para operar um torno mecânico?

## Explicação
O primeiro código é um exemplo de código inicial para um chatbot que recebe perguntas do usuário via terminal e publica essas perguntas em um tópico chamado "/llm". O segundo código é a parte central do sistema, onde um nó ROS 2 chamado "llm_node" recebe as mensagens, processa-as utilizando um modelo de linguagem (especificamente um modelo da biblioteca langchain), e publica as respostas em um tópico chamado "/chatbot". Por fim, o terceiro código é responsável pela inicialização do nó "llm_node" e pelo gerenciamento do ciclo de vida do ROS.

## Vídeo
[link](https://github.com/luanaparra/modulo8_ponderados/blob/main/ponderada4/ponderada4.mp4)
