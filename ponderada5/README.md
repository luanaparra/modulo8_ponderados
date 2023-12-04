# Ponderada 5

## Descrição 
Melhoria do chatbot desenvolvido na ponderada anterior, agora utilizando técnicas de leitura de documentos, nesse caso é um pdf.

## Explicação
Na ponderada 4, o primeiro código é um exemplo de código inicial para um chatbot que recebe perguntas do usuário via terminal e publica essas perguntas em um tópico chamado "/llm". O segundo código é a parte central do sistema, onde um nó ROS 2 chamado "llm_node" recebe as mensagens, processa-as utilizando um modelo de linguagem (especificamente um modelo da biblioteca langchain), e publica as respostas em um tópico chamado "/chatbot". Por fim, o terceiro código é responsável pela inicialização do nó "llm_node" e pelo gerenciamento do ciclo de vida do ROS.

Posto isso, a ponderada 5 teve uma nova maneira de implementação do chat, além da interface.

**Importações**: 
- O código começa importando as bibliotecas necessárias, incluindo módulos do ROS (rclpy, Node, Float32MultiArray), ferramentas para processamento de texto e IA (gradio, dotenv, etc.).

**Classe LLMNode**:
- Inicialização: O construtor __init__ define o nó llm_node; cria um publicador para enviar mensagens do tipo Float32MultiArray para o tópico /waypoints; inicia o carregamento do modelo de chatbot, carrega dados de um PDF, configura o processamento de texto, etc.
- Métodos: load(): Carrega o modelo de chatbot, define o padrão de mensagem e configura o processamento de texto; publish_command(): Publica os pontos (coordenadas) em um tópico ROS com base em uma mensagem recebida do chatbot; run(): Define uma interface de chat usando o gradio, permitindo interação com o chatbot; Função main() inicializa o nó LLMNode e mantém o programa em execução usando rclpy.spin().

**Funcionamento Geral**:
- O nó LLMNode inicia o chatbot e estabelece uma interface de interação com ele por meio do gradio. Quando uma mensagem é enviada para o chatbot, ele a processa usando o modelo de linguagem carregado e gera uma resposta. Em seguida, essa resposta é verificada por padrões (como coordenadas) e, se correspondido, publica essas coordenadas em um tópico ROS específico.

## Vídeo
[link](https://github.com/luanaparra/modulo8_ponderados/blob/main/ponderada5/ponderada5.webm)
