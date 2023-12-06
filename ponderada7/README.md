# Ponderada 7

## Descrição 
Treinar e utilizar uma rede neural convolucional para classificar corretamente o dataset MNIST.

1. Carregamento e Pré-processamento dos Dados:
   - Carregamento do conjunto de dados MNIST e normalização das imagens.
   
2. Definição da Arquitetura do Modelo:
   - Utilização uma rede neural com camadas densas para classificação.
   - Configuração da arquitetura com diferentes camadas e ativações.
   
3. Compilação e Treinamento do Modelo:
   - Compilação do modelo usando o otimizador 'adam' e a perda 'sparse_categorical_crossentropy'.
   - Treinamento do modelo com 3 épocas usando os dados de treino.

4. Salvamento e Carregamento do Modelo:
   - Salvar o modelo treinado em 'num_classifier.model'.
   - Carregar o modelo salvo usando 'tf.keras.models.load_model'.
   
5. Avaliação do Modelo nos Dados de Teste:
   - Avaliação do modelo carregado nos dados de teste para verificar seu desempenho.

6. Resultados:
   - Exibição da perda e acurácia do modelo nos dados de teste.

Observações:
- É possível ajustar a arquitetura, como o número de neurônios ou camadas, para explorar diferentes configurações.
- Os hiperparâmetros, como épocas de treinamento, também podem ser ajustados para otimizar o desempenho.

**Instruções de Execução:**
1. Certifique-se de ter o Python e TensorFlow instalados no seu ambiente.
2. Copie e cole este código em um arquivo Python (por exemplo, 'mnist_classifier.py').
3. Execute o arquivo Python em um ambiente com acesso à internet para baixar o conjunto de dados MNIST.
4. O código treinará um modelo de classificação de dígitos usando a rede neural convolucional.
5. Após o treinamento, o modelo será salvo como 'num_classifier.model'.
6. Para carregar o modelo e avaliá-lo novamente nos dados de teste, execute o arquivo Python.

Observações:
- Pode ser necessário instalar as bibliotecas necessárias usando 'pip install tensorflow'.
- Certifique-se de ter permissões para gravar arquivos no diretório onde o código está sendo executado para salvar o modelo.
