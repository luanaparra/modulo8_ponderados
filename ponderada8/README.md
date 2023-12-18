# Ponderada 8

## Descrição 
Criação de uma aplicação que integra um tradutor de texto (pode ser baseado em LLM ou não), uma aplicação de speech-to-text e uma aplicação de text-to-speech.

Assim, a ferramenta de terminal deve ser capaz de receber como argumento o caminho para um arquivo de áudio contendo a fala de uma pessoa. Após ler o arquivo fornecido, a aplicação deve ser capaz de transformar o áudio em texto corretamente, usar esse texto gerado para alimentar um tradutor de texto (e.g. transforma um texto do português para o inglês) e, por fim, transforma o texto traduzido em áudio novamente e reproduz para o usuário.

## Execução

Em primeiro lugar, é necessário instalar todas as bibliotecas e dependências.

```
pip install googletrans SpeechRecognition gtts playsound
```

Em seguida, é preciso rodar:

```
python nome_do_arquivo.py caminho_para_o_arquivo_de_audio
```
Nesse caso, o 'caminho_para_o_arquivo_de_audio' é onde você quer guardar o arquivo do audio. 

## Vídeo
[link](https://github.com/luanaparra/modulo8_ponderados/blob/main/ponderada8/ponderada8.mp4)
