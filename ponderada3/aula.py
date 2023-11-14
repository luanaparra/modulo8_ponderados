#! /bin/env python3

import re

def vai_para_ponto(x):
    print(f"Vou te levar até o ponto {x}.")

pontos = {
    "x": (0.0, 0.0, 1.0)
}

intencoes = {
    r".?\bponto\s?([a-z])": "destino",
}

acoes = {
    "destino": vai_para_ponto,
}

comando = input("Digite seu comando: ")

for chave, valor in intencoes.items():
    padrao = re.compile(chave, re.IGNORECASE)
    grupo_captura = padrao.findall(comando) # Verifica se há match com o padrão
    if grupo_captura:
        print(f"Detectei a intenção: {valor}.")
        acoes[valor](grupo_captura[0])