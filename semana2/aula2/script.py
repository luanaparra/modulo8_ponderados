import csv
import yaml

def calcular_media(notas):
    return sum(notas) / len(notas)

def ler_notas_csv(arquivo_csv):
    notas = []
    with open(arquivo_csv, 'r') as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader)
        for row in csv_reader:
            aluno = row[0]
            notas_aluno = [float(nota) for nota in row[1:]]
            media = calcular_media(notas_aluno)
            notas.append({'aluno': aluno, 'notas': notas_aluno, 'media': media})
    return notas

def ler_notas_yaml(arquivo_yaml):
    with open(arquivo_yaml, 'r') as file:
        data = yaml.safe_load(file)
        return data

def escrever_notas_csv(notas, arquivo_csv):
    with open(arquivo_csv, 'w', newline='') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(['Aluno', 'Notas', 'Média'])
        for aluno in notas:
            csv_writer.writerow([aluno['aluno'], ', '.join(map(str, aluno['notas'])), aluno['media']])

def escrever_notas_yaml(notas, arquivo_yaml):
    with open(arquivo_yaml, 'w') as file:
        yaml.dump(notas, file)

arquivo_entrada = 'notas.csv'
formato_arquivo_entrada = arquivo_entrada.split('.')[-1]
arquivo_saida = 'notas_saida.csv'

if formato_arquivo_entrada == 'csv':
    notas = ler_notas_csv(arquivo_entrada)
elif formato_arquivo_entrada == 'yml':
    notas = ler_notas_yaml(arquivo_entrada)
else:
    print("Formato de arquivo de entrada não suportado.")
    notas = []

for aluno in notas:
    aluno['media'] = calcular_media(aluno['notas'])

if arquivo_saida == 'stdout':
    for aluno in notas:
        print(f"Aluno: {aluno['aluno']}, Notas: {aluno['notas']}, Média: {aluno['media']:.2f}")
else:
    formato_arquivo_saida = arquivo_saida.split('.')[-1]
    if formato_arquivo_saida == 'csv':
        escrever_notas_csv(notas, arquivo_saida)
    elif formato_arquivo_saida == 'yml':
        escrever_notas_yaml(notas, arquivo_saida)
    else:
        print("Formato de arquivo de saída não suportado.")