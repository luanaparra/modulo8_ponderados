# Ponderada 2

## Descrição da atividade
Desenvolvimento de um pacote em ROS com as funcionalidades de mapeamento e navegação utilizando o turtlebot 3, no caso a ponderada foi testada com o robô real. 

## Explicação
As funcionalidades do pacote criado são: 
- Realização do mapeamento, utilizando nós e lançadores já existentes: Teleop, Rviz e Gazebo.
- Para a navegação, existem quatro nós que são responsáveis pela movimentação do robô: inicialização da pose no nav2, envio de pontos para navegação, implementação de uma fila para gerenciar a ordem dos pontos a serem percorridos e simple commander para controlar o robô.

## Como rodar
1. Clone este repositório:
```
https://github.com/luanaparra/modulo8_ponderados/ponderada2
```
2. Construção do workspace
```
cd ws
colcon build
source install/local_setup.bash
```
3. Executar o lançador para mapeamento
```
ros2 launch map map_launch.py
```
4. Após o mapeamento, salvar o mapa em outro terminal
```
ros2 run nav2_map_server map_saver_cli -f caminho/para/salvar/mapa
```
5. Adicionar o mapa gerado na pasta do lançador com os nomes map.yaml e map.pgm
6. Executar o lançador para navegação
```
ros2 launch navigation navigator_launch.py
```

## Video do funcionamento 
[link](https://github.com/luanaparra/modulo8_ponderados/blob/main/ponderada2/video-robo.mov)


