cmake_minimum_required(VERSION 3.5)
project(simple_chatbot)

# Em ROS2, é comum encontrar essas linhas incluídas para
# garantir a compatibilidade com os recursos do ROS.
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Adiciona a subpasta 'scripts' como um executável ROS2
# Esta é a pasta onde seu arquivo Python do chatbot está localizado.
ament_python_install_package(${\Users\Inteli\Documents\GitHub\modulo8_ponderados\ponderada3\chatbot-teste-aula\chatbot\scripts\chatbot_node.py})

# Indica que o pacote será uma biblioteca Python.
# Esta linha é essencial se o seu nó ROS2 for escrito em Python.
ament_export_dependencies(ament_cmake)
ament_package()
