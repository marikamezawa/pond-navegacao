# README – Ponderada de Navegação Autônoma

Esta ponderada consiste em desenvolver um sistema de navegação autônoma para um robô em um labirinto, utilizando **ROS 2**, algoritmos de grafos e o simulador fornecido pelo professor. O projeto é dividido em duas partes:

### **Parte 1 – Navegação com Mapa**

O robô recebe um mapa completo via serviço ROS, processa esse mapa, executa o algoritmo **BFS** para encontrar o menor caminho até o alvo e envia comandos de movimento ao simulador.

### **Parte 2 – Mapeamento Autônomo**

O robô deverá navegar sem mapa pronto, explorando o labirinto e construindo seu próprio mapa. Ao final, deve-se comprovar que o mapa gerado é suficiente para reproduzir a rota da parte 1.



## Pacotes e Recursos ROS Utilizados

### **Serviços**

* `/get_map` – Fornece o mapa completo do labirinto.
* `/move_command` – Move o robô um passo para cima, baixo, esquerda ou direita.

### **Nós usados do simulador**

* `cg maze` – Abre a simulação do labirinto (pygame).
* `cg_teleop` – Permite controle manual, usado apenas para testes.

### **Nós criados pelo projeto**

* `map_loader` – Faz a requisição do mapa ao serviço `/get_map`.
* `pathfinder` – Implementa o algoritmo BFS e produz a lista de coordenadas do menor caminho.
* `main_node` – Carrega o mapa, encontra o caminho e controla o robô até o alvo.


## Estrutura do Projeto

```
minha_ponderada/
 ├── include/minha_ponderada/
 │    ├── map_loader.hpp
 │    ├── pathfinder.hpp
 │    └── robot_mover.hpp
 ├── src/
 │    ├── map_loader.cpp
 │    ├── pathfinder.cpp
 │    ├── robot_mover.cpp
 │    └── main.cpp
 ├── package.xml
 ├── CMakeLists.txt
```


## Como Instalar e Executar

### **1. Clone o repositório do professor**

```bash
git clone https://github.com/rmnicola/culling_games.git
```

### **2. Coloque sua pasta dentro do workspace**

Depois de clonar, insira a pasta **`minha_ponderada/`** dentro de:

```
culling_games/src/
```

Estrutura final:

```
culling_games/
 └── src/
      ├── cg
      ├── cg_interfaces
      ├── cg_teleop
      ├── minha_ponderada   ← seu pacote aqui
```


### **3. Criar um ambiente virtual usando Micromamba**

No diretório `culling_games/`:

```bash
micromamba activate ros_env
```

### **4. Compile o workspace**

```bash
colcon build
source install/setup.bash
```

### **5. Execute o simulador em um terminal separado**

```bash
ros2 run cg maze
```

### **6. Em outro terminal, execute sua ponderada**

```bash
source install/setup.bash
ros2 run minha_ponderada main_node
```

O robô irá:

1. Carregar o mapa via serviço.
2. Calcular o menor caminho usando BFS.
3. Movimentar-se célula por célula até chegar ao alvo.


## Observações importantes

* A simulação deve estar sempre rodando antes do seu nó.
* É necessário executar **dois terminais** simultâneos: simulador e projeto.
* A posição do robô e do alvo depende dos dados enviados pelo serviço `/get_map`.
* Você deve ter o ROS2 instalado para executar esse projeto [link](https://robostack.github.io/GettingStarted.html)


## Vídeo
[link](https://drive.google.com/file/d/1CxrmauQ6KiyP0syVOH31GkqaahWwwYTi/view?usp=sharing)