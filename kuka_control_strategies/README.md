# `kuka_control_strategies`

**Objetivo 1 — Comparação de estratégias de controle de manipuladores: local (PD por junta), centralizado (torque computado) e espaço operacional (task-space).**

> **Estado desta versão:** inclui a correção do parâmetro `custom_positions` não declarado
> em `operational_space.py` e a ativação efetiva de `kp_pos`/`kd_pos`. Detalhes na seção 5.5.

Este pacote implementa três arquiteturas clássicas de controle e um nó de *benchmark* que as
compara quantitativamente.

---

## 1. Ponto crítico de arquitetura — leia antes de tudo

Os três "controladores" deste pacote **não fecham a malha de torque diretamente**. O
`ros2_control` do workspace base expõe um `JointTrajectoryController` (JTC) por
`FollowJointTrajectory`, e é ele quem realmente aciona as juntas.

O que cada nó faz, na prática:

1. **Calcula** a lei de controle (PD, torque computado ou task-space) em paralelo, a 50 Hz;
2. **Publica** essa lei de controle e o erro em tópicos de diagnóstico;
3. **Envia** um goal de trajetória com a **posição-alvo** ao JTC, que executa o movimento.

Ou seja: as leis de controle são **observadas e comparadas**, não aplicadas. É um arranjo
didático — o movimento físico é sempre o mesmo (JTC), enquanto os nós instrumentam e comparam
as métricas que *cada estratégia produziria*. Isso deve ficar explícito em qualquer relatório
baseado neste pacote.

Consequência operacional: **rode um controlador por vez**. Três nós disputando o mesmo action
server geram goals concorrentes — o JTC aborta o goal anterior a cada novo. O README do
workspace alerta para isso, embora o launch file suba os três juntos.

---

## 2. Visão geral

| Item | Valor |
|---|---|
| Build type | `ament_python` |
| Versão | 0.1.0 |
| Dependências | `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `trajectory_msgs`, `visualization_msgs`, `tf2_ros`, `tf2_geometry_msgs` |
| Testes | `ament_copyright`, `ament_flake8`, `ament_pep257` |

### Executáveis

| Executável | Estratégia |
|---|---|
| `local_pd_controller` | Controle **local** — um PD independente por junta |
| `computed_torque` | Controle **centralizado** — modelo dinâmico completo |
| `operational_space` | Controle no **espaço operacional** — via Jacobiano |
| `strategy_benchmark` | Coleta as métricas dos três e emite relatório comparativo |

### Alvos de demonstração (compartilhados)

```python
DEMO_TARGETS = {
    'home':    [0.0,   0.0,   0.0,  0.0,  0.0,  0.0],
    'pick':    [0.0,  -1.0,   1.2,  0.0,  0.8,  0.0],
    'place':   [1.57, -1.0,   1.0,  0.0,  0.5,  0.0],
    'stretch': [0.0,  -1.57,  1.57, 0.0,  1.57, 0.0],
}
```

---

## 3. `local_pd_controller.py` — controle local (descentralizado)

### 3.1 Conceito

A estratégia mais simples: **cada junta é tratada como um sistema SISO independente**. Não há
modelo dinâmico, não há acoplamento entre eixos. O acoplamento inercial e os efeitos de
Coriolis são tratados como *perturbação* que o PD deve rejeitar.

```
u_i = Kp_i · (q_des_i − q_i) + Kd_i · (q̇_des_i − q̇_i)
```

Cada junta tem seu próprio par de ganhos — daí os vetores em vez de escalares.

### 3.2 Parâmetros

| Parâmetro | Padrão |
|---|---|
| `kp` | `[200, 200, 150, 100, 80, 50]` |
| `kd` | `[20, 20, 15, 10, 8, 5]` |
| `target` | `pick` |
| `custom_positions` | `[0,0,0,0,0,0]` |

Os ganhos decrescem do eixo 1 ao 6 porque a inércia refletida também decresce: a junta 1
carrega todo o braço (≈175 kg de elo), enquanto a junta 6 move só o punho (≈3,6 kg).

Se `target == 'custom'`, usa `custom_positions`; senão busca em `DEMO_TARGETS`, com fallback
para `home`.

### 3.3 Operação

| Interface | Nome | Tipo |
|---|---|---|
| Assina | `/joint_states` | `sensor_msgs/JointState` |
| Publica | `/kuka_control_strategies/pd_error` | `Float64MultiArray` (erro das 6 juntas, 50 Hz) |
| Action | `/kuka_arm_controller/follow_joint_trajectory` | `FollowJointTrajectory` |

Dois timers:

- **2,0 s → `_send_goal`** — tenta enviar o goal. Protegido pela flag `goal_sent`: se o action
  server ainda não estiver disponível, apenas loga aviso e tenta de novo no próximo ciclo.
  O goal é um único ponto, `time_from_start = 4 s`.
- **0,02 s (50 Hz) → `_publish_error`** — calcula `e = q_des − q`, acumula a norma em
  `errors_history` e publica o vetor de erro.

Ao concluir o movimento, `_result_cb` loga o erro final e o **RMS acumulado**, e libera
`goal_sent = False` — permitindo um novo goal (o timer de 2 s reenviará).

O `Ctrl+C` imprime o RMS final.

> **Observação:** `errors_history` cresce indefinidamente a 50 Hz (≈180 mil pontos/hora).
> Para sessões longas seria conveniente limitar com um `deque`.

---

## 4. `computed_torque.py` — controle centralizado (model-based)

### 4.1 Conceito

Lei de controle por **linearização por realimentação**, que cancela a dinâmica não linear do
manipulador:

```
u = M(q)·(q̈_des + Kd·ė + Kp·e) + C(q,q̇)·q̇ + g(q)
```

Se o modelo `M`, `C`, `g` fosse exato, a malha fechada viraria seis integradores duplos
desacoplados — e o PD externo teria desempenho uniforme em toda a área de trabalho, ao
contrário do controle local.

### 4.2 O que está de fato implementado

Aqui é preciso ser exato sobre o que o código faz:

- **`g(q)` — implementado.** Via `compute_gravity_vector`, por trabalho virtual e diferenças
  finitas (`dq = 1e-6`), somando `m_j · G · ∂z_CoM_j/∂q_i` para `j ≥ i`.
- **`M(q)` — não implementado.** Não há cálculo da matriz de inércia.
- **`C(q,q̇)` — não implementado.** Não há termos de Coriolis/centrífugos.
- **A linha `tau = g`** deixa isso explícito no código, com o comentário
  *"simplificado: usa só compensação de gravidade para direção"*. A variável `v` (o termo PD)
  é calculada mas **não entra em `tau`**.

Portanto o que este nó publica em `/kuka_control_strategies/computed_torque` é o **vetor
gravitacional**, não o torque computado completo.

### 4.3 Modelo próprio (independente do pacote de FK)

Diferente dos outros pacotes, este arquivo define **sua própria cinemática por DH**:

```python
DH_PARAMS = [  # [a, alpha, d, theta_offset]
    [0.175,   π,     0.236,   0.0],
    [0.890,   0.0,  -0.285,   0.0],
    [0.30805, π/2,  -0.0975,  0.0],
    [0.0,    -π/2,   0.72695, 0.0],
    [0.149,   π/2,   0.04795, 0.0],
    [0.0,     0.0,   0.036,   0.0],
]
LINK_MASSES = [150.0, 80.0, 45.0, 20.0, 10.0, 5.0]   # kg (arredondadas)
LINK_COM    = [[0,0,0.25],[0.65,0,0],[0,0,0.1],[0,0,0.5],[0,0,0],[0,0,0.1]]
```

**Este é justamente o modelo DH que o README do workspace declara ter sido abandonado** por
perder informação de orientação. As massas também diferem das do URDF usadas em
`kinematics.py` (175.205 vs 150.0 etc.). Ponto a favor: aqui há um `LINK_COM` explícito por
elo, em vez de assumir o CoM na origem do frame.

Consequência prática: **os valores de `g(q)` deste pacote não são comparáveis com os de
`kuka_forward_kinematics.kinematics.gravity_vector`** — são dois modelos distintos.

`dh_matrix(a, alpha, d, theta)` monta a transformação DH padrão (convenção de Denavit-Hartenberg
distal).

### 4.4 Parâmetros e interfaces

| Parâmetro | Padrão |
|---|---|
| `kp` | `[150, 150, 120, 80, 60, 40]` (vira `np.diag`) |
| `kd` | `[25, 25, 20, 15, 10, 8]` (vira `np.diag`) |
| `target` | `pick` |
| `custom_positions` | `[0,0,0,0,0,0]` |

| Publica | Conteúdo |
|---|---|
| `/kuka_control_strategies/computed_torque` | `tau` (= `g(q)`), 50 Hz |
| `/kuka_control_strategies/ct_error` | Erro das 6 juntas, 50 Hz |

Ganhos convertidos em matrizes diagonais para permitir as multiplicações `self.kp @ e`.
Estrutura de timers e callbacks idêntica à do `local_pd_controller`.

No log de envio do goal aparece `||g(q)||` em Nm — é a grandeza mais informativa que o nó
produz, mostrando quanto torque gravitacional o robô precisa vencer em cada postura.

> **Nota:** ao contrário do PD local, este nó **não** trata `target == 'custom'` — o parâmetro
> é declarado mas nunca consultado; `q_des` sai sempre de `DEMO_TARGETS`.

---

## 5. `operational_space.py` — controle no espaço operacional

### 5.1 Conceito

Em vez de especificar ângulos de junta, especifica-se a **posição cartesiana do end-effector**.
A lei transforma erro cartesiano em torque de junta via Jacobiano transposto:

```
F   = Kp_pos · (p_des − p_atual)      # força virtual no TCP
τ   = Jᵀ · F + g(q)
```

O `Jᵀ` é a dualidade estática força↔torque: uma força aplicada no TCP corresponde a esse
vetor de torques nas juntas. É a base do controle por impedância.

### 5.2 Como o alvo é definido

O nó mantém **dois dicionários paralelos**:

```python
CARTESIAN_TARGETS = {  # posição do TCP (m)
    'home':    [1.522, 0.763, 0.5705],
    'pick':    [1.213, 1.294, 0.5705],
    'stretch': [0.520, 1.468, 0.5705],
    'center':  [1.213, 1.294, 0.5705],   # = pick
    'left':    [1.213, 1.294, 0.5705],   # = pick
}
JOINT_SEEDS = {  # configuração de juntas correspondente
    'home':    [0, 0, 0, 0, 0, 0],
    'pick':    [0, -1.0, 1.2, 0, 0.8, 0],
    ...
}
```

`CARTESIAN_TARGETS` é usado apenas para **medir o erro**; o goal enviado ao robô usa
`JOINT_SEEDS`. Ou seja, o nó não resolve IK — ele já traz a resposta pré-calculada e usa o
alvo cartesiano só como referência de métrica.

> **Importante:** esses alvos cartesianos foram calculados com o **modelo DH** importado de
> `computed_torque.py`, e por isso **divergem das poses do README do workspace**, que vêm da
> FK do URDF (`home` = (2,285 · 0,000 · 0,625) contra (1,522 · 0,763 · 0,5705) aqui). Os
> erros cartesianos reportados por este nó são internamente consistentes, mas não comparáveis
> com os dos pacotes de FK/IK.

### 5.3 Cinemática local

O arquivo **importa** `dh_matrix`, `compute_gravity_vector` e `DH_PARAMS` de
`computed_torque.py` e define suas próprias:

- `forward_kinematics(q)` — encadeia as 6 matrizes DH, retorna `T` e a lista de transformações;
- `geometric_jacobian(q, transforms)` — Jacobiano **analítico** (não numérico), pela fórmula
  clássica para juntas rotacionais:
  ```
  J_v,i = z_i × (p_n − p_i)
  J_ω,i = z_i
  ```

É uma implementação diferente da de `kuka_forward_kinematics` — mais rápida, mas amarrada ao
modelo DH.

### 5.4 Parâmetros e interfaces

| Parâmetro | Padrão | Uso |
|---|---|---|
| `kp_pos` | `[300, 300, 300]` | Ganho proporcional cartesiano (N/m) |
| `kd_pos` | `[30, 30, 30]` | Ganho derivativo cartesiano (N·s/m) |
| `target` | `center` | Chave de `CARTESIAN_TARGETS` / `JOINT_SEEDS`, ou `custom` |
| `custom_positions` | `[0,0,0,0,0,0]` | Configuração de juntas usada quando `target=custom` |

| Publica | Conteúdo |
|---|---|
| `/kuka_control_strategies/os_error_cartesian` | Erro cartesiano `[ex, ey, ez]` em metros |
| `/kuka_control_strategies/tcp_pose` | `PoseStamped` do TCP no frame `base_link` |
| `/kuka_control_strategies/os_torque` | `τ = Jᵀ·F + g` |

No loop de 50 Hz, calcula FK e Jacobiano na configuração medida e aplica a lei PD no espaço
operacional:

```python
v_tcp = J[:3, :] @ qd                      # velocidade linear do TCP
F_lin = kp_pos * ep − kd_pos * v_tcp       # força virtual no TCP
tau   = Jᵀ @ [F_lin, 0, 0, 0] + g(q)
```

A velocidade cartesiana do TCP é obtida projetando `q̇` pela parte linear do Jacobiano, que é
a forma correta de montar o termo derivativo em task-space (derivar `p` numericamente
introduziria ruído).

### 5.5 Correções aplicadas nesta versão

**(a) `custom_positions` não declarado — nó não subia.**

```python
custom = self.get_parameter('custom_positions').value
```

O parâmetro era lido sem ter sido declarado. Em ROS 2 isso lança
`ParameterNotDeclaredException` e o nó **morria no `__init__`**. Corrigido com a declaração
junto aos demais parâmetros:

```python
self.declare_parameter('custom_positions', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
```

Foi adicionada também **validação de tamanho**: se `custom_positions` não tiver exatamente 6
elementos, o nó loga erro e cai para o alvo `center` em vez de quebrar ao montar o vetor.

**(b) `kp_pos` / `kd_pos` eram declarados mas ignorados.**

A força virtual usava um ganho fixo `300.0` embutido no código, e não havia termo derivativo
algum. Agora os dois parâmetros alimentam de fato a lei de controle (ver 5.4). Como o padrão
de `kp_pos` é `[300, 300, 300]`, o comportamento nominal permanece equivalente ao anterior —
mas agora é sintonizável, e o amortecimento passa a existir.

> O valor publicado em `/kuka_control_strategies/os_torque` muda em consequência: antes era
> `Jᵀ·(300·ep) + g`, agora inclui o termo `−Kd_pos·v_tcp`. Dados coletados antes e depois da
> correção não são diretamente comparáveis.

---

## 6. `strategy_benchmark.py` — comparação quantitativa

Nó puramente observador: **não comanda o robô**, apenas assina os três tópicos de erro e
compila estatísticas.

### 6.1 Parâmetros

| Parâmetro | Padrão | Uso |
|---|---|---|
| `duration` | `30.0` s | Janela de coleta antes do relatório |
| `threshold` | `0.01` rad | Critério de assentamento |

### 6.2 Tópicos observados

| Estratégia | Tópico |
|---|---|
| `local_pd` | `/kuka_control_strategies/pd_error` |
| `computed_torque` | `/kuka_control_strategies/ct_error` |
| `operational_space` | `/kuka_control_strategies/os_error_cartesian` |

Os três callbacks são o mesmo `_cb_error` amarrado por `lambda` com a chave da estratégia.
Para cada mensagem: calcula a norma do vetor, marca `start` na primeira amostra, acumula na
lista e — se a norma cair abaixo de `threshold` após pelo menos 10 amostras — registra o
**settling time**.

### 6.3 Relatório

Um timer de 1 s verifica o tempo decorrido; ao atingir `duration`, `_publish_report()` monta
por estratégia:

| Métrica | Definição |
|---|---|
| `rms_error` | √(média dos erros²) |
| `max_error` | Pico |
| `mean_error` | Média |
| `settling_time` | Primeiro instante abaixo do limiar (−1 se nunca) |
| `n_samples` | Nº de mensagens recebidas |

O relatório é impresso formatado no log e publicado como JSON em
`/kuka_control_strategies/benchmark`. Há também log parcial a cada ~5 s.

> **Comparação de unidades diferentes:** `local_pd` e `computed_torque` reportam erro em
> **radianos** (espaço de juntas); `operational_space` reporta em **metros** (espaço
> cartesiano). O RMS das três aparece lado a lado na mesma tabela, mas **não são grandezas
> comparáveis**. Interprete cada linha isoladamente ou normalize antes de concluir qual
> estratégia é "melhor".

---

## 7. `launch/control_strategies.launch.py`

Sobe os **quatro nós simultaneamente**, com ganhos pré-definidos:

| Nó | Ganhos no launch |
|---|---|
| `local_pd_controller` | Kp `[200,200,150,100,80,50]`, Kd `[20,20,15,10,8,5]` |
| `computed_torque_controller` | Kp `[150,150,120,80,60,40]`, Kd `[25,25,20,15,10,8]` |
| `operational_space_controller` | Kp_pos `[300,300,300]`, Kd_pos `[30,30,30]`, target fixo `center` |
| `strategy_benchmark` | `duration=60.0`, `threshold=0.01` |

Argumento de launch: `target` (padrão `pick`), aplicado ao PD local e ao torque computado.

**Cuidado ao usar este launch:** os três nós disputam o mesmo action server, e o JTC aborta o
goal anterior a cada novo. Ele é útil principalmente para levantar `strategy_benchmark` junto
de **um** controlador. (O antigo impedimento adicional — o `operational_space` não subir por
causa do parâmetro não declarado — está corrigido; ver 5.5.)

---

## 8. Uso recomendado

```bash
# Um controlador por vez
ros2 run kuka_control_strategies local_pd_controller --ros-args -p target:=pick
ros2 run kuka_control_strategies computed_torque     --ros-args -p target:=pick
ros2 run kuka_control_strategies operational_space   --ros-args -p target:=center

# Posição arbitrária — PD local
ros2 run kuka_control_strategies local_pd_controller --ros-args \
  -p target:=custom -p custom_positions:="[0.5, -1.2, 1.0, 0.3, 0.6, 0.0]"

# Posição arbitrária — espaço operacional (o alvo cartesiano sai da FK do seed)
ros2 run kuka_control_strategies operational_space --ros-args \
  -p target:=custom -p custom_positions:="[0.5, -1.2, 1.0, 0.3, 0.6, 0.0]"

# Sintonia do PD cartesiano
ros2 run kuka_control_strategies operational_space --ros-args \
  -p kp_pos:="[500.0, 500.0, 500.0]" -p kd_pos:="[50.0, 50.0, 50.0]"

# Benchmark em terminal separado, junto de um controlador
ros2 run kuka_control_strategies strategy_benchmark --ros-args -p duration:=60.0

# Acompanhar as métricas
ros2 topic echo /kuka_control_strategies/pd_error
ros2 topic echo /kuka_control_strategies/benchmark
```

---

## 9. Comparação conceitual das três estratégias

| Aspecto | Local (PD) | Centralizado (CT) | Espaço operacional |
|---|---|---|---|
| Espaço de referência | Junta | Junta | Cartesiano |
| Usa modelo dinâmico | Não | Sim (aqui: só `g`) | Parcial (`g` + Jacobiano) |
| Custo computacional | Muito baixo | Alto | Médio |
| Desempenho uniforme na área de trabalho | Não | Sim (em teoria) | Sim para a tarefa |
| Sensível a erro de modelagem | Não | Sim | Sim |
| Comportamento em singularidade | Indiferente | Indiferente | Degrada (`Jᵀ` perde posto) |
| Natural para tarefas de contato | Não | Não | Sim |

---

## 10. Resumo dos pontos de atenção

### Corrigidos nesta versão

1. ✅ `operational_space.py` lia `custom_positions` sem declará-lo → exceção no `__init__`.
   **Declarado**, com validação de tamanho.
2. ✅ `kp_pos` / `kd_pos` eram declarados mas ignorados (ganho fixo 300, sem derivativo).
   **Agora alimentam a lei de controle**, com o termo derivativo obtido de `J·q̇`.

### Ainda em aberto

3. Nenhum nó aplica torque de fato; o movimento vem sempre do JTC.
4. `computed_torque` implementa apenas `g(q)`; `M` e `C` estão ausentes.
5. Modelo DH e massas de `computed_torque.py` divergem do modelo URDF de `kinematics.py`.
6. `CARTESIAN_TARGETS` incompatível com as poses de referência do workspace.
7. `computed_torque` declara `custom_positions` mas nunca o consulta.
8. O benchmark mistura rad e m na mesma tabela.
9. Rodar os três controladores juntos causa disputa de goals no action server.
10. `errors_history` cresce sem limite.
