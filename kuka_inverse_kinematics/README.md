# `kuka_inverse_kinematics`

**Cinemática inversa numérica do KUKA KR70 R2100 — três solvers iterativos, execução no robô e gráficos de convergência.**

Este pacote resolve o problema inverso: *dada uma pose cartesiana desejada do TCP, quais os
ângulos das 6 juntas?* Não há solução analítica fechada implementada — todos os métodos são
**iterativos baseados no Jacobiano**, apoiados na FK do pacote `kuka_forward_kinematics`.

---

## 1. Visão geral

| Item | Valor |
|---|---|
| Build type | `ament_python` |
| Versão | 0.1.0 |
| Licença | Apache-2.0 |
| Dependências | `rclpy`, **`kuka_forward_kinematics`**, `sensor_msgs`, `geometry_msgs`, `trajectory_msgs`, `std_msgs`, `control_msgs` |

### Executáveis

| Executável | Módulo | Papel |
|---|---|---|
| `ik_node` | `ik_node.py` | Resolve a IK para um alvo e **envia o movimento ao robô** via action |
| `ik_plotter` | `ik_plotter.py` | Offline: resolve com os 3 métodos e gera 6 gráficos comparativos |
| `ik_comparison` | `ik_comparison.py` | Resolve, executa e **grava a resposta dos sensores** para validar a solução |

> Note que `ik_solver.py` **não** tem entry point — é biblioteca, importada pelos outros três.

---

## 2. `ik_solver.py` — a biblioteca de solvers

### 2.1 Estratégia de importação com *fallback*

O arquivo tenta importar `forward_kinematics`, `geometric_jacobian`, `rotation_to_euler_zyx`,
`JOINT_LIMITS` e `JOINT_NAMES` de `kuka_forward_kinematics.kinematics`. Se o import falhar
(`ImportError`), ele **redefine tudo inline** — uma cópia byte-a-byte equivalente da FK do
URDF, dos limites de junta e do Jacobiano numérico.

Isso torna o módulo autocontido: pode ser usado como script Python puro, fora do ROS, sem o
pacote de FK compilado. O preço é **duplicação de código**: se `_JOINTS` mudar em
`kinematics.py`, o bloco de fallback fica desatualizado silenciosamente. É o principal risco
de manutenção do pacote.

### 2.2 `_rot_err(R_des, R_cur)` — erro de orientação

Converte a discrepância de orientação em um **vetor de rotação** (eixo × ângulo), que é a
forma correta de alimentar as linhas 3–5 do Jacobiano:

```
R_e = R_des · R_curᵀ
θ   = arccos( clip( (tr(R_e) − 1)/2 , −1, 1 ) )
eixo = [R_e[2,1]−R_e[1,2], R_e[0,2]−R_e[2,0], R_e[1,0]−R_e[0,1]] / (2 sin θ)
erro = eixo · θ
```

O `np.clip` protege contra erro numérico que faria `arccos` receber algo levemente fora de
[−1, 1]. Quando `θ < 1e-8` retorna vetor nulo, evitando divisão por `sin θ ≈ 0`.

### 2.3 Estrutura comum aos três solvers

Todos seguem o mesmo esqueleto e devolvem o **mesmo dicionário `hist`**:

```
para it em 0..max_iter-1:
    T = FK(q)
    ep = p_des − T[:3,3]                 # erro de posição (m)
    eo = _rot_err(R_des, T[:3,:3])       # erro de orientação (rad)
    e  = [ep; eo]                        # vetor 6×1
    registra ‖ep‖ e ‖eo‖ no histórico
    se ‖ep‖ < tol_pos e ‖eo‖ < tol_ori: convergiu, break
    J = jacobiano_numérico(q)
    dq = <regra de atualização — muda por método>
    dq = clip(dq, −0.15, +0.15)          # passo máximo por iteração
    q  = clip(q + dq, limites_de_junta)  # respeita JOINT_LIMITS
    registra q no histórico
```

Duas salvaguardas importantes aparecem em todos:

- **Saturação do passo** (`±0,15 rad ≈ 8,6°`): impede saltos enormes perto de singularidades,
  onde o Jacobiano é mal-condicionado e o `dq` calculado explodiria.
- **Clamp nos limites** (`_clamp`): a solução nunca sai da faixa mecânica do robô. Isso torna
  o método um *projected gradient* — pode ficar preso numa face do domínio.

O `for/else` do Python é usado com elegância aqui: o bloco `else` só executa se o laço
terminar sem `break`, marcando `converged = False`.

#### Campos do dicionário de resultado

| Chave | Conteúdo |
|---|---|
| `q` | Lista com o vetor de juntas a cada iteração (histórico completo) |
| `error_pos`, `error_ori` | Séries de ‖ep‖ (m) e ‖eo‖ (rad) por iteração |
| `method` | Nome legível do método |
| `converged` | `True`/`False` |
| `iterations` | Nº de iterações efetivamente gastas |
| `q_result` | Vetor final de juntas |
| `final_error_pos`, `final_error_ori` | Últimos erros registrados |
| `p_des`, `R_des` | Adicionados por `solve_ik` (usados pelos plotters) |

### 2.4 Os três métodos

#### `ik_levenberg_marquardt` — **recomendado**

```python
dq = solve( JᵀJ + λI , Jᵀe )     # λ = lam = 0.01
```

Defaults: `max_iter=300`, `tol_pos=1e-4` (0,1 mm), `tol_ori=1e-3` rad (≈0,057°), `lam=0.01`.

A regularização de Tikhonov (`λI`) torna o sistema **sempre invertível**, mesmo em
singularidades onde `J` perde posto. Interpola entre Gauss-Newton (λ→0, rápido) e gradiente
descendente (λ grande, robusto). O λ aqui é **fixo** — não há a adaptação dinâmica do LM
clássico (aumentar λ quando a iteração piora o erro, reduzir quando melhora).

#### `ik_jacobian_pinv` — pseudo-inverso

```python
dq = α · pinv(J) · e     # α = alpha = 0.5
```

Defaults: `max_iter=400`, `alpha=0.5`. Usa `np.linalg.pinv` (SVD com truncamento de valores
singulares). Converge rápido longe de singularidades, mas fica numericamente instável perto
delas — motivo pelo qual precisa de mais iterações que o LM.

#### `ik_jacobian_transpose` — transposto

```python
dq = α · Jᵀ · e          # α = alpha = 0.02
```

Defaults: `max_iter=600`, `alpha=0.02`. É essencialmente **gradiente descendente** sobre
½‖e‖². Não requer inversão alguma (barato por iteração e imune a singularidades), mas a
convergência é linear e lenta — daí `α` pequeno e 600 iterações. Serve principalmente como
termo de comparação didática nos gráficos.

### 2.5 `solve_ik(...)` — interface unificada com multi-restart

```python
solve_ik(p_des, R_des=None, q_init=None, method='lm', n_restarts=5, **kwargs)
```

Comportamento:

1. Se `R_des is None`, assume **identidade** (orientação alinhada com a base).
2. Se `q_init is None`, parte de zeros.
3. Escolhe o solver pelo dicionário `{'lm', 'pinv', 'transpose'}` — chave desconhecida cai no
   LM por padrão.
4. **Multi-restart:** roda até `n_restarts` tentativas. A tentativa 0 usa `q_init`; as demais
   sorteiam uma configuração inicial aleatória uniforme dentro dos limites de junta
   (`rng = default_rng(0)` → **reprodutível**).
5. Mantém sempre a melhor tentativa pelo critério `final_error_pos`.
6. Sai assim que uma tentativa converge.

O multi-restart ataca o problema real de métodos locais: mínimos locais e configurações
iniciais ruins. Como o KR70 tem múltiplas soluções IK (elbow-up/down, wrist-flip), o alvo
pode ser inalcançável a partir de um `q_init` específico mas alcançável de outro.

> **Detalhe:** `solve_ik` sempre passa `R_des` ao solver. Quando o chamador não informa
> orientação, o solver tenta atingir **posição *e* orientação identidade** simultaneamente —
> o que pode ser inalcançável mesmo quando a posição sozinha seria. É o que acontece em
> `ik_comparison` e em `ik_plotter`, que chamam `solve_ik` sem `R_des`.

---

## 3. `ik_node.py` — resolver e mover o robô

Nó `ik_node` que fecha o ciclo: recebe alvo → resolve → verifica → **comanda o robô**.

### 3.1 Parâmetros

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `target_x` / `target_y` / `target_z` | 1.2 / 0.5 / 0.8 | Posição desejada do TCP (m) |
| `target_yaw` / `target_pitch` / `target_roll` | 0.0 | Orientação desejada em **graus** (Euler ZYX) |
| `method` | `lm` | `lm` \| `pinv` \| `transpose` |
| `execute` | `True` | Se `False`, apenas resolve e publica, sem mover |
| `max_iter` | 200 | Limite de iterações |
| `tol_pos` | 1e-4 | Tolerância de posição (m) |
| `tol_ori` | 1e-3 | Tolerância de orientação (rad) |
| `move_time` | 4.0 | Duração do movimento (s) |

Os ângulos de Euler são convertidos em `R_des` por `_euler_zyx_to_rotation` (`Rz·Ry·Rx`).

### 3.2 Interfaces ROS

| Direção | Nome | Tipo |
|---|---|---|
| Assina | `/joint_states` | `sensor_msgs/JointState` (seed do solver) |
| Assina | `/ik/target_pose` | `geometry_msgs/PoseStamped` (alvo dinâmico) |
| Publica | `/ik/joint_solution` | `std_msgs/Float64MultiArray` (6 juntas em rad) |
| Publica | `/ik/convergence_info` | `std_msgs/String` (JSON de métricas) |
| Publica | `/ik/estimated_tcp` | `geometry_msgs/PoseStamped` (TCP re-verificado por FK) |
| Action client | `/kuka_arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` |

### 3.3 Sequência de execução

Um timer de **2 s** dispara `_solve_and_execute()` — o atraso deliberado dá tempo de o
`/joint_states` chegar, para que `q_current` sirva de seed realista.

> **Comportamento a observar:** `create_timer(2.0, ...)` cria um timer **periódico**, não
> single-shot. O nó re-resolve e re-envia o goal a cada 2 segundos enquanto estiver vivo.
> Para uma execução única, encerre o nó após o movimento ou use `execute:=false`.

Dentro de `_solve_and_execute`:

1. Chama `solve_ik` com o seed atual.
2. **Verificação independente:** aplica a FK sobre `q_sol` e compara com `p_des`
   (`|p_des − p_fk|` em mm). É uma checagem de sanidade contra bugs do solver.
3. Loga convergência, iterações, erro de posição (mm), erro de orientação (graus) e a solução
   junta a junta em graus.
4. Publica os três tópicos de saída.
5. Se `execute=True` **e** convergiu, chama `_send_to_robot`.

### 3.4 `_send_to_robot(q_sol)`

Monta um `JointTrajectory` de **um único ponto** (posições = solução, velocidades e
acelerações zeradas, `time_from_start = move_time`) e envia como goal `FollowJointTrajectory`.
O `JointTrajectoryController` do `ros2_control` faz a interpolação interna.

Se o action server não responder em 2 s, loga erro e desiste. Os callbacks `_goal_cb` tratam
aceitação/rejeição e registram a conclusão.

> `Duration(sec=int(self.move_time))` **trunca** o valor para inteiro — `move_time:=4.5`
> vira 4 s.

### 3.5 Alvo por tópico

Ao receber `/ik/target_pose`, o nó converte o quaternion em matriz de rotação
(`_quat_to_rotation`, fórmula direta sem normalização prévia) e dispara imediatamente uma nova
resolução. Isso permite pilotar o robô publicando poses.

---

## 4. `ik_comparison.py` — IK estimada vs sensores

Junta resolução, execução e **coleta de dados** num único nó, para produzir a evidência de que
a solução IK é de fato atingida pelo robô simulado.

### 4.1 Máquina de coleta

O nó mantém a flag `self.collecting`. Enquanto `False`, o callback de `/joint_states` só
atualiza `q_current`. Assim que a IK é resolvida e o goal é enviado, `collecting = True` e o
mesmo callback passa a gravar, a cada mensagem:

- `sensor_times` — tempo relativo ao primeiro dado;
- `sensor_q` — ângulos medidos;
- `sensor_xyz` — TCP calculado por FK sobre os ângulos medidos;
- `ik_q_repeated` — a solução IK repetida (constante), para alinhar as séries.

### 4.2 Parâmetros

| Parâmetro | Padrão |
|---|---|
| `target_x` / `target_y` / `target_z` | 1.2 / 0.5 / 0.8 |
| `method` | `lm` |
| `label` | `alvo` (sufixo dos arquivos) |
| `output_dir` | `~/kuka_ik_plots` |
| `move_time` | 5.0 |

Aqui `solve_ik` é chamado **sem `R_des`** e com `max_iter=300`.

### 4.3 Gráficos gerados (`Ctrl+C`)

| Arquivo | Conteúdo |
|---|---|
| `ik_comparison_joints_<label>.png` | Grade 2×3: curva do sensor por junta + linha tracejada da solução IK + limites de junta pontilhados |
| `ik_comparison_error_<label>.png` | Grade 2×3: \|sensor − solução IK\| por junta, com RMS no título |
| `ik_comparison_3d_<label>.png` | Trajetória 3D do TCP, com marcadores de início/fim, o alvo (X preto) e uma linha vermelha ligando posição final ao alvo, rotulada com o erro em mm |

Este é o gráfico que responde "o robô chegou onde a IK mandou?". O erro residual visível
mistura três fontes: erro de convergência do solver, erro de rastreamento do
`JointTrajectoryController` e dinâmica não modelada do Gazebo.

---

## 5. `ik_plotter.py` — análise offline dos métodos

Nó one-shot, **sem simulação**, que resolve o mesmo alvo com os três métodos e produz o
conjunto completo de gráficos comparativos.

### 5.1 Fluxo

```python
for method in ['pinv', 'transpose', 'lm']:
    r = solve_ik(p_des, q_init=zeros, method=method, max_iter=300)
```

Os três resultados alimentam as funções de plotagem. O LM (índice 2) é tratado como
"melhor método" e recebe os gráficos individuais.

### 5.2 Funções de plotagem

| Função | Saída | Detalhes |
|---|---|---|
| `plot_joint_evolution` | `ik_joint_evolution_lm.png` | Grade 2×3, ângulo de cada junta × iteração, com limites tracejados e preenchimento entre a curva e o valor final |
| `plot_convergence` | `ik_convergence_lm.png` | Dois subplots em **escala logarítmica**: erro de posição (mm) e de orientação (graus), com linhas de tolerância |
| `plot_methods_comparison` | `ik_methods_comparison.png` | Os três métodos sobrepostos em semilog, cor e estilo distintos, nº de iterações na legenda |
| `plot_joint_space_trajectory` | `ik_joint_space_trajectory.png` | Caminho percorrido no espaço de juntas por cada método |
| `plot_solution_3d` | `ik_solution_3d.png` | Robô desenhado na configuração solução, TCP (estrela), alvo (X vermelho) e linha de erro rotulada em mm |
| `_plot_multiple_targets` | `ik_multiple_targets_3d.png` | Resolve 4 alvos fixos e desenha as 4 posturas resultantes, com ✓/✗ de convergência na legenda |

O uso de **semilog** nos gráficos de convergência é o que revela a diferença qualitativa entre
os métodos: o LM cai quase verticalmente (convergência quadrática perto da solução), o
pseudo-inverso desce em rampa, e o transposto forma uma reta longa de inclinação suave
(convergência linear).

Os 4 alvos fixos de `_plot_multiple_targets`: `(1.2, 0.5, 0.8)`, `(1.0, −0.5, 1.0)`,
`(0.8, 0.0, 1.5)`, `(1.4, 0.3, 0.5)`.

### 5.3 Parâmetros

`output_dir` (padrão `~/kuka_ik_plots`), `target_x`, `target_y`, `target_z`.

---

## 6. `launch/inverse_kinematics.launch.py`

Sobe `ik_node` **e** `ik_plotter` juntos, compartilhando os mesmos argumentos de alvo.
Argumentos disponíveis: `target_x`, `target_y`, `target_z`, `method`, `execute`, `output_dir`.

Resultado prático: o `ik_plotter` gera os PNGs de análise e encerra, enquanto o `ik_node`
resolve e comanda o robô.

---

## 7. Uso típico

```bash
# Resolver e mover o robô
ros2 run kuka_inverse_kinematics ik_node --ros-args \
  -p target_x:=1.0 -p target_y:=-0.6 -p target_z:=1.1 -p method:=lm

# Resolver sem mover (só análise)
ros2 run kuka_inverse_kinematics ik_node --ros-args \
  -p target_x:=1.0 -p target_y:=-0.6 -p target_z:=1.1 -p execute:=false

# Resolver, mover e coletar dados (Ctrl+C gera os gráficos)
ros2 run kuka_inverse_kinematics ik_comparison --ros-args \
  -p target_x:=1.0 -p target_y:=-0.6 -p target_z:=1.1 -p label:=alvo1

# Comparar os 3 métodos offline
ros2 run kuka_inverse_kinematics ik_plotter

# Alvo dinâmico por tópico
ros2 topic pub /ik/target_pose geometry_msgs/PoseStamped \
  "{pose: {position: {x: 1.0, y: -0.6, z: 1.1}, orientation: {w: 1.0}}}"
```

---

## 8. Desempenho esperado

| Método | Iterações típicas | Robustez em singularidade | Custo/iteração |
|---|---|---|---|
| Levenberg-Marquardt | Dezenas | Alta (regularização λ) | Resolve sistema 6×6 |
| Pseudo-inverso | Centenas | Média (SVD ajuda, mas degrada) | SVD 6×6 |
| Transposto | Muitas centenas | Alta (não inverte nada) | Só produto matriz-vetor |

Todos pagam 6 avaliações extras de FK por iteração, devido ao Jacobiano numérico.

---

## 9. Pontos de atenção

- **Duplicação de código:** o bloco de fallback em `ik_solver.py` replica a FK. Se
  `kinematics.py` mudar, esse bloco precisa ser atualizado manualmente.
- **Timer periódico** em `ik_node` e `ik_comparison`: `create_timer(2.0, ...)` re-dispara
  indefinidamente.
- **Orientação implícita:** chamar `solve_ik` sem `R_des` força orientação identidade, o que
  restringe o conjunto de alvos alcançáveis.
- **`Duration(sec=int(...))`** trunca tempos fracionários.
- **`_quat_to_rotation`** não normaliza o quaternion recebido; quaternions não unitários
  produzem uma "matriz de rotação" com escala.
- Se a IK não convergir, `ik_node` **não** envia o movimento (comportamento seguro, mas o
  robô simplesmente não se move e a única pista é o log).
