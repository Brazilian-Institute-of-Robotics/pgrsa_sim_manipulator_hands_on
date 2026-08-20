# `kuka_gain_analysis`

**Objetivo 2 — Estimar a influência das variações nos ganhos e no tipo de controle sobre a estabilidade e o desempenho do sistema.**

> **Estado desta versão:** `gain_sweep` agora aplica os ganhos ao controlador via
> `rcl_interfaces/SetParameters` e usa máquina de estados com timer único. Detalhes em 2.2, 2.7 e 2.8.

Pacote de **experimentação automatizada**: varre sistematicamente o espaço de ganhos Kp × Kd,
mede o comportamento resultante, detecta instabilidade em tempo real e produz um relatório com
recomendação de sintonia.

---

## 1. Arquitetura

Três nós que operam em cadeia, ligados por um arquivo CSV e por tópicos:

```
                    ┌──► /<controller>/set_parameters   (aplica Kp, Kd)
                    │    /<controller>/list_parameters  (sondagem inicial)
                    │
  gain_sweep ───────┼──► /<controller>/joint_trajectory (degrau home → pick)
      │             │
      │             └──► /kuka_gain_analysis/current_metrics
      │
      ├──► output_csv (com coluna gains_applied) ──► gain_report ──► relatório
      └──► /kuka_gain_analysis/sweep_status ─────────────┘

  stability_monitor ──► /kuka_gain_analysis/stability_alert   (independente, tempo real)
```

| Item | Valor |
|---|---|
| Build type | `ament_python` |
| Versão | 0.1.0 |
| Dependências | `rclpy`, `std_msgs`, `sensor_msgs`, `trajectory_msgs`, `rcl_interfaces` |

### Executáveis

| Executável | Papel |
|---|---|
| `gain_sweep` | Executa a bateria de testes e grava o CSV |
| `stability_monitor` | Detecta divergência, oscilação e chatter em tempo real |
| `gain_report` | Lê o CSV e emite o relatório com recomendação |

---

## 2. `gain_sweep.py` — varredura automática de ganhos

### 2.1 O experimento

Para cada combinação `(Kp, Kd)` do produto cartesiano das listas configuradas:

1. **Aplica os ganhos** ao controlador via `rcl_interfaces/SetParameters` e aguarda a
   confirmação do serviço;
2. Reseta buffers e comanda o robô de `home` para `pick` (degrau);
3. Coleta o erro por `test_duration` segundos a 20 Hz;
4. Calcula RMS, pico, overshoot, settling time e flag de instabilidade;
5. Acumula o resultado (incluindo se os ganhos foram de fato aplicados);
6. Manda o robô de volta a `home` e espera `home_settle` segundos;
7. Passa à próxima combinação.

Com os padrões (5 valores de Kp × 5 de Kd = **25 combinações**, 8 s cada + 3 s de retorno), a
varredura leva aproximadamente **4 a 5 minutos**.

### 2.2 Máquina de estados

O nó é dirigido por **um único timer a 20 Hz** que despacha conforme o estado:

```
INIT ──(startup_delay)──► SET_GAINS ──(SetParameters respondeu)──► RUNNING
                              ▲                                       │
                              │                          (test_duration decorrido)
                              │                                       ▼
                              └────(home_settle)──────────────────  HOMING
                                                                      │
                                              (última combinação) ────► DONE
```

| Estado | O que acontece |
|---|---|
| `INIT` | Espera `startup_delay` s para o Gazebo estabilizar |
| `SET_GAINS` | Requisição assíncrona de `SetParameters`; aguarda o *future* resolver |
| `RUNNING` | Degrau enviado, coleta de métricas a 20 Hz |
| `HOMING` | Robô retorna a `home`, aguarda `home_settle` s |
| `DONE` | CSV salvo, status final publicado, nó ocioso |

O `SET_GAINS` tem **timeout de segurança** (`gain_service_timeout`): se o serviço não
responder, o nó registra `gains_applied=False` e prossegue, em vez de travar a varredura.

> Esta estrutura substitui a versão anterior, que chamava `create_timer(3.0, ...)` dentro de
> `_finish_test` a cada teste sem nunca cancelar — ao fim das 25 combinações havia dezenas de
> timers periódicos vivos, reiniciando testes já concluídos e reescrevendo o CSV
> repetidamente. Não há mais criação dinâmica de timers.

### 2.3 Parâmetros

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `kp_values` | `[50, 100, 200, 400, 800]` | Escala aproximadamente logarítmica (×2) |
| `kd_values` | `[5, 10, 20, 40, 80]` | Mantém razão Kp/Kd ≈ 10 na diagonal |
| `test_duration` | `8.0` s | Tempo de coleta por combinação |
| `settle_threshold` | `0.02` rad | ≈1,15° — critério de assentamento |
| `output_csv` | `/tmp/gain_sweep_results.csv` | Arquivo de saída |
| `controller_name` | `kuka_arm_controller` | Nome do controlador alvo (sem `/`) |
| `apply_gains` | `True` | `False` = modo somente-observação, sem tocar no controlador |
| `gain_service_timeout` | `5.0` s | Timeout de `wait_for_service` e do *future* |
| `startup_delay` | `3.0` s | Espera antes do primeiro teste |
| `home_settle` | `3.0` s | Espera após retornar a `home` |

A escolha geométrica dos valores (dobrando a cada passo) é apropriada: efeitos de ganho são
multiplicativos, e uma grade linear desperdiçaria pontos na região de baixo ganho.

### 2.4 Alvos

```python
TARGET_HOME = [0.0,  0.0, 0.0, 0.0, 0.0, 0.0]
TARGET_PICK = [0.0, -1.0, 1.2, 0.0, 0.8, 0.0]
```

Cada teste é um **degrau** de `home` para `pick` — o estímulo clássico para levantar resposta
transitória.

### 2.5 Métricas calculadas (`_finish_test`)

| Métrica | Cálculo | Interpretação |
|---|---|---|
| `rms_error` | `√(mean(e²))` sobre toda a janela | Desempenho médio de rastreamento |
| `max_error` | `peak_error` acumulado | Pico do transitório |
| `final_error` | Média das **últimas 10 amostras** | Erro de regime permanente |
| `overshoot` | `(pico − final) / inicial`, com piso em 0 | Sobressinal normalizado |
| `settling_time` | Primeiro instante com `‖e‖ < settle_threshold`; `−1` se nunca | Velocidade de resposta |
| `unstable` | Flag booleana (ver 2.6) | Estabilidade |
| `gains_applied` | **Coluna nova** — `True` se o `SetParameters` foi confirmado | Validade do dado |

A coluna `gains_applied` é o que permite distinguir um resultado significativo de ruído: se
vier `False` em todas as linhas, os ganhos não chegaram ao controlador e as diferenças entre
combinações não têm significado físico.

### 2.6 Detecção de instabilidade

```python
if (len(errors_buffer) > 50 and
        norm_e > 3 * initial_error and
        norm_e > errors_buffer[-50]):
    unstable = True
```

Três condições simultâneas:

1. **pelo menos 50 amostras** (≈2,5 s) — evita falso positivo no transitório inicial;
2. **erro maior que 3× o inicial** — magnitude anormal;
3. **erro maior que o de 50 amostras atrás** — está *crescendo*, não decaindo.

A terceira condição é o que distingue divergência real de um overshoot grande porém
convergente.

### 2.7 Aplicação dos ganhos via `SetParameters`

Esta é a parte que torna a varredura significativa. O nó abre dois clientes de serviço no
controlador alvo:

| Cliente | Serviço | Uso |
|---|---|---|
| `cli_set_params` | `/<controller>/set_parameters` | Aplica os ganhos de cada combinação |
| `cli_list_params` | `/<controller>/list_parameters` | Sondagem inicial: os ganhos existem? |

#### Sondagem inicial (`_probe_gain_parameters`)

Antes de começar, o nó chama `ListParameters` com `prefixes=['gains']` e verifica se existem
parâmetros `gains.<joint>.p` para as 6 juntas. Três desfechos:

| Situação | Comportamento |
|---|---|
| Parâmetros encontrados | `gain_params_available = True` — reconfiguração **ATIVA** |
| `set_parameters` indisponível | Log de erro; segue em **modo somente-observação** |
| `list_parameters` indisponível | Log de aviso; assume que existem e tenta aplicar mesmo assim |
| Nenhum `gains.*` listado | Log de erro explicando a causa provável; **somente-observação** |

#### Montagem da requisição (`_build_gain_request`)

Para cada uma das 6 juntas, monta dois `rcl_interfaces/msg/Parameter` com
`ParameterType.PARAMETER_DOUBLE` — **12 parâmetros por requisição**:

```python
gains.joint_1.p = Kp     gains.joint_1.d = Kd
gains.joint_2.p = Kp     gains.joint_2.d = Kd
...                      ...
gains.joint_6.p = Kp     gains.joint_6.d = Kd
```

O mesmo par `(Kp, Kd)` é aplicado a todas as juntas — coerente com a grade 2D do experimento,
que varre um único par escalar. (Para ganhos por junta seria preciso uma grade 12-dimensional,
inviável por varredura exaustiva.)

#### Chamada assíncrona (`_request_gains` / `_check_gain_future`)

A requisição usa `call_async`, e o *future* é consultado a cada tick do timer no estado
`SET_GAINS`. **Não se usa `spin_until_future_complete` dentro do callback do timer** — isso
causaria reentrância no executor e travaria o nó.

O tratamento da resposta inspeciona `response.results`, que traz um `SetParametersResult` por
parâmetro:

- todos com `successful=True` → `gains_ok = True`, log de confirmação;
- qualquer falha → `gains_ok = False`, log com o `reason` do primeiro parâmetro rejeitado;
- resposta nula ou timeout → `gains_ok = False` e a varredura prossegue.

### 2.8 Pré-requisito no controlador — leia antes de rodar

O `JointTrajectoryController` só **expõe e usa** os parâmetros `gains.<joint>.p/.d` quando
está configurado com `command_interface` de **effort** ou **velocity**, casos em que ele
fecha a malha internamente com um PID por junta.

Com `command_interface: position` — a configuração mais comum em simulação — quem fecha a
malha é o `ign_ros2_control` / hardware, os parâmetros de ganho **não existem**, e nenhuma
varredura os alterará.

Verifique antes de rodar:

```bash
ros2 param list /kuka_arm_controller | grep gains
```

- **Saída com `gains.joint_1.p` etc.** → tudo certo, a varredura terá efeito físico.
- **Saída vazia** → edite o YAML do controlador (no pacote `kuka_gazebo` do KUKA-ROS2) para
  usar `effort` em `command_interfaces` e declare o bloco `gains:`, por exemplo:

```yaml
kuka_arm_controller:
  ros__parameters:
    command_interfaces: [effort]
    state_interfaces:   [position, velocity]
    gains:
      joint_1: {p: 200.0, d: 20.0, i: 0.0, i_clamp: 10.0}
      joint_2: {p: 200.0, d: 20.0, i: 0.0, i_clamp: 10.0}
      # ... demais juntas
```

O nó **detecta a ausência dos parâmetros**, avisa no log, marca `gains_applied=False` em todas
as linhas do CSV e emite um alerta final. O `gain_report` propaga esse aviso ao relatório. Ou
seja: a varredura ainda roda, mas você fica sabendo que o resultado não é interpretável.

### 2.9 Saídas

| Tópico | Tipo | Conteúdo |
|---|---|---|
| `/kuka_gain_analysis/current_metrics` | `Float64MultiArray` | `[Kp, Kd, ‖e‖, tempo_decorrido, flag_instável]` |
| `/kuka_gain_analysis/sweep_status` | `String` | JSON `{test, total, kp, kd, state}` |
| `<output_csv>` | arquivo | Uma linha por combinação, com `gains_applied` |

O JSON de status ganhou o campo `state` (`SET_GAINS` durante a varredura, `DONE` ao final),
usado pelo `gain_report` para saber quando gerar o relatório.

Ao final, `_save_results` cria o diretório de saída se necessário, grava o CSV via
`csv.DictWriter`, alerta se nenhum teste teve ganhos aplicados e loga a melhor combinação
**entre as estáveis**, pelo menor RMS. O `Ctrl+C` também dispara `_save_results`, preservando
os resultados parciais.

---

## 3. `stability_monitor.py` — vigilância em tempo real

Nó independente do sweep, que pode rodar sob qualquer controlador. Mantém janelas deslizantes
e aplica **três testes distintos** de instabilidade.

### 3.1 Parâmetros

| Parâmetro | Padrão | Uso |
|---|---|---|
| `window_size` | `200` | Amostras na janela (a 50 Hz = 4 s) |
| `diverge_factor` | `2.0` | Multiplicador do erro inicial |
| `osc_threshold` | `5` | Base para o critério de chatter (usado como `×2` = 10) |
| `rate_hz` | `50.0` | Frequência do loop e base da FFT |

Alvo de referência fixo no código: `q_target = [0, −1.0, 1.2, 0, 0.8, 0]` (a pose `pick`).

### 3.2 Teste 1 — divergência

```python
if norm_e > diverge_factor * initial_error and norm_e > 0.1:
```

Severidade **HIGH**. A segunda condição (`> 0,1 rad`) evita alarme quando o erro inicial já
era minúsculo — sem ela, um erro de 0,001 rad subindo para 0,003 dispararia o alerta.

### 3.3 Teste 2 — oscilação (análise espectral)

```python
fft   = |rfft(errors − mean(errors))|
freqs = rfftfreq(len(errors), d = 1/rate_hz)
razão = Σ fft[freqs > 2 Hz] / Σ fft
if razão > 0.4: alerta OSCILLATION
```

Remove-se a média antes da FFT para eliminar a componente DC (o erro de regime permanente
dominaria o espectro). Se mais de 40% da energia espectral estiver acima de **2 Hz**, o
sistema está oscilando em vez de convergir suavemente — assinatura típica de `Kp` alto demais
para o `Kd` disponível. Severidade **MEDIUM**.

Com janela de 200 amostras a 50 Hz, a resolução em frequência é 0,25 Hz e a frequência de
Nyquist é 25 Hz.

### 3.4 Teste 3 — chatter

```python
d_errors        = diff(errors)
zero_crossings  = Σ ( sign(d_errors) mudou )
if zero_crossings > osc_threshold * 2: alerta CHATTER
```

Conta quantas vezes a **derivada do erro** troca de sinal na janela. Muitas inversões =
o erro sobe e desce continuamente, sem tendência — comportamento de *chattering*, típico de
`Kd` excessivo (amplificação de ruído) ou de atraso na malha. Severidade **MEDIUM**.

Os três testes são complementares: divergência detecta perda de estabilidade franca;
oscilação detecta ciclo-limite; chatter detecta agitação de alta frequência e baixa amplitude.

### 3.5 Saídas

| Tópico | Tipo | Conteúdo |
|---|---|---|
| `/kuka_gain_analysis/stability_alert` | `String` | JSON com lista de alertas ativos, contadores acumulados e erro atual |
| `/kuka_gain_analysis/stability_metrics` | `Float64MultiArray` | `[‖e‖, desvio_padrão(e), razão_alta_freq, cruzamentos_zero, velocidade_média]` |

Cada alerta traz `type`, `severity`, `message` (já formatada) e `value`. Os contadores em
`alert_counts` acumulam quantas vezes cada tipo disparou desde o início.

> **Detalhe:** o log periódico usa `len(self.error_window) % int(rate_hz*5) == 0`. Como
> `error_window` é um `deque(maxlen=200)`, seu comprimento **satura em 200** — e
> `200 % 250 ≠ 0`. O log "a cada 5 s" nunca dispara depois que a janela enche. Deveria usar um
> contador independente.

---

## 4. `gain_report.py` — relatório e recomendação

Nó que **consome** o CSV do sweep e produz o documento final.

### 4.1 Parâmetros

| Parâmetro | Padrão |
|---|---|
| `input_csv` | `/tmp/gain_sweep_results.csv` |
| `publish_interval` | `10.0` s |

### 4.2 Dois gatilhos

1. **Timer periódico** (`publish_interval`) — tenta gerar; se o CSV ainda não existe, apenas
   loga e aguarda.
2. **Tópico `/kuka_gain_analysis/sweep_status`** — quando chega uma mensagem com
   `state == 'DONE'` (ou `test == total`), marca a flag `sweep_finished` e loga; a geração em
   si fica a cargo do timer que já existe.

> **Corrigido nesta versão:** o gatilho 2 antes chamava `create_timer(5.0, self._try_generate)`
> a cada mensagem de status final, sem cancelar — os timers se acumulavam e o relatório era
> regerado repetidamente. Agora ele apenas sinaliza por flag.

### 4.3 Conteúdo do relatório

**Aviso de validade** (novo): se nenhuma linha tiver `gains_applied=True`, o relatório abre
com um bloco destacado explicando que os números refletem apenas ruído da simulação, e
indicando o comando de diagnóstico do controlador.

**Tabela completa**, ordenada por `(Kp, Kd)`:

```
      Kp       Kd        RMS        Max  Settle(s)  Overshoot  Estável   Ganhos
```

Combinações que nunca assentaram aparecem com `N/A` na coluna de settling time. A coluna
`Ganhos` mostra `OK`, `NÃO` ou `?` — este último para CSVs antigos, gerados antes da coluna
`gains_applied` existir (o parser é retrocompatível).

**Resumo estatístico:** total testado, número e percentual de estáveis e instáveis, e a
contagem `Ganhos aplicados: N/total`.

**Recomendações:**

- **Melhor RMS** — combinação estável com menor erro quadrático médio;
- **Mais rápida** — entre as estáveis que assentaram (`settling_time > 0`), a de menor tempo.
  Usa `min(..., default=None)` para não quebrar se nenhuma assentar.

**Estimativa de Ziegler-Nichols:**

```python
kp_crit = menor Kp entre os instáveis
Kp_sugerido ≈ 0.6 * kp_crit
```

Aplica a regra clássica do método do ganho crítico: o menor `Kp` que produziu instabilidade é
tomado como `Kp_u` (ganho último), e a recomendação para um PID é `0,6·Kp_u`.

> **Limitação metodológica:** o Z-N clássico também exige `T_u`, o **período** da oscilação
> sustentada, para derivar `Ki` e `Kd`. O código comenta a fórmula
> (`Kd_opt = Kp·Tu/8`) mas não a aplica, porque `T_u` não é medido pelo sweep — só o `Kp`
> sugerido é emitido. Medir `T_u` exigiria análise espectral do erro na condição crítica,
> algo que o `stability_monitor` já faz parcialmente via FFT mas não exporta para o CSV.

O relatório é impresso no log e publicado em `/kuka_gain_analysis/gain_report` como `String`.

---

## 5. `launch/gain_analysis.launch.py`

Sobe os três nós simultaneamente, com os parâmetros padrão do sweep e o `gain_report`
apontando para o mesmo CSV. É a forma pretendida de uso, já que os nós se coordenam por
tópico e arquivo.

```bash
ros2 launch kuka_gain_analysis gain_analysis.launch.py
```

---

## 6. Uso típico

```bash
# 0) SEMPRE primeiro: confirmar que o controlador expõe os ganhos
ros2 param list /kuka_arm_controller | grep gains

# Varredura completa (~5 min com os padrões)
ros2 launch kuka_gain_analysis gain_analysis.launch.py

# Varredura reduzida, para teste rápido
ros2 run kuka_gain_analysis gain_sweep --ros-args \
  -p kp_values:="[100.0, 400.0]" \
  -p kd_values:="[10.0, 40.0]" \
  -p test_duration:=5.0

# Controlador com outro nome
ros2 run kuka_gain_analysis gain_sweep --ros-args \
  -p controller_name:=joint_trajectory_controller

# Modo somente-observação (não toca nos parâmetros do controlador)
ros2 run kuka_gain_analysis gain_sweep --ros-args -p apply_gains:=false

# Salvar fora de /tmp (o diretório é criado se não existir)
ros2 run kuka_gain_analysis gain_sweep --ros-args \
  -p output_csv:=~/kuka_gain_plots/sweep.csv

# Inspecionar os ganhos efetivamente aplicados durante a varredura
ros2 param get /kuka_arm_controller gains.joint_2.p

# Só o monitor, sob qualquer controlador
ros2 run kuka_gain_analysis stability_monitor

# Regerar o relatório de um CSV existente
ros2 run kuka_gain_analysis gain_report --ros-args \
  -p input_csv:=/tmp/gain_sweep_results.csv

# Acompanhar em tempo real
ros2 topic echo /kuka_gain_analysis/current_metrics
ros2 topic echo /kuka_gain_analysis/stability_alert
```

---

## 7. Interpretação dos resultados (teoria esperada)

Com os ganhos efetivamente aplicados (`gains_applied=True` — ver 2.8), o padrão esperado no
plano Kp × Kd:

| Região | Comportamento |
|---|---|
| Kp baixo, Kd qualquer | Lento, erro de regime alto, estável |
| Kp alto, Kd baixo | Rápido, muito overshoot, oscilação → **instável** |
| Kp alto, Kd alto | Rápido e amortecido, mas sensível a ruído (chatter) |
| Kp alto, Kd altíssimo | Sobreamortecido, lento de novo |
| Diagonal Kp/Kd ≈ 10–20 | Faixa de melhor compromisso |

Os alertas do `stability_monitor` mapeiam para essas regiões: `DIVERGENCE` na região de Kp alto
com Kd insuficiente; `OSCILLATION` na fronteira de estabilidade; `CHATTER` na região de Kd
excessivo.

---

## 8. Pontos de atenção

### Corrigidos nesta versão

1. ✅ **Reconfiguração dinâmica implementada.** Os ganhos agora são aplicados ao controlador
   via `rcl_interfaces/SetParameters` antes de cada teste (seção 2.7), cumprindo o que o
   docstring já prometia e justificando a dependência `rcl_interfaces` do `package.xml`.
2. ✅ **Timers acumulados eliminados.** `gain_sweep` passou a usar um único timer com máquina
   de estados explícita (seção 2.2); `gain_report._cb_status` agora só marca uma flag.
3. ✅ **Rastreabilidade da validade do dado.** A coluna `gains_applied` no CSV e a coluna
   `Ganhos` no relatório deixam explícito quando os números não são interpretáveis.
4. ✅ `_save_results` cria o diretório de saída e é chamado também no `Ctrl+C`.
5. ✅ O parser do `gain_report` tolera `True`/`true`/espaços e CSVs sem a coluna nova.

### Ainda em aberto

6. **A varredura só tem efeito com `command_interface` de effort/velocity** (seção 2.8). Com
   `position`, o nó avisa e segue em modo somente-observação, mas o pré-requisito continua
   sendo do usuário.
7. O mesmo par `(Kp, Kd)` é aplicado às 6 juntas; não há varredura por junta.
8. O termo integral (`gains.<joint>.i`) não é varrido nem zerado — permanece o valor do YAML.
9. O log periódico do `stability_monitor` nunca dispara após a janela encher
   (`len(deque) % 250` com `maxlen=200`).
10. O alvo do `stability_monitor` é fixo no código (`pick`) — não é parametrizável.
11. `initial_error` é capturado na primeira amostra do teste; se o robô ainda não voltou a
    `home`, o valor de referência sai distorcido e afeta overshoot e detecção de instabilidade.
    Aumentar `home_settle` mitiga.
12. `T_u` não é medido, então a recomendação de Ziegler-Nichols fica incompleta.
13. O CSV padrão vai para `/tmp` — **é perdido no reboot**. Use `output_csv` para persistir.
