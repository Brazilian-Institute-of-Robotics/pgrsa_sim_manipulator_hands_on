# Registro de Atividade: Roteiro Hand-on 3&4

**Nome dos estudantes:** `[ ... ]`

> Preencha cada seção com os dados/valores solicitados, suas observações e as figuras geradas. Substitua os campos `[ ... ]` pelas suas respostas e insira as imagens conforme o tutorial abaixo.

---

## Como inserir imagens no Markdown

Para inserir uma imagem neste arquivo, use a seguinte sintaxe:

```markdown
![texto alternativo](caminho/da/imagem.png)
```

Passo a passo:

1. Copie o arquivo de imagem gerado (por exemplo, em `~/kuka_control_plots/`) para uma pasta do seu repositório, como `imagens/`, para que ela fique versionada junto com o projeto.
2. Escreva o caminho relativo até a imagem, conforme o exemplo a seguir.


Suponha que o arquivo de imagem `erro_cartesiano.png` tenha sido armazenado na pasta `imagens/` do repositório. Para inseri-lo neste relatório, utilize:

```markdown
imagens/erro_cartesiano.png
```

Outro exemplo:

```markdown
imagens/torque_juntas.png
```

3. Indique, entre parênteses `( )`, o caminho até o local correto do arquivo em relação a este arquivo `.md`.
4. Salve o arquivo e visualize no GitHub (ou em um editor com preview de Markdown, como VS Code) para confirmar que a imagem aparece corretamente.
5. Execute os comandos `git add`, `git commit` e `git push` para incluir as imagens no controle de versão e enviá-las ao repositório remoto.

---

---

## Passo 3 — PD local por junta, sem compensação de gravidade

**Registre o erro de regime observado no TCP:**
`[ ... ]`

**Explique o comportamento do controlador durante a convergência:**
`[ ... ]`

**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/ctrl_joints_pd_sem_g.png`
- `~/kuka_control_plots/ctrl_error_torque_pd_sem_g.png`
- `~/kuka_control_plots/ctrl_tcp3d_pd_sem_g.png`
- `~/kuka_control_plots/ctrl_phase_pd_sem_g.png`

---

## Passo 4 — PD com compensação de gravidade

**Compare os resultados entre os arquivos `ctrl_error_torque_pd_sem_g.png` e `ctrl_error_torque_pd_com_g.png`:**
`[ ... ]`

**Explique o impacto da compensação de gravidade no erro e no torque aplicado:**
`[ ... ]`

**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/ctrl_joints_pd_com_g.png`
- `~/kuka_control_plots/ctrl_error_torque_pd_com_g.png`
- `~/kuka_control_plots/ctrl_tcp3d_pd_com_g.png`
- `~/kuka_control_plots/ctrl_phase_pd_com_g.png`


---

## Passo 5 — PD+G em posturas diferentes (pick x stretch)

**Registre as métricas obtidas:**

**RMS do erro do TCP (pick):**
`[ ... ]`

**RMS do erro do TCP (stretch):**
`[ ... ]`

**Sobressinal (pick):**
`[ ... ]`

**Sobressinal (stretch):**
`[ ... ]`

**Compare o desempenho entre as duas posturas:**
`[ ... ]`

**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/ctrl_joints_pdg_pick.png`
- `~/kuka_control_plots/ctrl_error_torque_pdg_pick.png`
- `~/kuka_control_plots/ctrl_tcp3d_pdg_pick.png`
- `~/kuka_control_plots/ctrl_phase_pdg_pick.png`
- `~/kuka_control_plots/ctrl_joints_pdg_stretch.png`
- `~/kuka_control_plots/ctrl_error_torque_pdg_stretch.png`
- `~/kuka_control_plots/ctrl_tcp3d_pdg_stretch.png`
- `~/kuka_control_plots/ctrl_phase_pdg_stretch.png`

---

## Passo 6 — Torque computado com modelo dinâmico completo

**Registre os resultados obtidos:**
`[ ... ]`

**Explique o desempenho do controlador utilizando o modelo dinâmico completo:**
`[ ... ]`

**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/ctrl_joints_ct.png`
- `~/kuka_control_plots/ctrl_error_torque_ct.png`
- `~/kuka_control_plots/ctrl_tcp3d_ct.png`
- `~/kuka_control_plots/ctrl_phase_ct.png`
---

## Passo 7 — Torque computado em posturas diferentes

**Registre as métricas obtidas:**

**RMS do erro do TCP (pick):**
`[ ... ]`

**RMS do erro do TCP (stretch):**
`[ ... ]`

**Compara a diferença observada entre as duas execuções:**
`[ ... ]`

**Compare com as diferenças observadas no Passo 5:**
`[ ... ]`

**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/ctrl_joints_ct_pick.png`
- `~/kuka_control_plots/ctrl_error_torque_ct_pick.png`
- `~/kuka_control_plots/ctrl_tcp3d_ct_pick.png`
- `~/kuka_control_plots/ctrl_phase_ct_pick.png`
- `~/kuka_control_plots/ctrl_joints_ct_stretch.png`
- `~/kuka_control_plots/ctrl_error_torque_ct_stretch.png`
- `~/kuka_control_plots/ctrl_tcp3d_ct_stretch.png`
- `~/kuka_control_plots/ctrl_phase_ct_stretch.png`

---

## Passo 8 — Controlador de espaço operacional

**Registre duas métricas contrastantes observadas nos logs:**
`[ ... ]`

**Explique os resultados obtidos:**
`[ ... ]`


**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/ctrl_joints_os_nominal.png`
- `~/kuka_control_plots/ctrl_error_torque_os_nominal.png`
- `~/kuka_control_plots/ctrl_tcp3d_os_nominal.png`
- `~/kuka_control_plots/ctrl_phase_os_nominal.png`*

---

## Passo 9 — Espaço nulo sem tarefa de postura

**Compare os gráficos `ctrl_joints_os_nominal.png` e `ctrl_joints_os_sem_postura.png`:**
`[ ... ]`

**Compare as diferenças observadas nas configurações articulares:**
`[ ... ]`


**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/ctrl_joints_os_nominal.png`
- `~/kuka_control_plots/ctrl_joints_os_sem_postura.png`
- `~/kuka_control_plots/ctrl_tcp3d_os_nominal.png`
- `~/kuka_control_plots/ctrl_tcp3d_os_sem_postura.png`


---

## Passo 10 — Comparação das três estratégias em condição de igualdade

**Compartilhe os dados presente na tabela:**
`[ ... ]`

### Tabela: Desempenho das estratégias local, centralizada e de espaço operacional sob a mesma sintonia `(wn, zeta)`, sem perturbações (`cmp_table_igualdade.txt`)

| Estratégia | RMS TCP [mm] | Pico TCP [mm] | Regime TCP [mm] | RMS junta [rad] | Assent. [s] | Sobressin. [%] | RMS τ [N·m] | Estável |
|---|---|---|---|---|---|---|---|---|
| Local (PD por junta) | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Centralizado (torque computado) | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Espaço operacional | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |

**Compare os resultados apresentados na tabela:**
`[ ... ]`

---

## Passo 11 — Comparação com payload não modelada

**Compartilhe os dados presente na tabela:**
`[ ... ]`

### Tabel: Desempenho das três estratégias com payload no flange não incluída no modelo do controlador (`cmp_table_payload.txt`)

| Estratégia | RMS TCP [mm] | Pico TCP [mm] | Regime TCP [mm] | RMS junta [rad] | Assent. [s] | Sobressin. [%] | RMS τ [N·m] | Estável |
|---|---|---|---|---|---|---|---|---|
| Local (PD por junta) | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Centralizado (torque computado) | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Espaço operacional | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |

**Compare os resultados com os obtidos no Passo 10:**
`[ ... ]`


---

## Passo 12 — Comparação com atrito de Coulomb não modelado

**Compartilhe os dados presente na tabela:**
`[ ... ]`

### Tabela: Desempenho das três estratégias com atrito de Coulomb nas juntas não incluído no modelo do controlador (`cmp_table_atrito.txt`)

| Estratégia | RMS TCP [mm] | Pico TCP [mm] | Regime TCP [mm] | RMS junta [rad] | Assent. [s] | Sobressin. [%] | RMS τ [N·m] | Estável |
|---|---|---|---|---|---|---|---|---|
| Local (PD por junta) | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Centralizado (torque computado) | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Espaço operacional | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
**Compare a coluna "regime TCP" com os resultados do Passo 10:**
`[ ... ]`



---

## Passo 13 — Execução no ambiente Gazebo

**Confirme a inicialização do robô e dos controladores:**
`[ ... ]`

**Inserir a imagem da captura de tela da simulação com o robô (`Captura da tela da simulação`):**
---

## Passo 14 — Desempenho do controlador no Gazebo

**Registre o RMS do erro do TCP (jtc_pick):**
`[ ... ]`

**Compare o resultado com os valores obtidos no Passo 10:**
`[ ... ]`

**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/jtc_joints_jtc_pick.png`
- `~/kuka_control_plots/jtc_error_jtc_pick.png`
- JSON: `~/kuka_control_plots/jtc_metrics_jtc_pick.json`

---

## Passo 15 — Dispersão do JTC real entre posturas

**Registre os valores observados:**

**RMS do erro do TCP (pick):**
`[ ... ]`

**RMS do erro do TCP (stretch):**
`[ ... ]`

**RMS do erro do TCP (left):**
`[ ... ]`

**Dispersão calculada ((máx − mín)/média):**
`[ ... ]`

**Compare com os experimentos simulados:**
`[ ... ]`


---

## Passo 16 — Redução da densidade de waypoints

**Explique o efeito da redução de waypoints sobre a trajetória:**
`[ ... ]`

**Comente sobre a presença de erros ou degradação da referência:**
`[ ... ]`

**Figuras:**
`[ inserir imagens ]`

**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/jtc_joints_jtc_poucos_wp.png`
- `~/kuka_control_plots/jtc_error_jtc_poucos_wp.png`
---

## Passo 17 — Comparação dos erros de rastreamento

**Compartilhe os dados da tabela comparativa:**
`[ ... ]`
### Tabela: Erro de rastreamento do TCP nas três estratégias simuladas e no `JointTrajectoryController` real do Gazebo, na mesma trajetória home → pick

Valores das linhas simuladas: `cmp_table_igualdade.txt` (Passo 10). Valores da linha do Gazebo: `jtc_metrics_jtc_pick.json` (Passo 14).

| Controlador | Origem | RMS TCP [mm] | Pico TCP [mm] | Regime TCP [mm] | RMS junta [rad] |
|---|---|---|---|---|---|
| PD local (PD por junta) | Simulação | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Torque computado (centralizado) | Simulação | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Espaço operacional | Simulação | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| JTC do Gazebo | Robô simulado no Gazebo | [ ... ] | [ ... ] | [ ... ] | [ ... ] |

**Registre a conclusão sobre qual estratégia mais se aproxima do controlador real:**
`[ ... ]`

---

## Passo 18 — Efeito do ruído de encoder sobre o ganho derivativo

**Compare os gráficos `ctrl_error_torque_ruido_*.png`:**
`[ ... ]`

**Explique o impacto do ruído sobre o termo derivativo:**
`[ ... ]`

**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/ctrl_error_torque_ruido_leve.png`
- `~/kuka_control_plots/ctrl_error_torque_ruido_forte.png`

---

## Passo 19 — Monitoramento da estabilidade em tempo real

**Registre os alertas observados (divergência, oscilação e chatter):**
`[ ... ]`

**Anote as observações sobre os alertas:**
`[ ... ]`

---

## Passo 20 — Linha de base do monitor com o robô parado

**Registre as métricas:**

**Pico de erro observado:**
`[ ... ]`

**Fração de alta frequência:**
`[ ... ]`

**Número de inversões:**
`[ ... ]`

**Explique os valores registrados:**
`[ ... ]`

---

## Passo 21 — Movimento agressivo e disparo de alertas

**Compare os resultados com a linha de base do Passo 20:**
`[ ... ]`

---

## Passo 22 — Controle de posição pura contra a superfície

**Registre a força de regime observada:**
`[ ... ]`

**Explique os dados presente no painel central de `force_run_pos_madeira.png`:**
`[ ... ]`

**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/force_run_pos_madeira.png`


---

## Passo 23 — Controle de posição em diferentes materiais

**Registre as forças de regime observadas:**

**Espuma:**
`[ ... ]`

**Madeira:**
`[ ... ]`

**Aço:**
`[ ... ]`

**Compare os resultados entre os três materiais:**
`[ ... ]`

**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/force_run_pos_espuma.png`
- `~/kuka_control_plots/force_run_pos_madeira.png`
- `~/kuka_control_plots/force_run_pos_aco.png`

---

## Passo 24 — Controle híbrido força/posição

**Registre as métricas:**

**Força de regime observada:**
`[ ... ]`

**Ondulação observada:**
`[ ... ]`

**Explique o desempenho do controlador híbrido:**
`[ ... ]`

**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/force_run_hibrido_madeira.png`
- `~/kuka_control_plots/force_torque_hibrido_madeira.png`
---

## Passo 25 — Comparação das quatro leis de interação

**Registre os dados tabela que foi gerada:**
`[ ... ]`

### Tabela: Contato com a madeira: posição pura, impedância, admitância e híbrido força/posição, com o mesmo comando abaixo da superfície e `F_des` = 20 N (`fcmp_table_madeira.txt`)

| Lei de interação | Pico [N] | Regime [N] | Erro [N] | Ondul. [N] | Penetr. [mm] | τ pico [N·m] |
|---|---|---|---|---|---|---|
| Posição pura | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Impedância | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Admitância | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Híbrido força/posição | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |

**Compare os resultados entre as estratégias avaliadas:**
`[ ... ]`

**Insira as imagens dos gráficos presentes em `~/kuka_control_plots/`:**

- `~/kuka_control_plots/fcmp_force_log_madeira.png`
- `~/kuka_control_plots/fcmp_force_madeira.png`
- `~/kuka_control_plots/fcmp_height_madeira.png`
- `~/kuka_control_plots/fcmp_summary_madeira.png`


---

## Passo 26 — Comparação entre os três materiais: 

**Registre as tabelas dos três materiais (espuma, madeira e aço):**
`[ ... ]`


### Tabela:  Contato com espuma (baixa rigidez): comparação das quatro leis de interação (`fcmp_table_espuma.txt`)

| Lei de interação | Pico [N] | Regime [N] | Erro [N] | Ondul. [N] | Penetr. [mm] | τ pico [N·m] |
|---|---|---|---|---|---|---|
| Posição pura | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Impedância | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Admitância | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Híbrido força/posição | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |

### Tabela: Contato com madeira (rigidez intermediária): comparação das quatro leis de interação (`fcmp_table_madeira.txt`)

| Lei de interação | Pico [N] | Regime [N] | Erro [N] | Ondul. [N] | Penetr. [mm] | τ pico [N·m] |
|---|---|---|---|---|---|---|
| Posição pura | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Impedância | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Admitância | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Híbrido força/posição | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |

### Tabela: Contato com aço (alta rigidez): comparação das quatro leis de interação (`fcmp_table_aco.txt`)

| Lei de interação | Pico [N] | Regime [N] | Erro [N] | Ondul. [N] | Penetr. [mm] | τ pico [N·m] |
|---|---|---|---|---|---|---|
| Posição pura | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Impedância | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Admitância | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |
| Híbrido força/posição | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] | [ ... ] |


**Explique quais métricas foram mais sensíveis à mudança de material:**
`[ ... ]`

**Compare os resultados obtidos entre os materiais:**
`[ ... ]`

