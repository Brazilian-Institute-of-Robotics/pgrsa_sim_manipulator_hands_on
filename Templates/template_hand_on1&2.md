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
4. Salve o arquivo e visualize no GitHub (ou em um editor com _preview_ de Markdown, como VS Code) para confirmar que a imagem aparece corretamente.
5. Execute os comandos `git add`, `git commit` e `git push` para incluir as imagens no controle de versão e enviá-las ao repositório remoto.


## Passo 3 — Inicialização do Gazebo com o robô KUKA KR70 R2100

**Confirme a inicialização do robô e do gripper Robotiq 2F-85 na simulação:**
`[ ... ]`

**Insira uma captura de tela da simulação, na qual o robô esteja visível (`Captura da tela da simulação`):**
`[ inserir imagens ]`

---

## Passo 4 — Nó de cinemática direta (fk_node) em modo de consulta pontual

**Registre a pose do TCP obtida para a configuração `custom_q = [0.0, -1.0, 1.2, 0.0, 0.8, 0.0]`:**
`[ ... ]`

**Explique o resultado obtido:**
`[ ... ]`

---

## Passo 5 — Inserção de objetos no ambiente de simulação (spawn_objects)

**Confirme os elementos carregados na simulação (mesa, objetos manipuláveis e caixa de destino):**
`[ ... ]`

**Insira a captura de tela da simulação, mostrando o robô e os objetos presentes no ambiente (`Captura da tela da simulação com a presença dos objetos`):**
`[ inserir imagens ]`

---

## Passo 6 — Execução das 8 tarefas de _pick-and-place_

**Registre o resultado de cada uma das 8 tarefas (sucesso ou falha na coleta, no transporte e na deposição):**

- **`red_box`:**
`[ ... ]`

- **`blue_cylinder`:**
`[ ... ]`

- **`green_sphere`:**
`[ ... ]`

- **`yellow_bottle`:**
`[ ... ]`

- **`orange_can`:**
`[ ... ]`

- **`pink_cube`:**
`[ ... ]`

- **`gray_puck`:**
`[ ... ]`

- **`teal_block`:**
`[ ... ]`

**Insira as observações gerais sobre o comportamento do manipulador durante as tarefas:**
`[ ... ]`

---

## Passo 7 — Gráficos estáticos de cinemática direta (fk_plotter)

**Registre as configurações de referência analisadas (`home`, `pick`, `place`, `stretch`, `left`, `custom`):**
`[ ... ]`

**Explique as figuras geradas:**
`[ ... ]`

**Insira as imagens as imagens dos 4 gráficos em `~/kuka_fk_plots/`: `fk_trajectory_3d.png`, `fk_xyz_euler.png`, `fk_workspace_slice.png`, `fk_robot_2d.png`:**
`[ inserir imagens ]`

---

## Passo 8 — Reinício da simulação

**Insira a captura de tela da simulação, mostrando o robô e os objetos presentes no ambiente (`Captura da tela da simulação com a presença dos objetos`):**
`[ inserir imagens ]`

---

## Passo 9 — Comparação da cinemática direta durante o movimento (fk_comparison)

**Registre o erro de rastreamento observado para cada uma das 8 manipulações:**

- **`red_box`:**
`[ ... ]`

- **`blue_cylinder`:**
`[ ... ]`

- **`green_sphere`:**
`[ ... ]`

 - **`yellow_bottle`:**
`[ ... ]`

- **`orange_can`:**
`[ ... ]`

- **`pink_cube`:**
`[ ... ]`

- **`gray_puck`:**
`[ ... ]`

- **`teal_block`:**
`[ ... ]`

**Compare os valores do erro de rastreamento entre as 8 tarefas monitoradas:**
`[ ... ]`

**Insira as 4 imagens de gráficos por tarefa :( `fk_comparison_xyz_*.png`, `fk_comparison_error_*.png`, `fk_comparison_3d_*.png`, `fk_joint_angles_*.png`):**
`[ inserir imagens ]`

---

## Passo 10 — Estimação de estado _offline_ (linha de base)

**Registre os resultados obtidos com o filtro de Kalman sob ruído gaussiano reprodutível:**
`[ ... ]`

**Explique se o filtro conseguiu estimar corretamente os estados do manipulador:**
`[ ... ]`

**Insira as 8 imagens de gráficos presente em `~/kuka_kf_plots/`:**
`[ inserir imagens ]`

---

## Passo 11 — Estimação _offline_ com ruído elevado e outliers

**Registre os resultados obtidos (`sigma_pos:=0.02`, `outlier_prob:=0.02`):**
`[ ... ]`

**Explique o impacto do ruído e dos outliers sobre a estimativa de movimento:**
`[ ... ]`

**Insira as 8 imagens de gráficos presente em `~/kuka_kf_plots/`:**
`[ inserir imagens ]`

---

## Passo 12 — Estimação _offline_ com viés constante

**Registre os resultados obtidos (`bias:=0.02`):**
`[ ... ]`

**Compare os resultados com os obtidos nos Passos 10 e 11:**
`[ ... ]`

**Insira as 8 imagens de gráficos presente em `~/kuka_kf_plots/`**
`[ inserir imagens ]`

---

## Passo 13 — _Pipeline_ completo de estimação _online_

**Confirme a inicialização simultânea dos três nós (`noise_injector`, `state_estimator` e `estimation_plotter`):**
`[ ... ]`

---

## Passo 14 — Movimentação do robô durante a estimação _online_

**Descreva o comportamento do _pipeline_ de estimação _online_ durante a execução do _pick-and-place_:**
`[ ... ]`

---

## Passo 15 — Encerramento do _pipeline_ _online_ e análise dos resultados

**Compare os gráficos gerados com os dos Passos 10, 11 e 12:**
`[ ... ]`

**Insira as 8 imagens de gráficos presente em `~/kuka_kf_plots/`**

---

## Passo 16 — Monitoramento do esforço (_effort_) por junta

**Registre os valores máximos de `effort` observados em cada junta durante a tarefa `red_box`:**

- **Junta 1:**
`[ ... ]`

- **Junta 2:**
`[ ... ]`

- **Junta 3:**
`[ ... ]`

- **Junta 4:**
`[ ... ]`

- **Junta 5:**
`[ ... ]`

- **Junta 6:**
`[ ... ]`

**Explique como o torque varia entre as juntas ao longo da tarefa:**
`[ ... ]`

---

## Passo 17 — Reinício da simulação

**Insira a captura de tela da simulação, mostrando o robô e os objetos presentes no ambiente (`Captura da tela da simulação com a presença dos objetos`):**
`[ inserir imagens ]`

---

## Passo 18 — Comparação do torque entre tarefas diferentes

**Registre os valores de pico de `effort` observados em cada junta durante a tarefa `orange_can`:**

- **Junta 1:**
`[ ... ]`

- **Junta 2:**
`[ ... ]`

- **Junta 3:**
`[ ... ]`

- **Junta 4:**
`[ ... ]`

- **Junta 5:**
`[ ... ]`

- **Junta 6:**
`[ ... ]`

**Compare os valores obtidos com os registrados no Passo 16:**
`[ ... ]`

---

## Passo 19 — Comparação do erro de trajetória entre tarefas

**Registre os valores de erro obtidos no tópico `/kuka_arm_controller/follow_joint_trajectory/_action/feedback` para cada uma das duas tarefas monitoradas:**
`[ ... ]`

**Compare os valores de erro obtidos entre as duas tarefas:**
`[ ... ]`

---

## Passo 20 — Finalização e versionamento

**Confirme a atualização do arquivo `template_hand-on1.md`, sua renomeação para o formato `seu_nome_completo_hand_on1&2.md` e o envio via `git add`, `git commit` e `git push`:**
`[ ... ]`