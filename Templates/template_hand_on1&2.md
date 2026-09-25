# Registro de Atividade — Manipulação, Estimativa e Controle (Passos 3 a 20)

**Estudante(s):** `[ ... ]`

> Preencha cada seção com os dados/valores solicitados, suas observações e as figuras geradas. Substitua os campos `[ ... ]` pelas suas respostas e insira as imagens conforme o tutorial abaixo.

---

## Como inserir imagens no Markdown

Para inserir uma imagem neste arquivo, use a seguinte sintaxe:

```markdown
![Texto alternativo](caminho/da/imagem.png)
```

Passo a passo:

1. Copie o arquivo de imagem gerado (por exemplo, em `~/kuka_fk_plots/` ou `~/kuka_kf_plots/`) para uma pasta do seu repositório, como `imagens/`, para que ela fique versionada junto com o projeto.
2. No local desejado do template, escreva o caminho relativo até a imagem. Exemplo:

   ```markdown
   ![Comparação da cinemática direta - pick](imagens/fk_comparison_xyz_pick.png)
   ```

3. O texto entre colchetes `[ ]` é o texto alternativo (descrição da imagem) — use algo breve e descritivo.
4. O caminho entre parênteses `( )` deve apontar para o local correto do arquivo em relação a este `.md`.
5. Salve o arquivo e visualize no GitHub (ou em um editor com preview de Markdown, como VS Code) para confirmar que a imagem aparece corretamente.
6. Não esqueça de rodar `git add`, `git commit` e `git push` para que as imagens também sejam enviadas ao repositório remoto.

---

## Passo 3 — Inicialização do Gazebo com o robô KUKA KR70 R2100

**Confirme a inicialização do robô e do gripper Robotiq 2F-85 na simulação:**
`[ ... ]`

**Figura (captura de tela da simulação):**
`[ inserir imagem ]`

---

## Passo 4 — Nó de cinemática direta (fk_node) em modo de consulta pontual

**Registre a pose do TCP obtida para a configuração `custom_q = [0.0, -1.0, 1.2, 0.0, 0.8, 0.0]`:**
`[ ... ]`

**Analise o resultado obtido:**
`[ ... ]`

---

## Passo 5 — Inserção de objetos no ambiente de simulação (spawn_objects)

**Confirme os elementos carregados na simulação (mesa, objetos manipuláveis e caixa de destino):**
`[ ... ]`

**Figuras:**
`[ inserir imagens ]`

---

## Passo 6 — Execução das 8 tarefas de pick-and-place

**Registre o resultado de cada uma das 8 tarefas (sucesso ou falha na coleta, no transporte e na deposição):**

**`red_box`:**
`[ ... ]`

**`blue_cylinder`:**
`[ ... ]`

**`green_sphere`:**
`[ ... ]`

**`yellow_bottle`:**
`[ ... ]`

**`orange_can`:**
`[ ... ]`

**`pink_cube`:**
`[ ... ]`

**`gray_puck`:**
`[ ... ]`

**`teal_block`:**
`[ ... ]`

**Observações gerais sobre o comportamento do manipulador durante as tarefas:**
`[ ... ]`

---

## Passo 7 — Gráficos estáticos de cinemática direta (fk_plotter)

**Registre as configurações de referência analisadas (`home`, `pick`, `place`, `stretch`, `left`, `custom`):**
`[ ... ]`

**Analise as figuras geradas:**
`[ ... ]`

**Figuras (4 gráficos em `~/kuka_fk_plots/`: `fk_trajectory_3d.png`, `fk_xyz_euler.png`, `fk_workspace_slice.png`, `fk_robot_2d.png`):**
`[ inserir imagens ]`

---

## Passo 8 — Reinício da simulação

**Confirme o reinício do Gazebo (repetição dos Passos 3 e 5):**
`[ ... ]`

---

## Passo 9 — Comparação da cinemática direta durante o movimento (fk_comparison)

**Registre o erro de rastreamento observado para cada uma das 8 manipulações:**

**`red_box`:**
`[ ... ]`

**`blue_cylinder`:**
`[ ... ]`

**`green_sphere`:**
`[ ... ]`

**`yellow_bottle`:**
`[ ... ]`

**`orange_can`:**
`[ ... ]`

**`pink_cube`:**
`[ ... ]`

**`gray_puck`:**
`[ ... ]`

**`teal_block`:**
`[ ... ]`

**Compare os valores do erro de rastreamento entre as 8 tarefas monitoradas:**
`[ ... ]`

**Figuras (4 gráficos por tarefa — `fk_comparison_xyz_*.png`, `fk_comparison_error_*.png`, `fk_comparison_3d_*.png`, `fk_joint_angles_*.png`):**
`[ inserir imagens ]`

---

## Passo 10 — Estimação de estado offline (linha de base)

**Registre os resultados obtidos com o filtro de Kalman sob ruído gaussiano reprodutível:**
`[ ... ]`

**Analise se o filtro conseguiu estimar corretamente os estados do manipulador:**
`[ ... ]`

**Figuras (8 gráficos em `~/kuka_kf_plots/`):**
`[ inserir imagens ]`

---

## Passo 11 — Estimação offline com ruído elevado e outliers

**Registre os resultados obtidos (`sigma_pos:=0.02`, `outlier_prob:=0.02`):**
`[ ... ]`

**Analise o impacto do ruído e dos outliers sobre a estimativa de movimento:**
`[ ... ]`

**Figuras:**
`[ inserir imagens ]`

---

## Passo 12 — Estimação offline com viés constante

**Registre os resultados obtidos (`bias:=0.02`):**
`[ ... ]`

**Compare os resultados com os obtidos nos Passos 10 e 11:**
`[ ... ]`

**Figuras:**
`[ inserir imagens ]`

---

## Passo 13 — Pipeline completo de estimação online

**Confirme a inicialização simultânea dos três nós (`noise_injector`, `state_estimator` e `estimation_plotter`):**
`[ ... ]`

---

## Passo 14 — Movimentação do robô durante a estimação online

**Descreva o comportamento do pipeline de estimação online durante a execução do pick-and-place:**
`[ ... ]`

---

## Passo 15 — Encerramento do pipeline online e análise dos resultados

**Compare os gráficos gerados com os dos Passos 10, 11 e 12:**
`[ ... ]`

**Figuras (8 gráficos em `~/kuka_kf_plots/`):**
`[ inserir imagens ]`

---

## Passo 16 — Monitoramento do esforço (effort) por junta

**Registre os valores máximos de `effort` observados em cada junta durante a tarefa `red_box`:**

**Junta 1:**
`[ ... ]`

**Junta 2:**
`[ ... ]`

**Junta 3:**
`[ ... ]`

**Junta 4:**
`[ ... ]`

**Junta 5:**
`[ ... ]`

**Junta 6:**
`[ ... ]`

**Analise como o torque varia entre as juntas ao longo da tarefa:**
`[ ... ]`

---

## Passo 17 — Reinício da simulação

**Confirme o reinício do Gazebo (repetição dos Passos 3 e 5):**
`[ ... ]`

---

## Passo 18 — Comparação do torque entre tarefas diferentes

**Registre os valores de pico de `effort` observados em cada junta durante a tarefa `orange_can`:**

**Junta 1:**
`[ ... ]`

**Junta 2:**
`[ ... ]`

**Junta 3:**
`[ ... ]`

**Junta 4:**
`[ ... ]`

**Junta 5:**
`[ ... ]`

**Junta 6:**
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