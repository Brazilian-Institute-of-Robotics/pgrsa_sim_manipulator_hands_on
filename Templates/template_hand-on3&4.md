# Registro de Atividade — Dinâmica e Controle do Manipulador (Passos 3 a 27)

**Estudante(s):** `[ ... ]`

> Preencha cada seção com os dados/valores solicitados, suas observações e as figuras geradas. Substitua os campos `[ ... ]` pelas suas respostas e insira as imagens conforme o tutorial abaixo.

---

## Como inserir imagens no Markdown

Para inserir uma imagem neste arquivo, use a seguinte sintaxe:

```markdown
!caminho/da/imagem.png
```

Passo a passo:

1. Copie o arquivo de imagem gerado (por exemplo, em `~/kuka_control_plots/`) para uma pasta do seu repositório, como `imagens/`, para que ela fique versionada junto com o projeto.
2. No local desejado do template, escreva o caminho relativo até a imagem. Exemplo:

   ```markdown
   ![Erro de torque PD sem compensação de gravidade](imagens/entre colchetes `[ ]` é o texto alternativo (descrição da imagem) — use algo breve e descritivo.
4. O caminho entre parênteses `( )` deve apontar para o local correto do arquivo em relação a este `.md`.
5. Salve o arquivo e visualize no GitHub (ou em um editor com preview de Markdown, como VS Code) para confirmar que a imagem aparece corretamente.
6. Não esqueça de rodar `git add`, `git commit` e `git push` para que as imagens também sejam enviadas ao repositório remoto.

---

## Passo 3 — PD local por junta, sem compensação de gravidade

**Registre o erro de regime observado no TCP:**
`[ ... ]`

**Analise o comportamento do controlador durante a convergência:**
`[ ... ]`

**Figuras (4 gráficos em `~/kuka_control_plots/`):**
`[ inserir imagens ]`

---

## Passo 4 — PD com compensação de gravidade

**Compare os resultados entre os arquivos `ctrl_error_torque_pd_sem_g.png` e `ctrl_error_torque_pd_com_g.png`:**
`[ ... ]`

**Analise o impacto da compensação de gravidade no erro e no torque aplicado:**
`[ ... ]`

**Figuras:**
`[ inserir imagens ]`


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

**Figuras:**
`[ inserir imagens ]`

---

## Passo 6 — Torque computado com modelo dinâmico completo

**Registre os resultados obtidos:**
`[ ... ]`

**Analise o desempenho do controlador utilizando o modelo dinâmico completo:**
`[ ... ]`

**Figuras:**
`[ inserir imagens ]`


---

## Passo 7 — Torque computado em posturas diferentes

**RMS do erro do TCP (pick):**
`[ ... ]`

**RMS do erro do TCP (stretch):**
`[ ... ]`

**Diferença observada entre as duas execuções:**
`[ ... ]`

**Compare com as diferenças observadas no Passo 5:**
`[ ... ]`

**Figuras:**
`[ inserir imagens ]`


---

## Passo 8 — Controlador de espaço operacional

**Registre duas métricas contrastantes observadas nos logs:**
`[ ... ]`

**Analise os resultados obtidos:**
`[ ... ]`

**Figuras:**
`[ inserir imagens ]`



---

## Passo 9 — Espaço nulo sem tarefa de postura

**Compare os gráficos `ctrl_joints_os_nominal.png` e `ctrl_joints_os_sem_postura.png`:**
`[ ... ]`

**Analise as diferenças observadas nas configurações articulares:**
`[ ... ]`

**Figuras:**
`[ inserir imagens ]`


---

## Passo 10 — Comparação das três estratégias em condição de igualdade

**Registre a tabela gerada:**
`[ ... ]`

**Analise os resultados apresentados na tabela:**
`[ ... ]`

---

## Passo 11 — Comparação com payload não modelada

**Registre a tabela gerada:**
`[ ... ]`

**Compare os resultados com os obtidos no Passo 10:**
`[ ... ]`


---

## Passo 12 — Comparação com atrito de Coulomb não modelado

**Registre a tabela gerada:**
`[ ... ]`

**Compare a coluna "regime TCP" com os resultados do Passo 10:**
`[ ... ]`



---

## Passo 13 — Subida do ambiente no Gazebo

**Confirme a inicialização do robô e dos controladores:**
`[ ... ]`

**Figuras (`Captura da tela da simulação`):**
`[ inserir imagens ]`

---

## Passo 14 — Desempenho do controlador no Gazebo

**RMS do erro do TCP (jtc_pick):**
`[ ... ]`

**Compare o resultado com os valores obtidos no Passo 10:**
`[ ... ]`

**Figuras (2 gráficos e JSON gerado):**
`[ inserir imagens/arquivo ]`


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

**Analise o efeito da redução de waypoints sobre a trajetória:**
`[ ... ]`

**Comente sobre a presença de "cantos" ou degradação da referência:**
`[ ... ]`

**Figuras:**
`[ inserir imagens ]`

---

## Passo 17 — Confronto entre números do Gazebo e da simulação

**Registre a tabela comparativa:**
`[ ... ]`

**(PD local simulado, torque computado simulado, espaço operacional simulado e JTC do Gazebo)**

**Conclusão sobre qual estratégia mais se aproxima do controlador real:**
`[ ... ]`

---

## Passo 18 — Efeito do ruído de encoder sobre o ganho derivativo

**Compare os gráficos `ctrl_error_torque_ruido_*.png`:**
`[ ... ]`

**Analise o impacto do ruído sobre o termo derivativo:**
`[ ... ]`

**Figuras:**
`[ inserir imagens ]`


---

## Passo 19 — Monitoramento da estabilidade em tempo real

**Registre os alertas observados (divergência, oscilação e chatter):**
`[ ... ]`

**Observações:**
`[ ... ]`

---

## Passo 20 — Linha de base do monitor com o robô parado

**Pico de erro observado:**
`[ ... ]`

**Fração de alta frequência:**
`[ ... ]`

**Número de inversões:**
`[ ... ]`

**Analise os valores registrados:**
`[ ... ]`

---

## Passo 21 — Movimento agressivo e disparo de alertas

**Compare os resultados com a linha de base do Passo 20:**
`[ ... ]`

---

## Passo 22 — Controle de posição pura contra a superfície

**Registre a força de regime observada:**
`[ ... ]`

**Analise o painel central de `force_run_pos_madeira.png`:**
`[ ... ]`

**Figura:**
`[ inserir imagem ]`


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

**Figuras:**
`[ inserir imagens ]`


---

## Passo 24 — Controle híbrido força/posição

**Força de regime observada:**
`[ ... ]`

**Ondulação observada:**
`[ ... ]`

**Analise o desempenho do controlador híbrido:**
`[ ... ]`

**Figuras:**
`[ inserir imagens ]`

---

## Passo 25 — Comparação das quatro leis de interação

**Registre a tabela gerada (pico, regime, erro, ondulação, penetração e torque de pico):**
`[ ... ]`

**Compare os resultados entre as estratégias avaliadas:**
`[ ... ]`

**Figura (`fcmp_force_log_madeira.png`) e demais gráficos:**
`[ inserir imagens ]`

---

## Passo 26 — Comparação nos três materiais

**Registre as tabelas dos três materiais (espuma, madeira e aço):**
`[ ... ]`

**Analise quais métricas foram mais sensíveis à mudança de material:**
`[ ... ]`

**Compare os resultados obtidos entre os materiais:**
`[ ... ]`

