# Flat Surface Cleaner

Add-on para Blender que limpa superfícies planas: ajusta o plano da seleção, reconstrói como uma única face sem geometria interna e aplica correções opcionais de contorno e normais. Útil para corrigir superfícies degradadas por booleans, importações de CAD ou modelagem hard-surface.

## Compatibilidade e requisitos
- **Blender:** testado para **3.6+** (usa API `bpy` padrão, sem dependências externas). 
- **Modo de uso:** funciona no **Edit Mode** com objetos de malha.
- **Arquivo:** `dissolve.py` deve ser instalado como add-on.

## Instalação e ativação
1. Baixe o arquivo `dissolve.py` deste repositório (menu **Code > Download ZIP** ou botão **Raw** para salvar o `.py`).
2. No Blender, abra **Edit > Preferences… > Add-ons**.
3. Clique em **Install…** e selecione o `dissolve.py` baixado.
4. Marque a caixa para ativar **Flat Surface Cleaner**.
5. No 3D Viewport, pressione **N** para abrir a Sidebar, aba **Mesh**. O painel aparece como **Flat Surface Cleaner**.

## Localização do painel
`View3D > Sidebar (N) > Mesh > Flat Surface Cleaner`

## Opções da interface e quando usar
- **Plano de Referência** (`BEST_FIT`, `ACTIVE`, `AVERAGE`):
  - *Melhor Ajuste:* calcula plano de regressão pelos vértices; ideal para superfícies tortas que precisam ser replanarizadas sem referência clara.
  - *Face Ativa:* usa a normal/centro da face ativa; bom para alinhar toda a seleção a uma face “guia”.
  - *Média das Normais:* media ponderada das faces selecionadas; útil quando há várias faces coplanares com pequenos desvios.
- **Usar Apenas o Maior Contorno:** mantém só o loop de maior área quando há múltiplos contornos; ajuda a fechar furos ou ignorar ilhas pequenas.
- **Weld no Contorno:** mescla vértices muito próximos antes de recriar a face; previne duplicatas pós-boolean ou import.
  - **Distância Weld:** raio usado no weld; aumente levemente se ainda restarem duplos, reduza se colapsar detalhes.
- **Simplificar Contorno:** dissolve vértices colineares no perímetro para limpar contornos com muitos pontos.
  - **Tolerância (°):** controla a agressividade; valores baixos preservam curvas leves, altos removem mais vértices.
- **Recalcular Normais:** recalcula a normal da face resultante; mantenha ativo para shading correto, desative se quiser manter a orientação manual.

## Fluxos de trabalho recomendados
1. **Limpar superfície planar importada:** selecione faces da região plana, defina `Plano de Referência = Melhor Ajuste`, mantenha *Weld* ativo e simplificação desligada; execute o operador.
2. **Alinhar a uma face guia:** selecione uma face “boa”, torná-la ativa, selecione faces vizinhas tortas, escolha `Face Ativa`, ative *Usar Apenas o Maior Contorno* para eliminar furos, e execute.
3. **Reduzir vértices de contorno:** para contornos densos de CAD, ative *Simplificar Contorno* com tolerância baixa (0.2–0.5°) antes de planarizar.

## Limitações conhecidas
- Contornos não-manifold ou auto-intersectantes podem impedir a criação da face única.
- Loops abertos (bordas com buracos) cancelam a operação; feche as bordas ou use *Usar Apenas o Maior Contorno* se houver múltiplos loops.
- Seleções com menos de três vértices não podem ser planarizadas.

## Troubleshooting
- **Erro ao criar face única:** verifique se o contorno é fechado e não possui auto-interseções; tente reduzir a tolerância do weld ou desativar *Simplificar Contorno*.
- **Face invertida/escura:** habilite *Recalcular Normais* ou use `Alt+N > Flip` após a operação.
- **Buracos permanecem:** certifique-se de que *Usar Apenas o Maior Contorno* está ativo para ignorar ilhas internas; caso contrário, feche manualmente com `F` ou `Grid Fill`.
- **Detalhes sumindo após weld:** diminua **Distância Weld** até preservar elementos finos.

## FAQ
- **Preciso estar no Edit Mode?** Sim, o operador só aparece e funciona no Edit Mode com malhas.
- **Funciona em objetos não-mesh?** Não; converta para mesh (`Alt+C` ou `Object > Convert To`).
- **Posso usar em superfícies curvas?** O add-on força a planarização; para curvas, use `Shrinkwrap` ou retopo manual.

## Licença
MIT (vide `LICENSE`).
