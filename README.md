# addon-super-cleanup

Addon Blender para limpar superfícies planas selecionadas.

## Build/Distribuição

1. Garanta que o código-fonte esteja atualizado e salvo em `dissolve.py`.
2. Crie uma pasta `addon_super_cleanup` (o nome do módulo instalável) e copie o conteúdo de `dissolve.py` para um arquivo `__init__.py` dentro dela.
3. Gere o pacote zip contendo essa pasta. Exemplo de linha de comando:
   ```bash
   python - <<'PY'
   import pathlib, shutil, zipfile, tempfile

   root = pathlib.Path(__file__).parent
   src = root / "dissolve.py"
   with tempfile.TemporaryDirectory() as tmp:
       pkg_root = pathlib.Path(tmp) / "addon_super_cleanup"
       pkg_root.mkdir()
       shutil.copy(src, pkg_root / "__init__.py")
       with zipfile.ZipFile(root / "addon-super-cleanup.zip", "w", zipfile.ZIP_DEFLATED) as zf:
           for path in pkg_root.rglob("*"):
               zf.write(path, path.relative_to(tmp))
   PY
   ```
4. Instale no Blender via **Edit > Preferences > Add-ons > Install…** escolhendo o `addon-super-cleanup.zip`.
5. Alternativa rápida: mantenha o script como arquivo único (`dissolve.py`) e utilize **Text Editor > Run Script** para carregar o addon apenas na sessão atual.

Também é possível usar o script auxiliar `make_zip.py` incluído no repositório:
```bash
python make_zip.py
```
Ele gera `addon-super-cleanup.zip` na raiz do projeto.

## Versionamento

- Atualize `bl_info["version"]` (tupla `major, minor, patch`) em `dissolve.py` para refletir cada release.
- Opcional: alinhe o nome do arquivo zip com a versão (ex.: `addon-super-cleanup-1.1.0.zip`) caso deseje distribuir múltiplas versões.

## Checklist antes de publicar

- Testar o addon no Blender 3.6 ou superior (preferencialmente 3.6 LTS e última versão estável disponível).
- Verificar se não há dependências externas além da API padrão do Blender.
- Conferir que o painel aparece em **View3D > Sidebar (N) > Mesh > Flat Surface Cleaner** e que as operações funcionam em malhas selecionadas.
- Validar mensagens e descrição no painel para evitar traduções incorretas.
- Regenerar o pacote zip e confirmar se instala/ativa sem erros em uma instalação limpa do Blender.
