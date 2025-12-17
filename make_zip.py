#!/usr/bin/env python3
"""Gera o pacote instalável do addon como arquivo zip.

Uso:
    python make_zip.py

Cria `flat-surface-cleaner.zip` contendo a pasta `flat_surface_cleaner` com
`__init__.py` (copiado de `dissolve.py`).
"""

from __future__ import annotations

import shutil
import tempfile
import zipfile
from pathlib import Path

ROOT = Path(__file__).parent
SRC = ROOT / "dissolve.py"
PACKAGE_NAME = "flat_surface_cleaner"
ZIP_NAME = ROOT / "flat-surface-cleaner.zip"


def build_zip() -> Path:
    if not SRC.exists():
        raise FileNotFoundError(f"Código-fonte não encontrado: {SRC}")

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)
        package_root = tmp_path / PACKAGE_NAME
        package_root.mkdir(parents=True, exist_ok=True)

        shutil.copy(SRC, package_root / "__init__.py")

        with zipfile.ZipFile(ZIP_NAME, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            for path in package_root.rglob("*"):
                arcname = path.relative_to(tmp_path)
                zf.write(path, arcname)

    return ZIP_NAME


def main() -> None:
    zip_path = build_zip()
    print(f"Arquivo gerado: {zip_path}")


if __name__ == "__main__":
    main()
