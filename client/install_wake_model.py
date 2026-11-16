#!/usr/bin/env python3
"""Scarica il modello italiano Vosk dal sito ufficiale, solo durante il setup."""
from pathlib import Path
import tempfile
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parent / 'models'
NAME = 'vosk-model-small-it-0.22'
URL = f'https://alphacephei.com/vosk/models/{NAME}.zip'


def main():
    if (ROOT / NAME / 'am/final.mdl').is_file():
        print('Modello wake word italiano già installato.')
        return
    ROOT.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=ROOT) as tmp:
        tmp = Path(tmp)
        archive = tmp / 'model.zip'
        print('Download modello italiano Vosk (circa 48 MB)…', flush=True)
        with urllib.request.urlopen(URL, timeout=60) as response, archive.open('wb') as out:
            size = 0
            while chunk := response.read(1024 * 1024):
                size += len(chunk)
                if size > 100_000_000:
                    raise ValueError('Archivio oltre la dimensione prevista')
                out.write(chunk)
        with zipfile.ZipFile(archive) as z:
            if sum(i.file_size for i in z.infolist()) > 200_000_000:
                raise ValueError('Modello oltre la dimensione prevista')
            for item in z.infolist():
                path = (tmp / item.filename).resolve()
                if not path.is_relative_to(tmp.resolve()) or not path.is_relative_to((tmp / NAME).resolve()):
                    raise ValueError('Percorso non valido nel modello')
            z.extractall(tmp)
        model = tmp / NAME
        if not (model / 'am/final.mdl').is_file():
            raise ValueError('Modello italiano incompleto')
        model.rename(ROOT / NAME)
    print('Wake word pronta. Collega il client e premi «Ascolto Sonia».')


if __name__ == '__main__':
    main()
